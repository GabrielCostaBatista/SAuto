#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import os
import yaml


class ArucoCompressedDetector:
    def __init__(self):
        rospy.init_node('aruco_compressed_detector', anonymous=True)

        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.parameters = cv2.aruco.DetectorParameters_create()

        # === Publisher for marker data ===
        self.marker_pose_pub = rospy.Publisher('/aruco/marker_pose', PoseStamped, queue_size=10)
        
        # === Load camera calibration ===
        calib_file = rospy.get_param('~calibration_file')
        if not os.path.exists(calib_file):
            rospy.logerr(f"[ERROR] Calibration file not found: {calib_file}")
            exit(1)
        rospy.loginfo(f"[INFO] Loading camera calibration from: {calib_file}")
        fs = cv2.FileStorage(calib_file, cv2.FILE_STORAGE_READ)
        self.camera_matrix = fs.getNode("camera_matrix").mat()
        self.dist_coeffs = fs.getNode("distortion_coefficients").mat()
        fs.release()
        
        if self.camera_matrix is None or self.dist_coeffs is None:
            rospy.logerr("[ERROR] Failed to load calibration.")
            exit(1)
        else:
            rospy.loginfo("[INFO] Camera calibration loaded.")

        # === Set marker length in real-world units (e.g., cm or meters) ===
        self.marker_size = rospy.get_param('marker_size', 0.10)  # detault to 10 cm marker size

        rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.image_callback)
        rospy.loginfo("ArUco detector node started. Waiting for images...")
        rospy.spin()
        

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            rospy.logerr(f"[ERROR] Failed to decode image: {e}")
            return

        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.dictionary, parameters=self.parameters)

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

            # Estimate pose for each detected marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )

            for i in range(len(ids)):
                tvec = tvecs[i][0]  # Only need translation vector (position)
                marker_id = ids[i][0]
                
                rospy.loginfo(f"[POSE] ID {marker_id} | Position: x={tvec[0]:.3f}, y={tvec[1]:.3f}, z={tvec[2]:.3f}")

                # Publish marker pose with ID encoded in frame_id
                marker_pose = PoseStamped()
                marker_pose.header.stamp = rospy.Time.now()
                marker_pose.header.frame_id = f"aruco_marker_{marker_id}"
                
                # Set position
                marker_pose.pose.position.x = float(tvec[0])
                marker_pose.pose.position.y = float(tvec[1])
                marker_pose.pose.position.z = float(tvec[2]) + 0.06  # Adjust Z position given camera position
                
                # Set orientation to identity (no rotation info needed)
                marker_pose.pose.orientation.x = 0.0
                marker_pose.pose.orientation.y = 0.0
                marker_pose.pose.orientation.z = 0.0
                marker_pose.pose.orientation.w = 1.0
                
                self.marker_pose_pub.publish(marker_pose)

                # Optional: draw coordinate axes on the image (not shown unless you use imshow)
                # cv2.aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i][0], tvec, self.marker_size * 0.5)

        else:
            rospy.loginfo("[INFO] No markers detected.")

def calibrate_camera_angle():
    """
    Centers both camera servo motors (pan and tilt) to their middle position using PCA9685.
    """
    try:
        # Import the PCA9685 driver
        try:
            from alphabot_driver.PCA9685 import PCA9685
        except ImportError:
            from PCA9685 import PCA9685
        
        # Initialize PWM controller
        pwm = PCA9685(0x40)
        pwm.setPWMFreq(50)
        
        # Define servo angle to pulse conversion function
        def set_servo_angle(pwm_controller, channel, angle):
            """
            Convert angle (0-180 degrees) to pulse width and set servo position.
            Includes compensation for physical servo misalignment.
            
            Args:
                pwm_controller: PCA9685 instance
                channel: Servo channel (0 for pan, 1 for tilt)
                angle: Angle in degrees (0-180, where 90 is middle)
            """
            # Compensation offsets for servo misalignment
            # Camera is facing 45° down and 45° to the right when servos are at 90°
            if channel == 0:  # Pan servo (horizontal)
                # Compensate for 45° right offset - subtract 45° to center
                # Anti-clockwise rotation is positive
                compensated_angle = angle + 27
            elif channel == 1:  # Tilt servo (vertical)
                # Compensate for 45° down offset - add 45° to center (assuming down is positive)
                # Down is positive
                compensated_angle = angle - 45
            else:
                rospy.logerr(f"[ERROR] Invalid servo channel: {channel}")
                return
            
            # Clamp compensated angle to valid servo range
            compensated_angle = max(0, min(180, compensated_angle))
            
            # Convert angle to pulse width (500-2500 microseconds)
            # 0 degrees = 500us, 90 degrees = 1500us, 180 degrees = 2500us
            pulse_width = int(500 + (compensated_angle / 180.0) * 2000)
            
            # Clamp to valid range
            pulse_width = max(500, min(2500, pulse_width))
            
            # Set servo position
            pwm_controller.setServoPulse(channel, pulse_width)
            
            rospy.loginfo(f"[SERVO] Channel {channel} set to {angle}° (compensated: {compensated_angle}°, pulse: {pulse_width}μs)")
        
        # Set both servos to middle position (90 degrees)
        rospy.loginfo("[CALIBRATION] Centering camera servos...")
        
        # Channel 0: Pan servo (horizontal) - center at 90 degrees (anti-clockwise rotation is positive)
        set_servo_angle(pwm, 0, 90)
        
        # Channel 1: Tilt servo (vertical) - center at 90 degrees (down is positive)  
        set_servo_angle(pwm, 1, 90)
        
        rospy.loginfo("[CALIBRATION] Camera servos centered successfully!")
        
        # Give servos time to move to position
        rospy.sleep(1.0)
        
    except Exception as e:
        rospy.logerr(f"[ERROR] Failed to calibrate camera angle: {e}")
        raise


if __name__ == "__main__":
    calibrate_camera_angle()
    rospy.loginfo("[INFO] Camera angle calibrated. Starting ArUco detector...")
    rospy.sleep(1.0)  # Wait for servos to stabilize
    try:
        ArucoCompressedDetector()
    except rospy.ROSInterruptException:
        pass
