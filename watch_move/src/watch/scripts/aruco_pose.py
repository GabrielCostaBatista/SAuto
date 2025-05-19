#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import os
import yaml


class ArucoCompressedDetector:
    def __init__(self):
        rospy.init_node('aruco_compressed_detector', anonymous=True)

        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.parameters = cv2.aruco.DetectorParameters_create()

        # === Load camera calibration ===
        calib_file = rospy.get_param('~calibration_file')
        if not os.path.exists(calib_file):
            rospy.logerr(f"[ERROR] Calibration file not found: {calib_file}")
            exit(1)
        rospy.loginfo(f"[INFO] Loading camera calibration from: {calib_file}")
        with open(calib_file, 'r') as f:
            calib_data = yaml.safe_load(f)
        self.camera_matrix = np.array(calib_data['camera_matrix']['data']).reshape((3, 3))
        self.dist_coeffs = np.array(calib_data['distortion_coefficients']['data']).reshape((1, 5))

        if self.camera_matrix is None or self.dist_coeffs is None:
            rospy.logerr("[ERROR] Failed to load calibration.")
            exit(1)
        else:
            rospy.loginfo("[INFO] Camera calibration loaded.")

        # === Set marker length in real-world units (e.g., cm or meters) ===
        self.marker_length = 0.10  # 10 cm marker length

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
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )

            for i in range(len(ids)):
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]
                rospy.loginfo(f"[POSE] ID {ids[i][0]} | tvec: {tvec} | rvec: {rvec}")

                # Optional: draw coordinate axes on the image (not shown unless you use imshow)
                # cv2.aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_length * 0.5)

        else:
            rospy.loginfo("[INFO] No markers detected.")

if __name__ == "__main__":
    try:
        ArucoCompressedDetector()
    except rospy.ROSInterruptException:
        pass
