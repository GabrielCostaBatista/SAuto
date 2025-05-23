#!/usr/bin/env python3

import rospy
import cv2
import os
import time
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class CaptureCalibrationImages:
    def __init__(self):
        rospy.init_node('capture_calibration_images', anonymous=True)
        
        # Parameters
        self.save_dir = os.path.expanduser('~/calibration_images')
        self.num_images = rospy.get_param('~num_images', 20)
        self.delay_between_captures = rospy.get_param('~delay', 2)  # seconds
        self.use_compressed = rospy.get_param('~use_compressed', True)
        
        # Create directory if it doesn't exist
        os.makedirs(self.save_dir, exist_ok=True)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize counters
        self.image_count = 0
        self.last_capture_time = time.time()
        
        # Subscribe to image topic
        if self.use_compressed:
            self.image_sub = rospy.Subscriber('/raspicam_node/image/compressed', 
                                             CompressedImage, 
                                             self.compressed_image_callback)
            rospy.loginfo("Subscribed to compressed image topic")
        else:
            self.image_sub = rospy.Subscriber('/raspicam_node/image', 
                                             Image, 
                                             self.image_callback)
            rospy.loginfo("Subscribed to raw image topic")
        
        rospy.loginfo(f"Will capture {self.num_images} images with {self.delay_between_captures}s delay")
        rospy.loginfo(f"Saving images to {self.save_dir}")
        rospy.loginfo("Press Ctrl+C to stop capturing early")

    def image_callback(self, data):
        # Convert ROS Image to OpenCV format
        try:
            current_time = time.time()
            if current_time - self.last_capture_time >= self.delay_between_captures:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                # Add a visual indicator that we're capturing this frame
                # Code that was previously erased and I don't know if it was important so I leave it here commented
                # timestamp = time.strftime("%H:%M:%S")
                # height, width = cv_image.shape[:2]
                # cv2.putText(cv_image, f"Calibration image {self.image_count+1}/{self.num_images}", 
                #            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                # cv2.putText(cv_image, f"{timestamp}", 
                #            (width - 150, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                self.save_image(cv_image)
                self.last_capture_time = current_time
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")

    def compressed_image_callback(self, data):
        # Convert compressed image to OpenCV format
        try:
            current_time = time.time()
            if current_time - self.last_capture_time >= self.delay_between_captures:
                # Use frombuffer instead of fromstring (fixes deprecation warning)
                np_arr = np.frombuffer(data.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                # Add a visual indicator that we're capturing this frame
                timestamp = time.strftime("%H:%M:%S")
                height, width = cv_image.shape[:2]
                cv2.putText(cv_image, f"Calibration image {self.image_count+1}/{self.num_images}", 
                           (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(cv_image, f"{timestamp}", 
                           (width - 150, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                self.save_image(cv_image)
                self.last_capture_time = current_time
        except Exception as e:
            rospy.logerr(f"Error processing compressed image: {e}")

    def save_image(self, cv_image):
        filename = os.path.join(self.save_dir, f'calibration_img_{self.image_count:03d}.jpg')
        cv2.imwrite(filename, cv_image)
        rospy.loginfo(f"Saved image {self.image_count+1}/{self.num_images}: {filename}")
        
        self.image_count += 1
        if self.image_count >= self.num_images:
            rospy.loginfo("Finished capturing all images. Shutting down...")
            rospy.signal_shutdown("Finished capturing images")

if __name__ == '__main__':
    try:
        import numpy as np  # Import here to avoid issues
        node = CaptureCalibrationImages()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo(f"Captured {node.image_count} images for calibration")
