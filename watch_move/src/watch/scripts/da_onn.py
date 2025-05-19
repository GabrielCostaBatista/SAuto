#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import sys

class ArucoCompressedDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.parameters = cv2.aruco.DetectorParameters_create()

        rospy.init_node('aruco_compressed_detector', anonymous=True)
        rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.image_callback)
        print("[INFO] ArUco detector initialized. Waiting for compressed image stream...")
        rospy.spin()

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            rospy.logerr(f"[ERROR] Failed to decode image: {e}")
            return

        corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.dictionary, parameters=self.parameters)

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            rospy.loginfo(f"Detected ArUco IDs: {ids.flatten().tolist()}")

        # This part is removed for headless systems:
        # cv2.imshow("ArUco Compressed Live View", cv_image)
        # cv2.waitKey(1)

if __name__ == "__main__":
    try:
        ArucoCompressedDetector()
    except rospy.ROSInterruptException:
        pass
    # Also remove GUI cleanup since no display is used
    # cv2.destroyAllWindows()
