#!/usr/bin/env python3

import rospy
from fiducial_msgs.msg import FiducialTransformArray

def marker_callback(msg):
    for transform in msg.transforms:
        marker_id = transform.fiducial_id
        # Position of the marker relative to the camera
        x = transform.transform.translation.x
        y = transform.transform.translation.y
        z = transform.transform.translation.z
        # Use x, y, z for navigation or localization

rospy.init_node('aruco_listener')
rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, marker_callback)
rospy.spin()