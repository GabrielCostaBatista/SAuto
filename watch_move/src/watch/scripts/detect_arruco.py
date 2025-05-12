#!/usr/bin/env python3

import rospy
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseWithCovariance

def marker_callback(msg):
    # For marker_publisher, we get a MarkerArray with multiple markers
    if len(msg.markers) == 0:
        return
        
    for marker in msg.markers:
        marker_id = marker.id
        # Position of the marker relative to the camera
        x = marker.pose.pose.position.x
        y = marker.pose.pose.position.y
        z = marker.pose.pose.position.z
        
        # Print marker information
        rospy.loginfo(f"Detected marker {marker_id} at position: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        # Use x, y, z for navigation or localization

rospy.init_node('aruco_listener')
rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, marker_callback)
rospy.spin()