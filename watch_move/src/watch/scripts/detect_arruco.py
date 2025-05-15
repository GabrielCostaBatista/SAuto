#!/usr/bin/env python3

import rospy
import time
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseWithCovariance

def marker_callback(msg):
    # For marker_publisher, we get a MarkerArray with multiple markers
    if len(msg.markers) == 0:
        if time.time() - marker_callback.last_empty_log > 5.0:  # Log every 5 seconds
            rospy.loginfo("No ArUco markers detected.")
            marker_callback.last_empty_log = time.time()
        return
        
    marker_callback.last_empty_log = time.time()  # Reset timer when markers are detected
    
    for marker in msg.markers:
        marker_id = marker.id
        # Position of the marker relative to the camera
        x = marker.pose.pose.position.x
        y = marker.pose.pose.position.y
        z = marker.pose.pose.position.z
        
        # Extract orientation
        qx = marker.pose.pose.orientation.x
        qy = marker.pose.pose.orientation.y
        qz = marker.pose.pose.orientation.z
        qw = marker.pose.pose.orientation.w
        
        # Print marker information
        rospy.loginfo(f"Detected marker {marker_id} at position: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        rospy.loginfo(f"Orientation quaternion: qx={qx:.3f}, qy={qy:.3f}, qz={qz:.3f}, qw={qw:.3f}")
        # Use x, y, z for navigation or localization

# Initialize node and setup subscriber
rospy.init_node('aruco_listener')
rospy.loginfo("ArUco listener node initialized. Waiting for marker detections...")

# Add a timer to prevent excessive logging
marker_callback.last_empty_log = time.time()

# Subscribe to the marker topic
rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, marker_callback)

# Add diagnostic information
topics = rospy.get_published_topics()
rospy.loginfo("Available topics:")
for topic, topic_type in topics:
    rospy.loginfo(f"  {topic} ({topic_type})")

rospy.loginfo("Starting spin...")
rospy.spin()