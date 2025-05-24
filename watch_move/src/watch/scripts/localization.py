#!/usr/bin/env python3
import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseStamped, PoseArray

# Number of protected markers (can be overridden via ROS parameter in launch file)
NUM_PROTECTED_MARKERS = rospy.get_param('~num_protected_markers', 2)

# Protected markers id list (can be overridden via ROS parameter in launch file)
PROTECTED_MARKERS = rospy.get_param('~protected_markers', [0, 1])

class RobotLocalizer:
    def __init__(self):
        rospy.init_node('robot_global_pose_publisher', anonymous=True)
        
        # Publishers
        self.robot_pose_pub = rospy.Publisher('global_locations/robot_pose', PoseStamped, queue_size=10)
        
        # Data storage
        self.global_markers = {}  # marker_id -> (x, y) in world coordinates
        self.robot_observations = {}  # marker_id -> (x, y) in robot frame
        self.protected_marker_positions = {}  # marker_id -> (x, y) for protected markers
        
        # Initialize protected marker positions (you can modify these as needed)
        # For now, setting some default positions - you should update these with actual positions
        for marker_id in PROTECTED_MARKERS:
            self.protected_marker_positions[marker_id] = (0.0, 0.0)  # Default to origin
            rospy.loginfo(f"Protected marker {marker_id} initialized at (0.0, 0.0) - update as needed")
        
        # Robot pose estimate
        self.robot_pose = PoseStamped()
        self.robot_pose.header.frame_id = "map"
        
        # Subscribers
        rospy.Subscriber('global_locations/marker_pose', PoseArray, self.global_markers_callback)
        rospy.Subscriber('aruco/marker_pose', PoseStamped, self.aruco_marker_callback)
        rospy.loginfo("Robot localizer initialized. Waiting for marker data...")


    def global_markers_callback(self, msg):
        """
        Callback for global marker positions (world coordinates)
        Assumes marker IDs are encoded somehow - for now using index as ID
        """
        self.global_markers.clear()
        for i, pose in enumerate(msg.poses):
            marker_id = i + NUM_PROTECTED_MARKERS
            self.global_markers[marker_id] = (pose.position.x, pose.position.y)
            rospy.loginfo(f"Updated global marker positions: {len(self.global_markers)} markers")
    

    def set_protected_marker_position(self, marker_id, x, y):
        """
        Set the global position of a protected marker
        """
        if marker_id in PROTECTED_MARKERS:
            self.protected_marker_positions[marker_id] = (x, y)
            rospy.loginfo(f"Updated protected marker {marker_id} position to ({x}, {y})")
        else:
            rospy.logwarn(f"Marker {marker_id} is not in the protected markers list")


    def aruco_marker_callback(self, msg):
        """
        Callback for ArUco marker detections (robot camera frame)
        Marker ID is encoded in frame_id as "aruco_marker_{id}"
        """
        try:
            # Extract marker ID from frame_id
            frame_parts = msg.header.frame_id.split('_')
            if len(frame_parts) == 3 and frame_parts[0] == "aruco" and frame_parts[1] == "marker":
                marker_id = int(frame_parts[2])
            else:
                rospy.logwarn(f"Invalid frame_id format: {msg.header.frame_id}")
                return

            # Store robot's observation of this marker
            self.robot_observations[marker_id] = (msg.pose.position.x, msg.pose.position.y)

            # Compute robot pose based on this marker observation
            self.compute_robot_pose(marker_id)

        except (ValueError, IndexError) as e:
            rospy.logerr(f"Error parsing marker ID: {e}")
    

    def compute_robot_pose(self, observed_marker_id):
        """
        Compute robot's global pose using marker observation
        
        Theory:
        - We know marker's position in world coordinates: P_world
        - We observe marker's position relative to robot: P_robot  
        - Robot's position in world = P_world - P_robot (simplified 2D case)
        """
        # Check if we have global position for this marker
        if observed_marker_id in self.global_markers:
            global_marker_pos = self.global_markers[observed_marker_id]

            # Get robot's observation of this marker
            if observed_marker_id not in self.robot_observations:
                rospy.logwarn(f"No robot observation found for marker {observed_marker_id}")
                return
            
            robot_observation = self.robot_observations[observed_marker_id]
            
            # Convert robot observation to world coordinates
            # Simple approach: assume robot and world frames are aligned (no rotation)
            # For a more complete solution, you'd need to handle rotation transformations
            
            # Robot position = Global marker position - Robot's observation of marker
            robot_x = global_marker_pos[0] - robot_observation[0]
            robot_y = global_marker_pos[1] - robot_observation[1]
            robot_z = 0.0  # Assuming robot moves on ground plane
            
            # Update robot pose
            self.robot_pose.header.stamp = rospy.Time.now()
            self.robot_pose.pose.position.x = robot_x
            self.robot_pose.pose.position.y = robot_y
            self.robot_pose.pose.position.z = robot_z
            
            # Set orientation to identity (no rotation estimation yet)
            self.robot_pose.pose.orientation.x = 0.0
            self.robot_pose.pose.orientation.y = 0.0
            self.robot_pose.pose.orientation.z = 0.0
            self.robot_pose.pose.orientation.w = 1.0
            
            rospy.loginfo(f"Robot localized using marker {observed_marker_id}: "
                        f"position=({robot_x:.3f}, {robot_y:.3f}, {robot_z:.3f})")

        elif observed_marker_id in self.protected_marker_positions:
            rospy.loginfo(f"Watching protected marker with ID {observed_marker_id}")
        else:
            rospy.logwarn(f"Marker {observed_marker_id} not found in global marker database or protected markers.")
            return
    
    def run(self):
        """Main loop to publish robot pose"""
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            # Publish current robot pose estimate
            self.robot_pose_pub.publish(self.robot_pose)
            rate.sleep()


if __name__ == '__main__':
    try:
        localizer = RobotLocalizer()
        localizer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Robot global pose publisher node terminated.")