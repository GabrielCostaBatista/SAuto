"""
    def compute_robot_pose(self, observed_marker_id):
        
        Compute robot's global pose using marker observation

        Theory:
        - We know marker's position and orientation in world coordinates: (x_m, y_m, θ_m)
        - We observe marker's position relative to robot: (x_r, y_r) - no orientation
        - We need to estimate robot's pose in world coordinates
        
        The marker's orientation tells us which direction the marker is facing in the world.
        The robot's observation tells us where the marker appears relative to the robot.
        We can use this to estimate both the robot's position and orientation.
        
      
        # Check if we have global position for this marker
        if observed_marker_id in self.global_markers:
            global_marker_pos = self.global_markers[observed_marker_id]

            # Get robot's observation of this marker
            if observed_marker_id not in self.robot_observations:
                rospy.logwarn(f"No robot observation found for marker {observed_marker_id}")
                return
            
            robot_observation = self.robot_observations[observed_marker_id]
            
            # Extract marker data
            marker_world_x, marker_world_y, marker_world_orientation = global_marker_pos
            robot_x, robot_y, robot_z = robot_observation

            # Compute robot orientation in world frame
            robot_world_yaw = self.discrete_orientation_to_yaw(marker_world_orientation) - math.atan2(robot_x, robot_z) # Adjusting for observation angle (minus because x is positive to the right)
            
         # Compute robot position in world frame
         # Transform the robot's observation vector to world coordinates using estimated robot orientation
            cos_robot_yaw = math.cos(robot_world_yaw)
            sin_robot_yaw = math.sin(robot_world_yaw)

             # Robot's position in world coordinates
            world_robot_x = marker_world_x - robot_z * cos_robot_yaw + robot_x * sin_robot_yaw
            world_robot_y = marker_world_y - robot_z * sin_robot_yaw - robot_x * cos_robot_yaw
            world_robot_z = 0.0  # Assuming robot moves on ground plane
            
            # Convert robot yaw to quaternion
            quat = self.yaw_to_quaternion(robot_world_yaw)
            
            # Update robot pose
            self.robot_pose.header.stamp = rospy.Time.now()
            self.robot_pose.pose.position.x = world_robot_x
            self.robot_pose.pose.position.y = world_robot_y
            self.robot_pose.pose.position.z = world_robot_z
          
            # Set orientation quaternion
            self.robot_pose.pose.orientation.x = quat['x']
            self.robot_pose.pose.orientation.y = quat['y']
            self.robot_pose.pose.orientation.z = quat['z']
            self.robot_pose.pose.orientation.w = quat['w']
            
            rospy.loginfo(f"Robot localized using marker {observed_marker_id}: "
                     f"position=({world_robot_x:.3f}, {world_robot_y:.3f}, {world_robot_z:.3f}), "
                        f"yaw={math.degrees(robot_world_yaw):.1f}°")

        elif observed_marker_id in self.protected_marker_positions:
            rospy.loginfo(f"Watching protected marker with ID {observed_marker_id}")
        else:
            rospy.logwarn(f"Marker {observed_marker_id} not found in global marker database or protected markers.")
            return
    """