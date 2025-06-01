#!/usr/bin/env python3
import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseStamped, PoseArray, Polygon, Point32
from std_msgs.msg import Header
from shapely.geometry import box, Polygon as ShapelyPolygon
from shapely.ops import unary_union


def annular_sector(center, r_inner, r_outer, angle_start, angle_end, num_points=100):
    """Create a shapely Polygon representing an annular sector."""
    cx, cy = center
    angles = np.linspace(np.radians(angle_start), np.radians(angle_end), num_points)
    outer = [(cx + r_outer*np.cos(a), cy + r_outer*np.sin(a)) for a in angles]
    inner = [(cx + r_inner*np.cos(a), cy + r_inner*np.sin(a)) for a in angles[::-1]]
    return ShapelyPolygon(outer + inner)

# Number of protected markers (can be overridden via ROS parameter in launch file)
NUM_PROTECTED_MARKERS = rospy.get_param('~num_protected_markers', 2)

# Protected markers id list (can be overridden via ROS parameter in launch file)
PROTECTED_MARKERS = rospy.get_param('~protected_markers', [0, 1])

# Get cell size from ROS parameter (default 0.25 meters)
CELL_SIZE = rospy.get_param('~cell_size', 0.25)  # meters per cell

RADIUS_N_STD_DEV = rospy.get_param('~radius_n_std_dev', 2)

# Get number of frames to average from ROS parameter (default 10)
NUM_FRAMES_TO_AVERAGE = rospy.get_param('~num_frames_to_average', 10)

global distance 

# Initialize distances list for averaging
distances = {}


class RobotLocalizer:
    def __init__(self):
        rospy.init_node('grid_probabilities_publisher', anonymous=True)
        
        # Publishers
        self.grid_prob_pub = rospy.Publisher('global_locations/grid_probabilities', Polygon, queue_size=10)
        
        # Data storage
        self.global_markers = {}  # marker_id -> (x, y, orientation) in world coordinates
        self.protected_marker_positions = {}  # marker_id -> (x, y) for protected markers
        self.robot_observations = {}  # marker_id -> (x, y, z) robot's observation of marker
        self.distance = 0.0
        
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
        Orientation is encoded in z coordinate: 0=right, 1=top, 2=left, 3=bottom
        """
        self.global_markers.clear()
        for i, pose in enumerate(msg.poses):
            marker_id = i + NUM_PROTECTED_MARKERS
            # Extract orientation from z coordinate (encoded as 0=right, 1=top, 2=left, 3=bottom)
            orientation = int(pose.position.z)
            self.global_markers[marker_id] = (pose.position.x, pose.position.y, orientation)
            rospy.loginfo(f"Updated global marker {marker_id}: pos=({pose.position.x:.3f}, {pose.position.y:.3f}), orientation={orientation}")
        rospy.loginfo(f"Updated global marker positions: {len(self.global_markers)} markers")


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
            self.robot_observations[marker_id] = (
                 msg.pose.position.x,
                 msg.pose.position.y, 
                 msg.pose.position.z
            )
            timestamp = msg.header.stamp

            # Compute distance for grid probabilities
            self.distance = math.sqrt((msg.pose.position.x/CELL_SIZE)**2 + (msg.pose.position.z/CELL_SIZE)**2)

            if marker_id in distances and distances[marker_id]:
                last_ts = distances[marker_id][-1][1]
                if timestamp - last_ts > rospy.Duration(1.0):
                    # Reset distances if last timestamp is too old
                    distances[marker_id] = []
                    
            if marker_id not in distances:
                distances[marker_id] = []
            
            distances[marker_id].append([self.distance, timestamp])

            if len(distances[marker_id]) == NUM_FRAMES_TO_AVERAGE:
                # Compute grid probabilities based on this marker observation
                self.grid_probabilities(marker_id)
                distances[marker_id].clear()  # Reset distances after processing

        except (ValueError, IndexError) as e:
            rospy.logerr(f"Error parsing marker ID: {e}")
    
   
    def discrete_orientation_to_yaw(self, discrete_orientation):
        """
        Convert discrete orientation to yaw angle in radians
        0=right (0°), 1=top (90°), 2=left (180°), 3=bottom (270°)
        """
        return discrete_orientation * math.pi / 2
    

    def yaw_to_quaternion(self, yaw):
        """
        Convert yaw angle to quaternion
        """
        return {
            'x': 0.0,
            'y': 0.0,
            'z': math.sin(yaw / 2.0),
            'w': math.cos(yaw / 2.0)
        }
    
    
    def grid_probabilities(self, observed_marker_id, distance_std_dev=0.025):
        """
        Compute robot pose based on observed marker ID and distance
        
        Args:
            observed_marker_id: ID of the observed marker
            distance_std_dev: Distance standard deviation to the marker in cells
        
        Output:
        Publishes a Polygon message where each Point32 contains:
        - x: row coordinate
        - y: column coordinate  
        - z: probability of being in that cell
        """
        # Check if marker is in global markers and is not a protected marker
        if observed_marker_id in self.global_markers:
            global_marker_pos = self.global_markers[observed_marker_id]

            # Get robot's observation of this marker
            if observed_marker_id not in self.robot_observations:
                rospy.logwarn(f"No robot observation found for marker {observed_marker_id}")
                return

            i_min = (global_marker_pos[0] - self.distance)
            i_max = (global_marker_pos[0] + self.distance)
            j_min = (global_marker_pos[1] - self.distance)
            j_max = (global_marker_pos[1] + self.distance)

            if global_marker_pos[2] == 0:
                i_max = global_marker_pos[0]
            elif global_marker_pos[2] == 1:
                j_min = global_marker_pos[1]
            elif global_marker_pos[2] == 2:
                i_min = global_marker_pos[0]
            elif global_marker_pos[2] == 3:
                j_max = global_marker_pos[1] 

            # Extract only the distance values for this marker
            marker_distances = [pair[0] for pair in distances[observed_marker_id]]
            self.distance = np.mean(marker_distances)
            distance_error = np.std(marker_distances) * RADIUS_N_STD_DEV

            sector = annular_sector(center=(global_marker_pos[0], global_marker_pos[1]), r_inner=self.distance - distance_error, r_outer=self.distance + distance_error, angle_start=0, angle_end=180)
            total_sector_area = sector.area if sector.area > 0 else 1  # avoid zero division

            # Create Polygon message to publish probabilities
            probability_map = Polygon()
            
            for i in np.linspace(i_min, i_max, num = i_max - i_min +1):
                for j in np.linspace(j_min, j_max, num = j_max - j_min +1):
                    cell_box = box(i, j, i+1, j+1)
                    intersection = cell_box.intersection(sector)
                    intersection_area = intersection.area if not intersection.is_empty else 0
                    probability = intersection_area / total_sector_area
                    #probability_map[i, j] = probability
                    if probability > 0:  # Only add points with non-zero probability
                        point = Point32()
                        point.x = i
                        point.y = j
                        point.z = probability
                        probability_map.points.append(point)

            self.grid_prob_pub.publish(probability_map)

        else:
            rospy.logwarn(f"Marker {observed_marker_id} not found in global marker database.")

    def run(self):
        """Main loop - grid probabilities are published in real-time via callbacks"""
        rospy.loginfo("Grid probabilities publisher is running. Probabilities published on marker detection.")
        rospy.spin()  # Keep node alive to process callbacks


if __name__ == '__main__':
    try:
        localizer = RobotLocalizer()
        localizer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Robot global pose publisher node terminated.")

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