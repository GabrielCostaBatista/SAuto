#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped


class ArucoListener:
    def __init__(self):
        rospy.init_node('aruco_listener', anonymous=True)
        
        # Subscribe to the unified marker pose topic
        rospy.Subscriber('/aruco/marker_pose', PoseStamped, self.marker_callback)
        
        rospy.loginfo("ArUco listener node started. Waiting for marker detections...")
        rospy.spin()
    
    def marker_callback(self, msg):
        """
        Callback function to process ArUco marker pose data.
        
        Args:
            msg (PoseStamped): Message containing marker pose with ID encoded in frame_id
        """
        try:
            # Extract marker ID from frame_id (format: "aruco_marker_{id}")
            frame_id_parts = msg.header.frame_id.split('_')
            if len(frame_id_parts) >= 3 and frame_id_parts[0] == "aruco" and frame_id_parts[1] == "marker":
                marker_id = int(frame_id_parts[2])
            else:
                rospy.logwarn(f"Invalid frame_id format: {msg.header.frame_id}")
                return
            
            # Extract position data
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            
            # Extract orientation (though it's set to identity in our case)
            qx = msg.pose.orientation.x
            qy = msg.pose.orientation.y
            qz = msg.pose.orientation.z
            qw = msg.pose.orientation.w
            
            # Here you can add your custom logic based on the marker data
            # self.process_marker_data(marker_id, x, y, z)
            
        except ValueError as e:
            rospy.logerr(f"Error parsing marker ID from frame_id '{msg.header.frame_id}': {e}")
        except Exception as e:
            rospy.logerr(f"Error processing marker data: {e}")
    
    def process_marker_data(self, marker_id, x, y, z):
        """
        Process the marker data for navigation or other purposes.
        
        Args:
            marker_id (int): The ID of the detected marker
            x (float): X position relative to camera
            y (float): Y position relative to camera  
            z (float): Z position relative to camera (distance)
        """
        # Example: Different behaviors based on marker ID
        if marker_id == 0:
            rospy.loginfo(f"Marker 0 detected - Wall at position ({x:.3f}, {y:.3f}, {z:.3f})!")
        else:
            rospy.loginfo(f"Localization marker {marker_id} detected - General processing")


if __name__ == "__main__":
    try:
        ArucoListener()
    except rospy.ROSInterruptException:
        rospy.loginfo("ArUco listener node stopped.")
