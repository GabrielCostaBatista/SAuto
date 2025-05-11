#!/usr/bin/env python3

import rospy
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler

from MDP_simple_maze_solver import MDP
import numpy as np



###################
# Maze Definition #
###################

maze = np.array([
    [0,0,0,0,1,0,0,0,0],
    [1,1,1,0,1,0,1,1,1],
    [0,0,0,0,1,0,1,0,0],
    [0,1,0,1,1,0,1,1,0],
    [0,1,0,0,1,0,0,1,0],
    [0,0,1,0,0,0,0,0,0],
], dtype=int)

n_rows, n_cols = maze.shape
start = (0, 0)                # In this case, the robot starts at the top left corner
goal  = (0, n_cols-1)         # In this case, the robot starts at the top right corner

# Create an MDP object
mdp = MDP(maze, start, goal)

path = mdp.get_optimal_path()

###################
# Robot Movement  #
###################



def move_robot_to_position(pub, x, y, theta):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = x
    pose.pose.position.y = y

    # Convert yaw → quaternion
    q = quaternion_from_euler(0.0, 0.0, theta)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]

    pub.publish(pose)
    rospy.loginfo(f"Sent goal: x={x:.2f}, y={y:.2f}, θ={theta:.2f} rad")


# if __name__ == '__main__':
rospy.init_node('move_robot')
pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
rospy.sleep(1.0)

for i in range(len(path)):
    x, y = path[i]

    # compute heading toward next waypoint (or zero at the end)
    if i < len(path) - 1:
        dx, dy = path[i+1]
        theta = float(__import__('math').atan2(dy - y, dx - x))
    else:
        theta = 0.0

    move_robot_to_position(pub, x, y, theta)

    # wait for the robot to reach (tune this duration as needed)
    rospy.sleep(5.0)

    rospy.loginfo("Waiting...")

rospy.loginfo("Goal of maze reached. Exiting.") 
