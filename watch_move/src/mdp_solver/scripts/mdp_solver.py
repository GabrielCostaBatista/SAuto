#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from MDP_simple_maze_solver import MDP
import sys
import termios
import tty
import os
import signal
import threading
import math
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

RESOLUTION = 0.10   # metres per cell

def move_robot_to_position(client, x, y, theta):
    pose = MoveBaseGoal()
    pose.target_pose.header.frame_id = "map"
    pose.target_pose.header.stamp = rospy.Time.now()
    pose.target_pose.pose.position.x = x * RESOLUTION
    pose.target_pose.pose.position.y = y  * RESOLUTION

    # Convert yaw → quaternion
    q = quaternion_from_euler(0.0, 0.0, theta)
    pose.target_pose.pose.orientation.x = q[0]
    pose.target_pose.pose.orientation.y = q[1]
    pose.target_pose.pose.orientation.z = q[2]
    pose.target_pose.pose.orientation.w = q[3]

    client.send_goal(pose)
    # client.wait_for_result()
    rospy.loginfo(f"Sent goal: x={x:.2f}, y={y:.2f}, θ={theta:.2f} rad")
    return client.get_state()

# if __name__ == '__main__':
rospy.init_node('move_robot')
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
# client.wait_for_server()

for i in range(len(path)):
    x, y = path[i]
    rospy.loginfo(f"Teste {i}: x={x}, y={y}")
    # compute heading toward next waypoint (or zero at the end)
    if i < len(path) - 1:
        dx, dy = path[i+1]
        theta = math.atan2((dx - x), (dy - y))
    else:
        theta = 0.0

    move_robot_to_position(client, x, y, theta)

    # wait for the robot to reach (tune this duration as needed)
    rospy.sleep(5.0)

    rospy.loginfo("Waiting...")

rospy.loginfo("Goal of maze reached. Exiting.") 
