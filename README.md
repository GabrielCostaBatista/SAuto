# Autonomous Systems Final Project: Maze Solving with AlphaBot2

This guide covers the entirety of the folders for the project.

The folder watch_move/src has all the codes developed. Following is the guide for each.

## Table of Contents

- [1. mdp_solver: Algorithm development and AlphaBot2 integration](#1-mdp_solver-algorithm-development-alphabot2-integration)
  - [1.1. Launch Files](#11-launch-files)
  - [1.2. Scripts: Algorithms](#12-scripts-algorithms)
  - [1.3. Scripts: AlphaBot2 integration](#13-scripts-alphabot2-integration)
- [2. move: First test of AlphaBot2 movement](#2-move-first-test-of-alphabot2-movement)
- [3. watch: Camera and ArUco Detection](#3-watch-camera-and-aruco-detection)
  - [3.1. ArUco Setups](#31-aruco-setups)
  - [3.2. Calibration](#32-calibration)
  - [3.3. ArUco Localization](#33-aruco-localization)
  - [3.4. Camera Info](#34-camera-info)
  - [3.5. Trials](#35-trials)

---

# 1. mdp_solver: Algorithm development and AlphaBot2 integration

This folder contains the files for the implementation of all algorithms. All important files will be mentioned with their purpose.

## 1.1. Launch files

- mdp_solver.launch: File to launch the file that integrates the AlphaBot2 and the MDP algorithm so that it may run on the AlphaBot2.

- qmdp_solver.launch: File to launch the file that integrates the AlphaBot2 and the QMDP algorithm so that it may run on the AlphaBot2.


## 1.2. Scripts: Algorithms

- MDP_simple_maze_solver.py: Algorithm to solve a maze using a MDP.

- POMDP_simple_solver.py: Algorithm to solve a maze using a QMDP.


## 1.3. Scripts: AlphaBot2 integration

- qmdp_solver.py: Integration of an algorithm with the AlphaBot2 interface and integration of robot's movement with the camera and ArUco detection.


# 2. move: First test of AlphaBot2 movement

Includes launch files as well. Has a first test of controlling the robot's movement.

- move.py and move_v2.py: First test of AlphaBot2's movement via keyboard control.


# 3. watch: Camera and ArUco Detection 

Includes launch files as well. Has the control of the camera and the process of the ArUco markers.

## 3.1. ArUco Setups

- aruco_listener.py: Detect the ArUco markers

- aruco_pose.py: Determine the position from the ArUco markers.

## 3.2. Calibration

- calibration_chess_board.py: Calibration of the Camera.

## 3.3. ArUco Localization

- localization.py: Localize the robot from the detection of ArUco markers.

## 3.4. Camera Info

- camera_info: Folder with camera calibration.

## 3.5. Trials

- trials: Tests of the calibration and detection of ArUco markers offline without the AlphaBot2



## Authors

This projected was made by:

André Feliciano

Gabriel Batista

Maria Maló

José Mariano