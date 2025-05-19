#!/bin/bash
# This script copies the camera calibration file from ~/.ros/camera_info
# to the project's camera_info directory

# First check if the calibration file exists
if [ -f ~/.ros/camera_info/head_camera.yaml ]; then
  # Create the target directory if it doesn't exist
  mkdir -p ~/SAuto/watch_move/src/watch/camera_info
  
  # Copy the file
  cp ~/.ros/camera_info/head_camera.yaml ~/SAuto/watch_move/src/watch/camera_info/camera.yaml
  
  echo "Calibration file successfully copied to the project directory!"
  echo "Location: ~/SAuto/watch_move/src/watch/camera_info/camera.yaml"
else
  echo "Error: Calibration file not found at ~/.ros/camera_info/head_camera.yaml"
  echo "Please complete the camera calibration process first."
fi
