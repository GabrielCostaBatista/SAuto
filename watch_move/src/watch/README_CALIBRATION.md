# Camera Calibration Instructions

This document provides instructions for calibrating the Raspberry Pi camera used with ArUco markers.

## Prerequisites

1. Install the camera_calibration package:
   ```
   sudo apt-get install ros-noetic-camera-calibration
   ```

2. Print a checkerboard pattern:
   - 9x7 checkerboard (9 squares wide, 7 squares tall)
   - Square size: 2.2cm (0.022 meters)
   - Print on rigid material if possible (cardboard or foam board)
   - Ensure the pattern is completely flat

## Running the Calibration

1. Launch the calibration process:
   ```
   roslaunch watch camera_calibration.launch
   ```

2. A window will open showing the camera feed with detection overlays.

3. Move the checkerboard in front of the camera:
   - Hold it at different angles
   - Cover the entire field of view (corners, edges, center)
   - Keep it at different distances
   - Rotate it in all directions

4. The calibration interface will show progress bars for:
   - X (left-right movement)
   - Y (up-down movement)
   - Size (distance from camera)
   - Skew (rotation around Z axis)

5. When all bars turn green, click the "CALIBRATE" button.

6. After calibration completes, click "SAVE" to store the calibration file.
   - The calibration will be saved to the `~/.ros/camera_info/` directory

7. Copy the calibration file to your project:
   ```
   mkdir -p ~/SAuto/watch_move/src/watch/camera_info
   cp ~/.ros/camera_info/head_camera.yaml ~/SAuto/watch_move/src/watch/camera_info/camera.yaml
   ```

## Using the Calibration

The calibration will be automatically used by the raspicam_node once saved and copied to the project directory.

To verify the calibration is being used:
1. Check that the `camera.yaml` file exists in `~/SAuto/watch_move/src/watch/camera_info/` directory
2. Look for the "Camera successfully calibrated from default file" message in the logs

## Tips for Good Calibration

1. Use good, even lighting
2. Move slowly to avoid motion blur
3. Cover the entire camera field of view
4. Use a variety of angles and distances
5. Keep the checkerboard completely visible
6. Use at least 20-30 different poses for accurate calibration
