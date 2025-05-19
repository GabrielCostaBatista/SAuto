#!/usr/bin/env python3

import cv2
import numpy as np
import os
import argparse
import yaml

def find_and_draw_corners(image, board_size, debug=False, adaptive_thresh=False):
    """Find chessboard corners in an image."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Try different detection methods if enabled
    if adaptive_thresh:
        # Apply adaptive thresholding and try to enhance pattern visibility
        gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                    cv2.THRESH_BINARY, 11, 2)
    
    # Try with different flags to improve detection
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    if debug:
        print(f"  Trying to find {board_size[0]}x{board_size[1]} checkerboard...")
        debug_img = image.copy()
        cv2.putText(debug_img, f"Looking for {board_size[0]}x{board_size[1]} board", 
                   (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imwrite(f"debug_before_detection.jpg", debug_img)
        cv2.imwrite(f"debug_gray.jpg", gray)
    
    # Try to find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, board_size, flags)
    
    if ret:
        # Refine corner detection
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
        # Draw corners on the image
        cv2.drawChessboardCorners(image, board_size, corners2, ret)
        if debug:
            print(f"  Success! Found {len(corners)} corners")
    elif debug:
        print(f"  Failed to find corners with standard method")
        # Try a different board size as a diagnostic check
        alt_size = (board_size[1], board_size[0])  # Try swapped dimensions
        ret_alt, corners_alt = cv2.findChessboardCorners(gray, alt_size, flags)
        if ret_alt:
            print(f"  However, found a {alt_size[0]}x{alt_size[1]} board instead!")
            print(f"  This suggests your --board_width and --board_height parameters may be swapped")
    
    return ret, corners

def calibrate_camera(image_dir, board_size, square_size, output_file, debug=False, adaptive_thresh=False):
    """Calibrate camera using images of a chessboard."""
    # Prepare object points (0,0,0), (1,0,0), (2,0,0) ...
    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
    objp = objp * square_size  # Scale by square size
    
    # Arrays to store object points and image points
    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane
    
    # Get list of image files
    image_files = [os.path.join(image_dir, f) for f in os.listdir(image_dir) 
                  if (f.endswith('.jpg') or f.endswith('.png')) and not f.startswith('annotated_')]
    
    print(f"Found {len(image_files)} images for calibration")
    if len(image_files) < 10:
        print("Warning: Fewer than 10 images might result in poor calibration")
    
    img_shape = None
    successful_images = 0
    
    # Try to detect a chessboard in the first image with extra debug info
    if debug and len(image_files) > 0:
        print(f"Analyzing first image with extra debug info: {image_files[0]}")
        first_img = cv2.imread(image_files[0])
        if first_img is not None:
            cv2.imwrite(os.path.join(image_dir, "debug_first_image.jpg"), first_img)
            # Try multiple chessboard sizes as a diagnostic
            for test_width, test_height in [(board_size[0], board_size[1]), 
                                           (board_size[1], board_size[0]),
                                           (board_size[0]-1, board_size[1]-1),
                                           (7, 6), (8, 5), (9, 6)]:
                print(f"Testing for a {test_width}x{test_height} chessboard...")
                test_img = first_img.copy()
                ret, _ = find_and_draw_corners(test_img, (test_width, test_height), debug=True)
                if ret:
                    print(f"FOUND a {test_width}x{test_height} chessboard!")
                    cv2.imwrite(os.path.join(image_dir, f"debug_found_{test_width}x{test_height}.jpg"), test_img)
                else:
                    print(f"Did NOT find a {test_width}x{test_height} chessboard")
    
    # Process each image
    for i, fname in enumerate(image_files):
        if debug:
            print(f"\nProcessing image {i+1}/{len(image_files)}: {fname}")
        
        img = cv2.imread(fname)
        if img is None:
            print(f"Could not read image {fname}")
            continue
            
        if img_shape is None:
            img_shape = img.shape[:2][::-1]  # width, height
            if debug:
                print(f"Image dimensions: {img_shape[0]}x{img_shape[1]}")
        
        # Find chess board corners
        ret, corners = find_and_draw_corners(img, board_size, debug=debug, adaptive_thresh=adaptive_thresh)
        
        if ret:
            objpoints.append(objp)
            imgpoints.append(corners)
            successful_images += 1
            print(f"Successfully processed image {fname} ({successful_images} total)")
            
            # Save annotated image for verification
            annotated_path = os.path.join(image_dir, f"annotated_{os.path.basename(fname)}")
            cv2.imwrite(annotated_path, img)
        else:
            print(f"No {board_size[0]}x{board_size[1]} chessboard found in {fname}")
    
    print(f"Found chessboard corners in {successful_images} out of {len(image_files)} images")
    
    if successful_images < 3:
        print("Error: Need at least 3 good images for calibration")
        return False
    
    # Perform calibration
    print("Performing camera calibration...")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, img_shape, None, None)
    
    # Calculate calibration error
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error
    
    print(f"Total calibration error: {mean_error/len(objpoints)}")
    
    # Save calibration results to YAML file
    calibration_data = {
        'image_width': img_shape[0],
        'image_height': img_shape[1],
        'camera_name': 'camera',
        'camera_matrix': {
            'rows': 3,
            'cols': 3,
            'data': mtx.flatten().tolist()
        },
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {
            'rows': 1,
            'cols': 5,
            'data': dist.flatten().tolist()
        },
        'rectification_matrix': {
            'rows': 3,
            'cols': 3,
            'data': np.eye(3).flatten().tolist()
        },
        'projection_matrix': {
            'rows': 3,
            'cols': 4,
            'data': np.hstack((mtx, np.zeros((3, 1)))).flatten().tolist()
        }
    }
    
    with open(output_file, 'w') as f:
        yaml.dump(calibration_data, f)
    
    print(f"Calibration data saved to {output_file}")
    return True

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process checkerboard images for camera calibration')
    parser.add_argument('--dir', type=str, default=os.path.expanduser('~/calibration_images'),
                        help='Directory containing calibration images')
    parser.add_argument('--board_width', type=int, default=9,
                        help='Number of inner corners along width (columns)')
    parser.add_argument('--board_height', type=int, default=7,
                        help='Number of inner corners along height (rows)')
    parser.add_argument('--square_size', type=float, default=0.022,
                        help='Size of each square in meters')
    parser.add_argument('--output', type=str, 
                        default=os.path.expanduser('~/SAuto/watch_move/src/watch/camera_info/camera.yaml'),
                        help='Output calibration file')
    parser.add_argument('--debug', action='store_true',
                        help='Enable debug mode with extra information and diagnostics')
    parser.add_argument('--adaptive', action='store_true',
                        help='Use adaptive thresholding to improve corner detection')
    parser.add_argument('--swap-dims', action='store_true',
                        help='Swap the board width and height dimensions')
    
    args = parser.parse_args()
    
    # Create output directory if it doesn't exist
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    
    # Get board dimensions, swap if requested
    board_width = args.board_width
    board_height = args.board_height
    if args.swap_dims:
        board_width, board_height = board_height, board_width
        print(f"Swapped dimensions: Using {board_width}x{board_height} board")
    
    print("\n=== Camera Calibration Process ===")
    print(f"Looking for a {board_width}x{board_height} chessboard with {board_width*board_height} internal corners")
    print(f"Square size: {args.square_size} meters")
    print(f"Image directory: {args.dir}")
    print(f"Debug mode: {'ON' if args.debug else 'OFF'}")
    print(f"Adaptive thresholding: {'ON' if args.adaptive else 'OFF'}")
    print("============================\n")
    
    # Run calibration
    success = calibrate_camera(
        args.dir, 
        (board_width, board_height), 
        args.square_size,
        args.output,
        debug=args.debug,
        adaptive_thresh=args.adaptive
    )
