#!/usr/bin/env python3

import cv2
import numpy as np
import os
import glob
from pathlib import Path

def detect_aruco_markers(image_path):
    """
    Detect ArUco markers in an image.
    Args:
        image_path: Path to the image file
    Returns:
        image: Original image
        corners: Detected marker corners
        ids: Marker IDs
        dict_type: The ArUco dictionary type that detected markers
    """
    # Load the image
    image = cv2.imread(image_path)
    if image is None:
        print(f"Failed to load image: {image_path}")
        return None, None, None, None
    
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Try different ArUco dictionaries
    aruco_dicts = {
        'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
        'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
        'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
        'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
        'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
        'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
        'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
        'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
        'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
        'DICT_7X7_50': cv2.aruco.DICT_7X7_50,
        'DICT_7X7_100': cv2.aruco.DICT_7X7_100,
        'DICT_7X7_250': cv2.aruco.DICT_7X7_250,
        'DICT_ARUCO_ORIGINAL': cv2.aruco.DICT_ARUCO_ORIGINAL,
    }
    
    aruco_params = cv2.aruco.DetectorParameters_create()
    
    # Try each dictionary until we find markers
    for dict_name, dict_type in aruco_dicts.items():
        aruco_dict = cv2.aruco.Dictionary_get(dict_type)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        
        if ids is not None and len(ids) > 0:
            return image, corners, ids, dict_name
    
    # If no markers found with any dictionary
    return image, None, None, None

def main():
    # Path to calibration images
    calibration_dir = Path('/home/ubuntu/SAuto/calibration_images')
    
    # Get all jpg files in the directory
    image_files = sorted(glob.glob(str(calibration_dir / '*.jpg')))
    
    if not image_files:
        print(f"No image files found in {calibration_dir}")
        return
    
    print(f"Found {len(image_files)} images. Checking for ArUco markers with various dictionaries...")
    
    # Process each image
    markers_found = 0
    detected_images = []
    dict_results = {}
    
    for img_path in image_files:
        image, corners, ids, dict_name = detect_aruco_markers(img_path)
        
        if image is None:
            continue
        
        # Get the filename without the full path
        filename = os.path.basename(img_path)
        
        # Check if markers were found
        if ids is not None and len(ids) > 0:
            print(f"{filename}: Found {len(ids)} markers with IDs: {ids.flatten()} using dictionary {dict_name}")
            markers_found += 1
            detected_images.append((img_path, dict_name))
            
            # Keep track of which dictionary worked
            if dict_name not in dict_results:
                dict_results[dict_name] = 0
            dict_results[dict_name] += 1
        else:
            print(f"{filename}: No ArUco markers detected with any dictionary")
    
    print(f"\nSummary: Found ArUco markers in {markers_found} out of {len(image_files)} images")
    
    if dict_results:
        print("\nDictionary detection results:")
        for dict_name, count in dict_results.items():
            print(f"  {dict_name}: {count} images")
        
        # Recommend the most successful dictionary
        best_dict = max(dict_results.items(), key=lambda x: x[1])[0]
        print(f"\nRecommended dictionary for these markers: {best_dict}")
    
    # Save images with detected markers
    if detected_images:
        print("\nSaving images with detected markers to 'detected_markers' directory...")
        
        # Create directory for output images if it doesn't exist
        output_dir = Path('/home/ubuntu/SAuto/detected_markers')
        output_dir.mkdir(exist_ok=True)
        
        # Process each image where markers were found
        for img_path, dict_name in detected_images:
            filename = os.path.basename(img_path)
            output_path = output_dir / f"detected_{filename}"
            
            # Load the image
            image = cv2.imread(img_path)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Detect using the successful dictionary
            aruco_dict = cv2.aruco.Dictionary_get(getattr(cv2.aruco, dict_name))
            aruco_params = cv2.aruco.DetectorParameters_create()
            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
            
            # Draw detected markers
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
            
            # Save the image with detected markers
            cv2.imwrite(str(output_path), image)
            print(f"  Saved: {output_path}")
            
        print(f"All images with detected markers saved to {output_dir}")

if __name__ == '__main__':
    main()
