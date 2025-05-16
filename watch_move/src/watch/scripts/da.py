#!/usr/bin/env python3
"""
da.py

Detects ArUco markers (DICT_4X4_100) in still images using the new OpenCV ArucoDetector API,
annotates them, and prints the detected marker IDs.

Usage:
    python da.py --images img1.jpg img2.png [--output-dir annotated/]

The script writes annotated images into the output directory
(with filenames prefixed by 'annotated_') and prints detected IDs.
"""

import cv2
import argparse
import os
import sys


def detect_and_annotate(image_path: str, detector, output_dir: str):
    # Load image
    image = cv2.imread(image_path)
    if image is None:
        print(f"[ERROR] Could not open image: {image_path}", file=sys.stderr)
        return

    # Detect markers
    corners, ids, rejected = detector.detectMarkers(image)

    # Prepare output filename
    basename = os.path.basename(image_path)
    out_name = os.path.join(output_dir, f"annotated_{basename}")

    if ids is None or len(ids) == 0:
        print(f"No markers detected in {basename}")
        # Optionally save the unmodified image
        cv2.imwrite(out_name, image)
        print(f"Saved copy to {out_name}")
        return

    # Draw detected markers on the image
    cv2.aruco.drawDetectedMarkers(image, corners, ids)

    # Print detected IDs
    print(f"Detected markers in {basename}: {ids.flatten().tolist()}")

    # Save the annotated image
    cv2.imwrite(out_name, image)
    print(f"Annotated image saved to {out_name}")


def main():
    parser = argparse.ArgumentParser(
        description="Detect ArUco markers (DICT_4X4_100) in images using ArucoDetector API."
    )
    parser.add_argument(
        '--images', '-i',
        nargs='+',
        required=True,
        help="Paths to one or more image files."
    )
    parser.add_argument(
        '--output-dir', '-o',
        default='.',
        help="Directory in which to save annotated images."
    )
    args = parser.parse_args()

    # Validate output directory
    output_dir = args.output_dir
    os.makedirs(output_dir, exist_ok=True)

    # Create the ArUco dictionary and detector using new API
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    # Process each image
    for img_path in args.images:
        detect_and_annotate(img_path, detector, output_dir)


if __name__ == "__main__":
    main()
