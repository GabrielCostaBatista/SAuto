#!/usr/bin/env python3
import cv2
import argparse
import os
import sys

def detect_and_annotate(image_path: str, dictionary, parameters, output_dir: str):
    image = cv2.imread(image_path)
    if image is None:
        print(f"[ERROR] Could not open image: {image_path}", file=sys.stderr)
        return

    corners, ids, rejected = cv2.aruco.detectMarkers(image, dictionary, parameters=parameters)

    basename = os.path.basename(image_path)
    out_name = os.path.join(output_dir, f"annotated_{basename}")

    if ids is None or len(ids) == 0:
        print(f"No markers detected in {basename}")
        cv2.imwrite(out_name, image)
        print(f"Saved copy to {out_name}")
        return

    cv2.aruco.drawDetectedMarkers(image, corners, ids)
    print(f"Detected markers in {basename}: {ids.flatten().tolist()}")
    cv2.imwrite(out_name, image)
    print(f"Annotated image saved to {out_name}")

def main():
    parser = argparse.ArgumentParser(
        description="Detect ArUco markers (DICT_4X4_100) in images using legacy API."
    )
    parser.add_argument('--images', '-i', nargs='+', required=True,
                        help="Paths to one or more image files.")
    parser.add_argument('--output-dir', '-o', default='.',
                        help="Directory in which to save annotated images.")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    parameters = cv2.aruco.DetectorParameters_create()

    for img_path in args.images:
        detect_and_annotate(img_path, dictionary, parameters, args.output_dir)

if __name__ == "__main__":
    main()
