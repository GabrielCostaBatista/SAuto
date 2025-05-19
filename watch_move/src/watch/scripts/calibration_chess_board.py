#!/usr/bin/env python3
import numpy as np
import cv2
import glob
import os

# === CONFIGURATION ===
chessboard_size = (8, 6)  # Number of inner corners (columns, rows)
square_size = 10         # Real-world square size (can be 1.0 if units don’t matter)
image_folder = "calibration_images"  # Folder containing calibration images
output_file = "~/SAuto/watch_move/src/watch/camera_info/camera.yaml"  # Where to save calibration

# === Prepare object points like (0,0,0), (1,0,0), ..., (8,5,0) ===
objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

# === Storage for object points and image points ===
objpoints = []  # 3D real-world points
imgpoints = []  # 2D image points

# === Load images ===
images = []
for ext in ('*.jpg', '*.jpeg', '*.png'):
    images.extend(glob.glob(os.path.join(image_folder, ext)))

if not images:
    print(f"[ERROR] No images found in {image_folder}")
    exit(1)

print(f"[INFO] Found {len(images)} images. Starting detection...")

# === Process each image ===
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )
        imgpoints.append(corners2)
        print(f"[OK] Corners found in {os.path.basename(fname)}")
    else:
        print(f"[WARN] Chessboard not found in {os.path.basename(fname)}")

# === Calibrate camera ===
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("\n[RESULT] Camera matrix:")
print(camera_matrix)
print("\n[RESULT] Distortion coefficients:")
print(dist_coeffs.ravel())

# === Save calibration to YAML ===
fs = cv2.FileStorage(output_file, cv2.FILE_STORAGE_WRITE)
fs.write("camera_matrix", camera_matrix)
fs.write("distortion_coefficients", dist_coeffs)
fs.release()

print(f"\n✅ Calibration saved to '{output_file}'")