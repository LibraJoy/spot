#!/usr/bin/env python3
"""
Visualize Fisheye Calibration Reprojection Error

Calculates and visualizes the reprojection error for fisheye camera calibration.
Shows per-image error, per-corner error, and spatial distribution of errors.
"""

import cv2
import numpy as np
import json
import glob
import os
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend for headless systems
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# =============================================================================
# CONFIGURATION
# =============================================================================

SOURCE = 'c1'
CALIBRATION_FILE = f'{SOURCE}_fisheye_calibration.json'
IMAGE_DIR = f'images_{SOURCE}/*.png'
CHECKERBOARD = (23, 14)  # (rows, cols)

# Visualization settings
SHOW_IMAGES = False  # Show each image with reprojection overlay (set False for headless)
SHOW_PLOTS = False   # Show error distribution plots (set False for headless)
SAVE_PLOTS = True    # Save plots to files
SAVE_IMAGES = True   # Save annotated images to files

# =============================================================================
# LOAD CALIBRATION
# =============================================================================

print("=" * 70)
print(f"REPROJECTION ERROR ANALYSIS - CAMERA {SOURCE}")
print("=" * 70)

# Load calibration results
with open(CALIBRATION_FILE, 'r') as f:
    calib = json.load(f)

K = np.array(calib['K'], dtype=np.float64)
D = np.array(calib['D'], dtype=np.float64)

print(f"\nLoaded calibration:")
print(f"  RMS error: {calib['rms_error']:.6f} pixels")
print(f"  Images used: {calib['images_used']}")
print(f"  fx={K[0,0]:.2f}, fy={K[1,1]:.2f}, cx={K[0,2]:.2f}, cy={K[1,2]:.2f}")
print(f"  k1={D[0,0]:.6f}, k2={D[1,0]:.6f}, k3={D[2,0]:.6f}, k4={D[3,0]:.6f}")

# =============================================================================
# PREPARE OBJECT POINTS
# =============================================================================

objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# =============================================================================
# PROCESS IMAGES AND CALCULATE REPROJECTION
# =============================================================================

images = sorted(glob.glob(IMAGE_DIR))
print(f"\nProcessing {len(images)} images...")
print("=" * 70)

objpoints = []
imgpoints = []
rvecs_list = []
tvecs_list = []
image_names = []
successful_images = []

subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)

for idx, fname in enumerate(images):
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Enhance for detection
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    gray_enhanced = clahe.apply(gray)

    print(f"[{idx+1}/{len(images)}] {os.path.basename(fname)}", end=" ")

    # Find corners
    ret, corners = cv2.findChessboardCorners(
        gray_enhanced, CHECKERBOARD,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    )

    if not ret:
        ret, corners = cv2.findChessboardCorners(
            gray_enhanced, (CHECKERBOARD[1], CHECKERBOARD[0]),
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

    if ret:
        # Refine corners
        cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), subpix_criteria)

        objpoints.append(objp)
        imgpoints.append(corners)
        image_names.append(os.path.basename(fname))
        successful_images.append(fname)

        print("✓")
    else:
        print("✗ skipped")

print(f"\n{len(successful_images)}/{len(images)} images with detected corners")

# =============================================================================
# RE-CALIBRATE TO GET ROTATION/TRANSLATION VECTORS
# =============================================================================

print("\nRe-calibrating to get rvecs/tvecs...")

# Re-run calibration to get rvecs and tvecs
N_OK = len(objpoints)
rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]

# Get image shape
test_img = cv2.imread(successful_images[0])
img_shape = test_img.shape[:2]

calibration_flags = (
    cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC +
    cv2.fisheye.CALIB_CHECK_COND +
    cv2.fisheye.CALIB_FIX_SKEW +
    cv2.fisheye.CALIB_FIX_INTRINSIC  # Fix K and D to the loaded values
)

# Calibrate with fixed K and D to get rvecs/tvecs
rms_check, _, _, rvecs, tvecs = cv2.fisheye.calibrate(
    objpoints,
    imgpoints,
    img_shape[::-1],
    K.copy(),
    D.copy(),
    rvecs,
    tvecs,
    calibration_flags,
    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
)

print(f"Re-calibration RMS: {rms_check:.6f} (should match {calib['rms_error']:.6f})")

# =============================================================================
# CALCULATE REPROJECTION ERRORS
# =============================================================================

print("\nCalculating reprojection errors...")

all_errors = []
per_image_errors = []
per_image_max_errors = []

for i, (obj_pts, img_pts) in enumerate(zip(objpoints, imgpoints)):
    rvec = rvecs[i]
    tvec = tvecs[i]

    if rvec is not None:
        rvecs_list.append(rvec)
        tvecs_list.append(tvec)

        # Project object points to image
        projected, _ = cv2.fisheye.projectPoints(
            obj_pts, rvec, tvec, K, D
        )

        # Calculate errors
        projected = projected.reshape(-1, 2)
        detected = img_pts.reshape(-1, 2)

        errors = np.linalg.norm(projected - detected, axis=1)
        all_errors.extend(errors)

        mean_error = np.mean(errors)
        max_error = np.max(errors)

        per_image_errors.append(mean_error)
        per_image_max_errors.append(max_error)

        print(f"  Image {i+1}: mean={mean_error:.4f}px, max={max_error:.4f}px")
    else:
        print(f"  Image {i+1}: pose estimation failed!")
        rvecs_list.append(None)
        tvecs_list.append(None)

# =============================================================================
# STATISTICS
# =============================================================================

all_errors = np.array(all_errors)
per_image_errors = np.array(per_image_errors)

print("\n" + "=" * 70)
print("REPROJECTION ERROR STATISTICS")
print("=" * 70)

print(f"\nOverall (all corners):")
print(f"  Mean error:   {np.mean(all_errors):.6f} pixels")
print(f"  Std dev:      {np.std(all_errors):.6f} pixels")
print(f"  Min error:    {np.min(all_errors):.6f} pixels")
print(f"  Max error:    {np.max(all_errors):.6f} pixels")
print(f"  Median error: {np.median(all_errors):.6f} pixels")

print(f"\nPer-image:")
print(f"  Mean error:   {np.mean(per_image_errors):.6f} pixels")
print(f"  Worst image:  {np.max(per_image_errors):.6f} pixels ({image_names[np.argmax(per_image_errors)]})")
print(f"  Best image:   {np.min(per_image_errors):.6f} pixels ({image_names[np.argmin(per_image_errors)]})")

# =============================================================================
# VISUALIZE ERROR DISTRIBUTION
# =============================================================================

if SHOW_PLOTS or SAVE_PLOTS:
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f'Fisheye Calibration Reprojection Error - Camera {SOURCE}', fontsize=14)

    # 1. Histogram of all errors
    ax = axes[0, 0]
    ax.hist(all_errors, bins=50, edgecolor='black', alpha=0.7)
    ax.axvline(np.mean(all_errors), color='r', linestyle='--', label=f'Mean: {np.mean(all_errors):.4f}px')
    ax.axvline(np.median(all_errors), color='g', linestyle='--', label=f'Median: {np.median(all_errors):.4f}px')
    ax.set_xlabel('Reprojection Error (pixels)')
    ax.set_ylabel('Frequency')
    ax.set_title('Error Distribution (All Corners)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 2. Per-image error bar plot
    ax = axes[0, 1]
    x = np.arange(len(per_image_errors))
    ax.bar(x, per_image_errors, alpha=0.7, edgecolor='black')
    ax.axhline(np.mean(per_image_errors), color='r', linestyle='--', label=f'Mean: {np.mean(per_image_errors):.4f}px')
    ax.set_xlabel('Image Index')
    ax.set_ylabel('Mean Reprojection Error (pixels)')
    ax.set_title('Per-Image Mean Error')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 3. Cumulative distribution
    ax = axes[1, 0]
    sorted_errors = np.sort(all_errors)
    cumulative = np.arange(1, len(sorted_errors) + 1) / len(sorted_errors) * 100
    ax.plot(sorted_errors, cumulative)
    ax.axvline(0.5, color='r', linestyle='--', label='0.5px')
    ax.axvline(1.0, color='orange', linestyle='--', label='1.0px')
    ax.set_xlabel('Reprojection Error (pixels)')
    ax.set_ylabel('Cumulative Percentage (%)')
    ax.set_title('Cumulative Error Distribution')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 4. Box plot comparison
    ax = axes[1, 1]
    ax.boxplot([all_errors], labels=['All Corners'])
    ax.set_ylabel('Reprojection Error (pixels)')
    ax.set_title('Error Distribution (Box Plot)')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    if SAVE_PLOTS:
        plt.savefig(f'{SOURCE}_reprojection_analysis.png', dpi=150, bbox_inches='tight')
        print(f"\n✓ Saved plot: {SOURCE}_reprojection_analysis.png")

    if SHOW_PLOTS:
        plt.show()
    else:
        plt.close()

# =============================================================================
# VISUALIZE ERROR OVERLAY ON IMAGES
# =============================================================================

if SHOW_IMAGES or SAVE_IMAGES:
    print("\nVisualizing reprojection on images...")

    # Create output directory for annotated images
    if SAVE_IMAGES:
        output_dir = f'{SOURCE}_reprojection_images'
        os.makedirs(output_dir, exist_ok=True)

    for i, (fname, obj_pts, img_pts, rvec, tvec) in enumerate(
        zip(successful_images, objpoints, imgpoints, rvecs_list, tvecs_list)
    ):
        if rvec is None:
            continue

        img = cv2.imread(fname)

        # Project points
        projected, _ = cv2.fisheye.projectPoints(obj_pts, rvec, tvec, K, D)
        projected = projected.reshape(-1, 2)
        detected = img_pts.reshape(-1, 2)

        # Calculate errors for coloring
        errors = np.linalg.norm(projected - detected, axis=1)
        max_err = np.max(errors)

        # Draw detected corners (green) and projected corners (red)
        for j, (det, proj, err) in enumerate(zip(detected, projected, errors)):
            # Color based on error (green = low, red = high)
            error_ratio = err / max(max_err, 1.0)
            color = (0, int(255 * (1 - error_ratio)), int(255 * error_ratio))

            # Draw detected corner
            cv2.circle(img, tuple(det.astype(int)), 5, (0, 255, 0), -1)

            # Draw projected corner
            cv2.circle(img, tuple(proj.astype(int)), 3, color, -1)

            # Draw error line
            cv2.line(img, tuple(det.astype(int)), tuple(proj.astype(int)), color, 1)

        # Add info text
        mean_err = np.mean(errors)
        cv2.putText(img, f"Image {i+1}/{len(successful_images)}: {os.path.basename(fname)}",
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(img, f"Mean error: {mean_err:.4f}px, Max: {max_err:.4f}px",
                   (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(img, "Green=detected, Red/Yellow=projected",
                   (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Save annotated image
        if SAVE_IMAGES:
            output_path = os.path.join(output_dir, f'reproj_{i+1:02d}_{os.path.basename(fname)}')
            cv2.imwrite(output_path, img)

        # Display if requested
        if SHOW_IMAGES:
            display_scale = 0.5
            img_display = cv2.resize(img, None, fx=display_scale, fy=display_scale)
            cv2.imshow('Reprojection Error Visualization', img_display)

            key = cv2.waitKey(0)
            if key == 27:  # ESC to quit
                break

    if SHOW_IMAGES:
        cv2.destroyAllWindows()

    if SAVE_IMAGES:
        print(f"\n✓ Saved {len(successful_images)} annotated images to: {output_dir}/")

print("\n" + "=" * 70)
print("✓ Analysis complete!")
print("=" * 70)
