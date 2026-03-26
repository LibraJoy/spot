#!/usr/bin/env python3
"""
Undistort fisheye images using calibration results
Loads calibration from c2_fisheye_calibration.json
"""

import cv2
import numpy as np
import os
import glob
import json

# =============================================================================
# CONFIGURATION
# =============================================================================

SOURCE = 'c2'

# Input calibration file
CALIBRATION_FILE = f'{SOURCE}_fisheye_calibration.json'

# Image directories
IMAGE_DIR = f'images_{SOURCE}/*.png'

# Output directory
OUTPUT_DIR = f'images_undistorted_{SOURCE}'

# Balance parameter for undistortion
# 1.0 = keep all pixels (may have black borders)
# 0.0 = crop to valid region only (no black borders)
BALANCE = 0.6

# =============================================================================
# LOAD CALIBRATION
# =============================================================================

print("=" * 70)
print("FISHEYE IMAGE UNDISTORTION")
print("=" * 70)

# Load calibration data
print(f"\nLoading calibration from: {CALIBRATION_FILE}")
with open(CALIBRATION_FILE, 'r') as f:
    calib = json.load(f)

# Extract calibration parameters
DIM = (calib['image_width'], calib['image_height'])  # (width, height)
K = np.array(calib['K'], dtype=np.float64)
D = np.array(calib['D'], dtype=np.float64)

print(f"\nCalibration data:")
print(f"  Image size: {DIM[0]}x{DIM[1]} (W x H)")
print(f"  Camera: {calib['camera']}")
print(f"  Model: {calib['model']}")
print(f"  RMS error: {calib['rms_error']:.6f} pixels")

print(f"\nCamera Matrix (K):")
print(f"  fx = {K[0,0]:.6f}")
print(f"  fy = {K[1,1]:.6f}")
print(f"  cx = {K[0,2]:.6f}")
print(f"  cy = {K[1,2]:.6f}")

print(f"\nDistortion Coefficients (D):")
print(f"  k1 = {D[0,0]:.10f}")
print(f"  k2 = {D[1,0]:.10f}")
print(f"  k3 = {D[2,0]:.10f}")
print(f"  k4 = {D[3,0]:.10f}")

# =============================================================================
# UNDISTORT FUNCTION
# =============================================================================

def undistort_image(img_path, K, D, DIM, balance=0.0, dim2=None, dim3=None):
    """
    Undistort fisheye image

    Args:
        img_path: Path to input image
        K: Camera matrix from calibration
        D: Distortion coefficients
        DIM: Calibration image dimensions (width, height)
        balance: 0.0=keep all pixels, 1.0=crop to valid only
        dim2: Output dimension for new_K estimation (default: same as input)
        dim3: Output dimension for maps (default: same as input)

    Returns:
        undistorted_img: Undistorted image
    """
    img = cv2.imread(img_path)
    dim1 = img.shape[:2][::-1]  # (width, height) of input image

    # Check aspect ratio matches calibration
    if abs(dim1[0]/dim1[1] - DIM[0]/DIM[1]) > 0.01:
        print(f"  ⚠ Warning: Aspect ratio mismatch!")
        print(f"    Input: {dim1[0]/dim1[1]:.3f}, Calibration: {DIM[0]/DIM[1]:.3f}")

    # Default dimensions
    if dim2 is None:
        dim2 = dim1
    if dim3 is None:
        dim3 = dim1

    # Scale K based on image size
    scaled_K = K * dim1[0] / DIM[0]
    scaled_K[2][2] = 1.0  # K[2][2] is always 1.0

    # Use original K as new_K (preserves full image without cropping)
    # estimateNewCameraMatrixForUndistortRectify produces invalid results for this calibration
    # new_K = scaled_K.copy()
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)

    # Create undistortion maps
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        scaled_K, D, np.eye(3), new_K, dim3, cv2.CV_16SC2
    )

    # Undistort
    undistorted_img = cv2.remap(
        img, map1, map2,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT
    )

    return undistorted_img

# =============================================================================
# PROCESS IMAGES
# =============================================================================

print("\n" + "=" * 70)
print("PROCESSING IMAGES")
print("=" * 70)

# Create output directory
os.makedirs(OUTPUT_DIR, exist_ok=True)
print(f"\nOutput directory: {OUTPUT_DIR}/")

# Get all images
images = sorted(glob.glob(IMAGE_DIR))
print(f"Found {len(images)} images to undistort")

if len(images) == 0:
    print(f"\n✗ No images found matching: {IMAGE_DIR}")
    exit(1)

print(f"\nUndistortion settings:")
print(f"  Balance: {BALANCE} (0=keep all pixels, 1=crop to valid)")

# Process images
# cv2.namedWindow("undistorted", cv2.WINDOW_NORMAL)
# cv2.resizeWindow("undistorted", 400, 400)
successful = 0
for idx, img_path in enumerate(images):
    basename = os.path.basename(img_path)

    try:
        # Undistort
        undistorted = undistort_image(img_path, K, D, DIM, balance=BALANCE)

        # Save
        output_path = os.path.join(OUTPUT_DIR, basename)
        cv2.imwrite(output_path, undistorted)

        successful += 1

        if (idx + 1) % 10 == 0 or idx == len(images) - 1:
            print(f"  Processed {idx + 1}/{len(images)} images")
            # display_img = cv2.resize(undistorted, (400, 600))
            # display_img = undistorted
            # cv2.imshow('undistorted', display_img)

            # # cv2.resizeWindow("undistorted", 400, 400)
            # key = cv2.waitKey(0)
            # if key == ord('q'):
            #     cv2.destroyAllWindows()

    except Exception as e:
        print(f"  ✗ Error processing {basename}: {e}")

print(f"\n✓ Successfully undistorted {successful}/{len(images)} images")
print(f"  Output: {OUTPUT_DIR}/")

# =============================================================================
# VERIFICATION
# =============================================================================

if successful > 0:
    # Check first image
    first_undist = cv2.imread(os.path.join(OUTPUT_DIR, os.path.basename(images[0])))
    non_black = np.count_nonzero(first_undist) / first_undist.size

    print(f"\nVerification (first image):")
    print(f"  Non-black pixels: {non_black*100:.1f}%")

    if non_black < 0.5:
        print(f"  ⚠ Warning: Image may be over-cropped!")
        print(f"    Try reducing balance parameter (currently {BALANCE})")
    elif non_black > 0.95:
        print(f"  ✓ Full image preserved")
    else:
        print(f"  ✓ Reasonable crop level")

print("\n" + "=" * 70)
