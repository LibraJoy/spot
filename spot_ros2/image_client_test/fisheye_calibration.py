import cv2
assert cv2.__version__[0] >= '3', 'The fisheye module requires opencv version >= 3.0.0'
import numpy as np
import os
import glob

# Checkerboard: 14 horizontal x 23 vertical internal corners
# OpenCV format: (rows, cols) = (vertical, horizontal)
CHECKERBOARD = (23, 14)

SOURCE = 'c1'

subpix_criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW
objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
_img_shape = None
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
# Use all images from both directories
images = glob.glob(f'images_{SOURCE}/*.png')
print(f"Processing {len(images)} images...")
print(f"Looking for {CHECKERBOARD[0]}x{CHECKERBOARD[1]} checkerboard (rows x cols)")
print("=" * 70)

for idx, fname in enumerate(images):
    img = cv2.imread(fname)
    if _img_shape == None:
        _img_shape = img.shape[:2]
    else:
        assert _img_shape == img.shape[:2], "All images must share the same size."
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Enhance contrast for better detection with fisheye distortion
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
    gray_enhanced = clahe.apply(gray)

    print(f"\n[{idx+1}/{len(images)}] {os.path.basename(fname)}")

    # Try original orientation first
    ret, corners = cv2.findChessboardCorners(gray_enhanced, CHECKERBOARD,
                                            cv2.CALIB_CB_ADAPTIVE_THRESH+
                                            cv2.CALIB_CB_NORMALIZE_IMAGE)

    # If not found, try swapped dimensions
    if not ret:
        ret, corners = cv2.findChessboardCorners(gray_enhanced, (CHECKERBOARD[1], CHECKERBOARD[0]),
                                                cv2.CALIB_CB_ADAPTIVE_THRESH+
                                                cv2.CALIB_CB_NORMALIZE_IMAGE)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        cv2.cornerSubPix(gray, corners, (3,3), (-1,-1), subpix_criteria)
        imgpoints.append(corners)
        print(f"  ✓ Found all {CHECKERBOARD[0]*CHECKERBOARD[1]} corners")
    else:
        print(f"  ✗ Failed to find corners")
N_OK = len(objpoints)

print("\n" + "=" * 70)
print(f"Successfully processed {N_OK}/{len(images)} images")

if N_OK < 5:
    print("\n✗ ERROR: Need at least 5 valid images for calibration!")
    exit(1)

print(f"\nCalibrating fisheye camera with {N_OK} images...")

K = np.zeros((3, 3))
D = np.zeros((4, 1))
rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]

rms, _, _, _, _ = \
    cv2.fisheye.calibrate(
        objpoints,
        imgpoints,
        gray.shape[::-1],
        K,
        D,
        rvecs,
        tvecs,
        calibration_flags,
        (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    )

print("\n" + "=" * 70)
print(f"FISHEYE CALIBRATION RESULTS - CAMERA {SOURCE}")
print("=" * 70)

print(f"\nImages used: {N_OK}")
print(f"Image size: {_img_shape[1]}x{_img_shape[0]} (W x H)")
print(f"RMS reprojection error: {rms:.6f} pixels")

print(f"\nCamera Matrix (K):")
print(f"  fx = {K[0,0]:.6f}")
print(f"  fy = {K[1,1]:.6f}")
print(f"  cx = {K[0,2]:.6f}")
print(f"  cy = {K[1,2]:.6f}")

print(f"\nFisheye Distortion Coefficients (D):")
print(f"  k1 = {D[0,0]:.10f}")
print(f"  k2 = {D[1,0]:.10f}")
print(f"  k3 = {D[2,0]:.10f}")
print(f"  k4 = {D[3,0]:.10f}")

print("\n" + "=" * 70)
print("NUMPY FORMAT:")
print("=" * 70)
print("DIM=" + str(_img_shape[::-1]))
print("K=np.array(" + str(K.tolist()) + ")")
print("D=np.array(" + str(D.tolist()) + ")")

# Save to JSON
import json
results = {
    'camera': SOURCE,
    'model': 'fisheye',
    'images_used': N_OK,
    'total_images': len(images),
    'image_width': int(_img_shape[1]),
    'image_height': int(_img_shape[0]),
    'rms_error': float(rms),
    'K': K.tolist(),
    'D': D.tolist(),
    'fx': float(K[0,0]),
    'fy': float(K[1,1]),
    'cx': float(K[0,2]),
    'cy': float(K[1,2]),
    'k1': float(D[0,0]),
    'k2': float(D[1,0]),
    'k3': float(D[2,0]),
    'k4': float(D[3,0])
}

with open(f'{SOURCE}_fisheye_calibration.json', 'w') as f:
    json.dump(results, f, indent=2)

print(f"\n✓ Results saved to: {SOURCE}_fisheye_calibration.json")
print("=" * 70)