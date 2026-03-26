# Fisheye Calibration - Quick Start

5-step pipeline for calibrating Spot CAM+ fisheye cameras.

---

## Prerequisites

```bash
pip3 install opencv-python numpy matplotlib bosdyn-client
```

Print checkerboard: 14 horizontal × 23 vertical internal corners on rigid surface.

---

## Pipeline

### 1. Collect Images (20-30 photos)

```bash
python3 test_image_client_spot_cam.py
# Uncomment save section in show_single()
# Capture checkerboard from different angles/distances
# Save to: images_c1/
```

### 2. Calibrate

```bash
# Edit: SOURCE = 'c1' in fisheye_calibration.py
python3 fisheye_calibration.py

# Output: c1_fisheye_calibration.json
# Check: RMS error < 0.5 pixels ✅
```

### 3. Validate

```bash
# Edit: SOURCE = 'c1' in visualize_reprojection_error.py
python3 visualize_reprojection_error.py

# Output: reprojection_analysis/c1_reprojection_analysis.png
# Check: Mean error < 0.5 pixels ✅
```

### 4. Undistort

```bash
# Edit: SOURCE = 'c1' in undistort_with_k14.py
python3 undistort_with_k14.py

# Output: images_undistorted_c1/
```

### 5. ROS2 (Optional)

```bash
python3 test_image_client_ros2_pub.py
# Publishes /camera/image_raw + /camera/camera_info
```

---

## Expected Results

| Step | Output | Quality Check |
|------|--------|---------------|
| Calibration | c1_fisheye_calibration.json | RMS < 0.5 px ✅ |
| Validation | PNG plots + images | Mean < 0.5 px ✅ |
| Undistortion | Corrected images | Lines straight ✅ |

---

## Reference Results

**c1:** RMS 0.355 px, fx=732.68, fy=732.48, k1=-0.042, k2=-0.017, k3=0.011, k4=-0.002

**c2:** RMS 0.432 px, fx=729.39, fy=733.51, k1=-0.047, k2=-0.003, k3=0.004, k4=-0.0004

---

## Troubleshooting

- **No checkerboard found?** → Improve lighting, ensure 14×23 corners
- **High RMS (> 1.0 px)?** → Need more images (20-30), vary angles
- **Cropped output?** → Use `Knew=K` in undistort script

---

See **[README.md](README.md)** for full documentation.
