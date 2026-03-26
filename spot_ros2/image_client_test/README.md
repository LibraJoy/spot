# Spot CAM+ Fisheye Camera Calibration

**Status:** 📦 ARCHIVED (March 26, 2026)

Complete pipeline for calibrating Boston Dynamics Spot CAM+ fisheye cameras and obtaining distortion coefficients (k1-k4).

---

## Quick Start

See **[QUICKSTART.md](QUICKSTART.md)** for fast reference.

---

## Directory Structure

```
image_client_test/
├── *.py                          # Python scripts (calibration pipeline)
├── *.json                        # Calibration results
├── images_c1/, images_c2/        # Checkerboard calibration images
├── images_undistorted_*/         # Undistorted output
├── reprojection_analysis/        # Quality validation plots
├── test_images/                  # Sample camera outputs
├── archived/                     # Legacy files
├── README.md                     # This file
└── QUICKSTART.md                 # Fast reference
```

---

## Pipeline Scripts

1. **test_image_client_spot_cam.py** - Retrieve images via ImageClient API
   - Shows single camera or multi-camera streams
   - Includes image saving functionality (uncomment in `show_single()`)

2. **fisheye_calibration.py** - Calculate K matrix and k1-k4 distortion
   - Set `SOURCE = 'c1'` to select camera
   - Uses checkerboard: 14×23 internal corners
   - Outputs: `{SOURCE}_fisheye_calibration.json`

3. **visualize_reprojection_error.py** - Validate calibration quality
   - Calculates reprojection errors
   - Generates analysis plots and annotated images
   - Outputs: `{SOURCE}_reprojection_analysis.png`

4. **undistort_with_k14.py** - Apply fisheye correction
   - Loads calibration JSON
   - Undistorts images without cropping
   - Outputs: `images_undistorted_{SOURCE}/`

5. **test_image_client_ros2_pub.py** - ROS2 publisher with calibration
   - Publishes `/camera/image_raw` and `/camera/camera_info`
   - Uses 'equidistant' distortion model
   - Optimized for ~7-8 Hz

---

## Complete Workflow

### 1. Collect Calibration Images

Print checkerboard (14 horizontal × 23 vertical internal corners) on rigid surface.

```bash
python3 test_image_client_spot_cam.py
# Uncomment save section in show_single() to capture images
# Take 20-30 photos from different angles/distances
# Save to: images_c1/
```

### 2. Run Calibration

```bash
# Edit fisheye_calibration.py: set SOURCE = 'c1'
python3 fisheye_calibration.py

# Output: c1_fisheye_calibration.json
# Check: RMS error should be < 0.5 pixels
```

### 3. Validate Quality

```bash
# Edit visualize_reprojection_error.py: set SOURCE = 'c1'
python3 visualize_reprojection_error.py

# Outputs:
#   - reprojection_analysis/c1_reprojection_analysis.png
#   - reprojection_analysis/c1_reprojection_images/
```

### 4. Undistort Images

```bash
# Edit undistort_with_k14.py: set SOURCE = 'c1'
python3 undistort_with_k14.py

# Output: images_undistorted_c1/
```

### 5. Use in ROS2 (Optional)

```bash
python3 test_image_client_ros2_pub.py
# Publishes calibrated camera stream at ~7-8 Hz
```

---

## ImageClient API Reference

### Setup

```python
import bosdyn.client
import bosdyn.client.spot_cam as spot_cam
from bosdyn.client.image import build_image_request

sdk = bosdyn.client.create_standard_sdk('ImageApp')
spot_cam.register_all_service_clients(sdk)

robot = sdk.create_robot('10.0.0.3')
robot.authenticate('user', 'password')

image_client = robot.ensure_client('spot-cam-image')
```

### Get Single Image

```python
# Available sources: 'c0', 'c1', 'c2', 'c3', 'c4', 'pano', 'ptz', etc.
request = build_image_request(
    'c1',                    # Camera name
    image_format=2,          # 1=JPEG, 2=RAW (RGB888)
    quality_percent=75,      # JPEG quality (ignored for RAW)
    resize_ratio=1.0         # Scale: 0.1 to 1.0
)

response = image_client.get_image([request])[0]
img_proto = response.shot.image

# Convert to numpy (RAW format)
import numpy as np
img_array = np.frombuffer(img_proto.data, dtype=np.uint8)
img = img_array.reshape((img_proto.rows, img_proto.cols, 3))  # RGB
```

### Get Multi-Camera

```python
requests = [
    build_image_request(f'c{i}', image_format=2, resize_ratio=0.5)
    for i in range(4)
]
responses = image_client.get_image(requests)
```

### Camera Intrinsics

```python
source = response.source
intrinsics = source.pinhole.intrinsics

fx = intrinsics.focal_length.x
fy = intrinsics.focal_length.y
cx = intrinsics.principal_point.x
cy = intrinsics.principal_point.y
```

**Note:** Robot does NOT provide distortion coefficients (k1-k4). Manual calibration required.

### Performance

| Configuration | FPS (1 cam) | FPS (4 cams) |
|---------------|-------------|--------------|
| RAW, 100% res | ~10 Hz      | ~2 Hz        |
| RAW, 50% res  | ~11 Hz      | ~4 Hz        |
| RAW, 30% res  | ~14 Hz      | ~7 Hz        |
| pano (stitched) | ~6-7 Hz   | N/A          |

**Hardware limitation:** Cannot sustain 10 Hz for 4+ cameras simultaneously.

---

## Calibration Results

### Camera c1
```json
{
  "rms_error": 0.355186,
  "fx": 732.680668, "fy": 732.483949,
  "cx": 991.069162, "cy": 555.717526,
  "k1": -0.0423058809, "k2": -0.0172994588,
  "k3": 0.0112229483, "k4": -0.0019479573
}
```
✅ RMS: 0.355 pixels (Excellent)

### Camera c2
```json
{
  "rms_error": 0.432112,
  "fx": 729.390015, "fy": 733.512451,
  "cx": 985.229675, "cy": 543.890869,
  "k1": -0.0470598862, "k2": -0.0034390844,
  "k3": 0.0035446882, "k4": -0.0004189369
}
```
✅ RMS: 0.432 pixels (Excellent)

**Quality standard:** < 0.5 pixels = Excellent ✅ | 0.5-1.0 px = Good | > 1.0 px = Poor

---

## Key Findings

### Hardware
- **Cameras:** 5 fisheye cameras (c0-c4), 1920×1080
- **Model:** Equidistant fisheye projection (Kannala-Brandt)
- **API:** ImageClient (`spot-cam-image` service)

### Performance
- Single camera: ~10 Hz (RAW format)
- 4 cameras parallel: ~2 Hz (hardware limited)
- Panorama ('pano'): ~6-7 Hz (5 cameras stitched) (see `./test_iamges/pano.png`)

### Robot-Provided Parameters
- ✅ Provides intrinsics (fx, fy, cx, cy)
- ✅ Provides extrinsics (camera poses in transform tree)
- ❌ Does NOT provide distortion coefficients (k1-k4)
- ✅ Manual calibration required for k1-k4

### Calibration Quality
- c1 and c2: RMS < 0.5 pixels (excellent)
- Reprojection errors well within acceptable range
- Undistortion successfully removes fisheye distortion

---

## Troubleshooting

**Checkerboard not detected?**
- Ensure 14×23 internal corners
- Improve lighting, reduce glare
- Try different angles/distances

**High RMS error (> 1.0 px)?**
- Collect 20-30 images minimum
- Vary angles and distances more
- Ensure pattern is flat and rigid

**Cropped undistortion?**
- Use `Knew=K` in cv2.fisheye.undistortImage
- Don't use estimateNewCameraMatrixForUndistortRectify

**Low ROS2 frame rate?**
- Use RAW format (not JPEG)
- Skip cv_bridge conversion (direct memory copy)
- Lower resize_ratio for smaller images

---

## Requirements

```bash
pip3 install opencv-python numpy matplotlib bosdyn-client bosdyn-api
pip3 install rclpy sensor_msgs cv_bridge  # For ROS2
```

---

## Authors

**Shiyu Chen** - March 2026

**Status:** ✅ Complete & Archived

Pipeline successfully calibrated Spot CAM+ fisheye cameras with excellent reprojection errors (< 0.5 pixels).
