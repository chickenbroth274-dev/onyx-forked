# Camera-Based Target Perception – TEKNOFEST UGV

## 1. Project Overview

This module is being developed as part of the TEKNOFEST Unmanned Ground Vehicle (UGV) project.

The objective is to develop a camera-based perception system capable of identifying the competition target and calculating its centre position in camera images.

The current implementation focuses on the mathematical foundation of target-centre localisation. Target detection and ROS 2 camera integration are planned as subsequent development stages.

## 2. Competition Target

According to the TEKNOFEST rulebook, the laser-fire mission uses an A3-sized target placed at a distance of 10 metres.

The target consists of three concentric circular discs:

| Target region | Diameter | Radius |
|---|---:|---:|
| Inner disc | 6 cm | 3 cm |
| Middle disc | 12 cm | 6 cm |
| Outer disc | 18 cm | 9 cm |

All three circles share a common centre.

The perception module aims to calculate this common centre from the camera image.

## 3. Development Environment

The development environment consists of:

- Operating System: Ubuntu 22.04
- Robotics Middleware: ROS 2 Humble
- Programming Language: Python 3
- Image Processing Library: OpenCV
- Numerical Computing: NumPy
- ROS 2 Image Processing: CV Bridge
- Build System: Colcon

## 4. Repository Structure

The implementation is developed inside the existing `turret_control` ROS 2 Python package.

```text
onyx-forked/
│
└── src/
    └── turret_control/
        │
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── LICENSE
        │
        ├── resource/
        │   └── turret_control
        │
        ├── turret_control/
        │   ├── __init__.py
        │   └── target_localizer.py
        │
        └── test/
            ├── test_copyright.py
            ├── test_flake8.py
            ├── test_pep257.py
            └── test_target_localizer.py
```

## 5. Work Completed

### 5.1 ROS 2 Package Configuration

Inspected the existing `turret_control` package and verified its dependencies.

The package includes:

- `rclpy` – ROS 2 Python client library.
- `sensor_msgs` – Sensor and camera image messages.
- `geometry_msgs` – Geometric message types.
- `std_msgs` – Standard ROS 2 messages.
- `cv_bridge` – Conversion between ROS images and OpenCV images.
- `python3-numpy` – Numerical operations.
- `python3-opencv` – Image processing.

Updated `setup.py` to register the future camera-perception executable:

```python
entry_points={
    'console_scripts': [
        'turret_vision = turret_control.camera_node:main',
    ],
},
```

The executable will become available after `camera_node.py` is implemented.

### 5.2 Target Centre Localisation

Created:

`turret_control/target_localizer.py`

This module provides two functions.

**A. calculate_center()**

Calculates the centre coordinates of a bounding box.

Inputs:
- `x`: Left coordinate.
- `y`: Top coordinate.
- `width`: Bounding box width.
- `height`: Bounding box height.

Mathematical calculation:

```text
center_x = x + width / 2
center_y = y + height / 2
```

Returns:

```python
(center_x, center_y)
```

The function also validates that bounding-box width and height are positive.

**B. calculate_error()**

Calculates the displacement of the detected target centre relative to the image centre.

Inputs:
- Target centre coordinates.
- Image width.
- Image height.

Calculations:

```text
image_center_x = image_width / 2
image_center_y = image_height / 2

error_x = center_x - image_center_x
error_y = center_y - image_center_y
```

Returns:

```python
(error_x, error_y)
```

Coordinate interpretation:

- Positive X: Target is to the right of image centre.
- Negative X: Target is to the left of image centre.
- Positive Y: Target is below image centre.
- Negative Y: Target is above image centre.
- Zero X and Y: Target centre coincides with image centre.

The function also validates image dimensions.

## 6. Example Calculation

Consider a camera image with resolution:

```text
Width  = 1280 pixels
Height = 720 pixels
```

Suppose a detected bounding box has:

```text
x      = 400
y      = 200
width  = 200
height = 200
```

The calculated target centre is:

```text
center_x = 400 + 200 / 2 = 500
center_y = 200 + 200 / 2 = 300
```

The image centre is:

```text
image_center_x = 640
image_center_y = 360
```

Therefore, the pixel displacement is:

```text
error_x = 500 - 640 = -140
error_y = 300 - 360 = -60
```

Result:

```python
(-140.0, -60.0)
```

This indicates that the target centre is 140 pixels to the left and 60 pixels above the image centre.

These values represent pixel displacement, not physical distance or angular error.

## 7. Unit Testing

Created:

`test/test_target_localizer.py`

The following cases are tested:

| Test | Purpose |
|---|---|
| `test_calculate_center` | Checks bounding-box centre calculation. |
| `test_calculate_error` | Checks horizontal and vertical pixel displacement. |
| `test_image_center` | Checks zero error when target and image centres coincide. |
| `test_invalid_bounding_box` | Checks validation of invalid bounding-box dimensions. |
| `test_invalid_image_dimensions` | Checks validation of invalid image dimensions. |

### Test Execution

Build the package:

```bash
cd ~/onyx-forked

source /opt/ros/humble/setup.bash

colcon build --packages-select turret_control
```

Source the workspace:

```bash
source install/setup.bash
```

Run the tests:

```bash
colcon test --packages-select turret_control
```

View the results:

```bash
colcon test-result --verbose
```

### Latest Test Results

| Metric | Result |
|---|---:|
| Total tests | 8 |
| Errors | 0 |
| Failures | 0 |
| Skipped | 1 |
| Package test status | Passed |

The package also passed its Flake8 and PEP257 style checks.

## 8. Current Development Status

| Component | Status |
|---|---|
| ROS 2 package inspection | Completed |
| Dependency verification | Completed |
| Package executable registration | Completed |
| Target-centre calculation | Implemented |
| Pixel displacement calculation | Implemented |
| Localisation unit tests | Passed |
| Target detection | Pending validation and integration |
| Camera image subscription | Pending |
| ROS 2 detection publishing | Pending |
| Real-camera testing at 10 metres | Pending |

## 9. Future Development

The next development stages are:

1. Implement concentric-circle target detection using OpenCV.
2. Validate the detection algorithm using actual competition-target photographs.
3. Integrate target detection and localisation into a ROS 2 camera node.
4. Subscribe to the aiming camera image topic.
5. Publish target detection status and pixel centre coordinates.
6. Generate annotated camera images for visual verification.
7. Test detection reliability under different lighting conditions and camera viewpoints.

## 10. Scope of This Module

The current development is limited to camera-based image processing and target-centre localisation.

It does not currently implement turret actuation, laser activation or autonomous aiming control.

The objective is to provide reliable visual target-position information for integration with the wider UGV system.
