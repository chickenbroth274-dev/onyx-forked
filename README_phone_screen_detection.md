# Phone-screen bullseye detection update

Apply this update on the `improve-phone-screen-detection` branch only.

## Files changed

- `src/turret_control/turret_control/target_detector.py`
  - Keeps the original strict 1:2:3 detector as the first detection path for the expected competition target.
  - Adds a fallback for oblique/phone-screen targets whose visible rings project as ellipses and do not have a 1:2:3 image-space radius ratio.
  - Allows a two-ring fallback only when a compact saturated red/yellow/green centre region supports the candidate.
  - Runs the fallback at a maximum width of 640 px and maps the detected centre/radii back to the input image coordinates.
  - Preserves the return keys: `center`, `radii`, and `score`.

- `src/turret_control/turret_control/camera_node.py`
  - Keeps the existing ROS topics and message types.
  - Avoids a redundant full-frame copy before detection.
  - Creates, resizes and converts the debug image only when a debug-image subscriber exists.
  - Continues processing only the latest camera frame, without building a frame queue.

`target_localizer.py` is unchanged.

## Install

From the repository root, unzip this archive so it overwrites the two matching Python files. Then rebuild:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select turret_control --symlink-install
source install/setup.bash
```

Start the system using the existing startup script:

```bash
./run_camera_system.sh
```

## Validation performed while preparing this update

- Python syntax compilation passed for the two updated files and the unchanged localizer.
- A synthetic 1:2:3 test target was detected at approximately `(640.0, 360.0)` in a 1280x720 image.
- The supplied RQT screenshot's image viewport was cropped and resized to 1280x720; the fallback detected its phone-screen bullseye at approximately `(1117.7, 459.2)` in that cropped frame.
- The same fallback's measured detector median was about 16 ms on the prepared screenshot crop in this environment. This is not an end-to-end ROS FPS measurement.

The RQT screenshot is not the original raw camera frame. Final verification must be done with the live DroidCam feed, including the actual A3 competition target at the required distance. The fallback is intended to improve robustness, not to certify competition accuracy.
