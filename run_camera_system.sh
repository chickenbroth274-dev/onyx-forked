#!/bin/bash

# Navigate to project
cd ~/onyx-forked || exit 1

echo "Starting Camera Perception System..."

# 1. Start DroidCam
gnome-terminal --title="DroidCam" -- bash -lc '
sudo modprobe v4l2loopback devices=1 video_nr=2 card_label="DroidCam" exclusive_caps=1
/usr/local/bin/droidcam -dev=/dev/video2
exec bash
'

echo "DroidCam started."
echo "Connect your Android phone."

# Wait for camera connection
read -p "Press ENTER after connecting DroidCam..."

# 2. Start ROS 2 camera node
gnome-terminal --title="Turret Vision Node" -- bash -lc '
source /opt/ros/humble/setup.bash
source ~/onyx-forked/install/setup.bash
ros2 run turret_control turret_vision
exec bash
'

sleep 3

# 3. Start RQT image viewer
gnome-terminal --title="RQT Image View" -- bash -lc '
source /opt/ros/humble/setup.bash
source ~/onyx-forked/install/setup.bash
ros2 run rqt_image_view rqt_image_view
exec bash
'

# 4. Monitor debug image publishing rate
gnome-terminal --title="Image Topic Rate" -- bash -lc '
source /opt/ros/humble/setup.bash
source ~/onyx-forked/install/setup.bash
ros2 topic hz /turret/debug_image
exec bash
'

echo "Camera perception system launched."
