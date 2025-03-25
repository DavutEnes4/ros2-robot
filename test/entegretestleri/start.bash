#!/bin/bash

# Function to open a new terminal and run a command
run_in_terminal() {
    local command="$1"
    local title="$2"

    # Check if gnome-terminal is available
    if command -v gnome-terminal &> /dev/null; then
        gnome-terminal --title="$title" -- bash -c "$command; exec bash"
    else
        echo "Error: gnome-terminal is not installed. Please install it or use another terminal emulator."
        exit 1
    fi
}

# Source the ROS2 workspace
source ~/Desktop/ros2-robot/ros2_ws/install/setup.bash


# 2. Komut - cPublisher
run_in_terminal "ros2 run serial_pubsub cPublisher" "cPublisher" &
sleep 2

# 3. Komut - subscriber_lidar
run_in_terminal "ros2 run lidar_maper subscriber_lidar" "subscriber_lidar" &
sleep 2

# 4. Komut - tello_node.launch.py
run_in_terminal "ros2 launch tello_ros2 tello_node.launch.py" "tello_node" &
sleep 2

# 5. Komut - yolo_object_detection.py
run_in_terminal "ros2 run tello_ros2 yolo_object_detection.py" "yolo_object_detection" &
sleep 2

echo "Tüm komutlar farklı terminallerde başlatıldı."