
ros2 launch multi_launch multi_launch.py

ros2 run serial_pubsub cPublisher
ros2 run lidar_maper subcriber_lidar
ros2 launch tello_ros2 tello_node.launch.py
ros2 run tello_ros2 yolo_object_detection.py