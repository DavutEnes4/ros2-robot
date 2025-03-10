import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/encoder/Desktop/ros2-robot/install/lidar_mapper'
