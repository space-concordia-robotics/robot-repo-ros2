import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/liliana/ros2_ws/robot-repo-ros2/src/install/object_recognition'
