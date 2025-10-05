import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/shamganesh/ros2_ws/src/path_nav/install/path_nav'
