import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/projet_sysmer/ros2_ws/install/autonomous_rov'
