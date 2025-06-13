import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jf/tello_tracking_ws/install/yolov5_bytetrack_ros'
