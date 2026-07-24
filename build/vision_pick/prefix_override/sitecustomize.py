import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hosea/Git Directory/Vision-Based-Pick-and-Place/install/vision_pick'
