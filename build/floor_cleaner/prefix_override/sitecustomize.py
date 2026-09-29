import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/debarghya/floor_cleaner_ws/install/floor_cleaner'
