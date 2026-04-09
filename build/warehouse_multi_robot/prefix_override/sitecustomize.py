import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/canozkan/thesis_ws/install/warehouse_multi_robot'
