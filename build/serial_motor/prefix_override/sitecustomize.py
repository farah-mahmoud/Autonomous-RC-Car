import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/farah/farah/GP_Repo/Autonomous-RC-Car/install/serial_motor'
