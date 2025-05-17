import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/harry/Documents/Real_antosim/AntoSim/antobot_ant_description/install/antobot_ant_description'
