import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vboxuser/Documents/SoftwarePrakt/SolarSwarm-Node-Visualizer/ros_ws/install/turtle_subscriber'
