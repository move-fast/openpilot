# TUV Test Setup

**Hardware needed**: Comma device running openpilot, laptop, joystick

### Steps

1. Connect a joystick to your PC.
2. Connect your laptop to your comma device's hotspot and open a new SSH shell. We need to write a parameter to let controlsd know to start in externalControl mode and configure the TuvTest Parameters if needed:
   Edit `tuvtesconfig.py` with the desired tuv test parameters, then:
  ```shell
    # on your comma device
    tools/tuvtest/tuvtestconfig.py
  ```
1. Run bridge with your laptop's IP address in a tmux session for all relevant topics:
  ```shell
    # on your comma device
    cereal/messaging/bridge {LAPTOP_IP} externalControl
    cereal/messaging/bridge {LAPTOP_IP} carState
    cereal/messaging/bridge {LAPTOP_IP} carControl
  ```
1. Run the ROS2 nodes on the PC.
  ```shell
    # on your laptop
    ros2 run teleop remote.py
    PYTHONPATH=$PYTHONPATH:/path/to/cereal_root ros2 run tuv_test tuv_test_node --ros-args -p log_folder:=/path/to/folder -p comma_ip:={COMMA_IP}
  ```
1. Start the car and Engage OpenPilot, you should be able to control the car from the Joystick.