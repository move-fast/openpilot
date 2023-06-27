# TUV Test Setup

**Hardware needed**: Comma device running openpilot, laptop, joystick

### Steps

1. Connect a joystick to your PC.
2. Connect your laptop to your comma device's hotspot and open a new SSH shell.
3. We need to write a parameter to let `controlsd` know to start in `externalControl` mode and configure the TuvTest Parameters if needed.
   This is done on `tools/tuvtest/tuvtestconfig.py`
   Edit `tuvtestconfig.py` with the desired tuv test parameters:
   - Set `torqueRateLimiting` to `False` to stop OP code from limiting the rate change of the torque commands.
   - Set `accelerationClipping` to `False` to stop OP code from clipping Acceleration commands to the limits.
   - Set `disengageOnBreak` to `False` to stop OP code from disengaging OP when break pedal is pressed.
   - Set `printAcc` and `printSteer` to `True` if you want OP to print out the steer and acc values to the output.
  Then:
  ```shell
    # on your comma device
    python tools/tuvtest/tuvtestconfig.py
  ```
  - The parameters are cleared on OPs transition from OnRoad to OffRoad, so you must always run this script before engaging OP for `externalControlMode` to be enabled.
4. Run bridge with your laptop's IP address in a tmux session for all relevant topics:
  ```shell
    # on your comma device
    ./cereal/messaging/bridge {LAPTOP_IP} externalControl
    ./cereal/messaging/bridge {LAPTOP_IP}
  ```
5. Run the ROS2 nodes on the PC.
  ```shell
    # on your laptop
    ros2 run teleop remote.py
    PYTHONPATH=$PYTHONPATH:/path/to/cereal_root ros2 run tuv_test tuv_test_node --ros-args -p log_folder:=/path/to/folder -p comma_ip:={COMMA_IP}
  ```
6. Start the car and Engage OpenPilot:
   - OP Should start in "External Control Mode" which is indicated in the UI.
   - Once engaged you should see the ACC and Steering control values printed also on the UI.
