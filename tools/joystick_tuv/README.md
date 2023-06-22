# Joystick

**Hardware needed**: device running openpilot, laptop, joystick (optional)

With tuvjoystickd, you can connect your laptop to your comma device over the network and debug controls using the tuv joystick.

## Usage

The car must be off, and openpilot must be offroad before starting `tuvjoystickd`.

### Joystick on your laptop

In order to use a joystick over the network, we need to run tuvjoystickd locally from your laptop and have it send `externalControl` packets over the network to the comma device.

1. Connect a joystick to your PC.
2. Connect your laptop to your comma device's hotspot and open a new SSH shell. We need to write a parameter to let controlsd know to start in externalControl mode and configure the TuvTest Parameters if needed:
   Edit `tuvtesconfig.py` with the desired tuv test parameters, then:
  ```shell
    # on your comma device
    tools/tuvtest/tuvtestconfig.py
  ```
3. Run bridge with your laptop's IP address. This republishes the `externalControl` packets sent from your laptop so that openpilot can receive them:
   ```shell
   # on your comma device
   cereal/messaging/bridge {LAPTOP_IP} externalControl
   ```
2. Start tuvjoystickd on your laptop in ZMQ mode.
   ```shell
   # on your laptop
   export ZMQ=1
   tools/joystick_tuv/tuvjoystickd.py
   ```

---
Now start your car and openpilot should go into joystick mode with an alert on startup! The status of the axes will display on the alert, while button statuses print in the shell.

Make sure the conditions are met in the panda to allow controls (e.g. cruise control engaged). You can also make a modification to the panda code to always allow controls.

![](steer.gif)
