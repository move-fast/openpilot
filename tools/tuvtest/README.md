# TUV Test setup

**Hardware needed**: device running openpilot, laptop

With tuvtestd, you can connect your laptop to your comma device over the network and send tuv test commands via joystick.

## Usage

The car must be off, and openpilot must be offroad before starting `tuvtestd`.

### Using a joystick

SSH into your comma device and start tuvtestd with the following command:

```shell
tools/tuvtest/tuvtestd.py
```

The available keys will print showing their key mappings.
Customize the safety parameters inside `tuvtestd.py` before running it.