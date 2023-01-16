1. Connect your laptop to your comma device's hotspot and open a new SSH shell. Since service is being run on your laptop, we need to write a parameter to let controlsd know to start in failState debug mode:
   ```shell
   # on your comma device
   echo -n "1" > /data/params/d/FailStateMode
   ```

2. Add safety parameters override
   ```shell
   # on your comma device
   echo -n "1.5,-3.5,1500,350,15,25,10,25,1,0,0" > /data/params/d/TuvTestParams
   ```
   Values are comma separated for:
      ACCEL_MAX (Float)
      ACCEL_MIN (Float)
      STEER_MAX (Int)
      STEER_ERROR_MAX (Int)
      STEER_DELTA_UP_torque (Int)
      STEER_DELTA_DOWN_torque (Int)
      STEER_DELTA_UP (Int)
      STEER_DELTA_DOWN (Int)
      torqueRateLimiting (Bool)
      printAcc (Bool)
      printSteer (Bool)