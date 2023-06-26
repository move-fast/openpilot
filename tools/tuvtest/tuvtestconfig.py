#!/usr/bin/env python
from common.params import Params
from selfdrive.car.tuvparams import TuvParams


def tuvtest_config():
  tuvParams = TuvParams()
  # configure Tuv Params as necessary here:  
  # Defaults to Toyota values with torque rate limiting and acceleration clipping.

  # LIMITS --------------
  # tuvParams.ACCEL_MAX = 1.5  # m/s2,  TOYOTA DEFAULT: 1.5,      VW DEFAULT: 2.0
  # tuvParams.ACCEL_MIN = -3.5  # m/s2, TOYOTA DEFAULT: -3.5,     VW DEFAULT: -3.5
  # tuvParams.STEER_MAX = 1500  #       TOYOTA DEFAULT: 1500,     VW DEFAULT: 300

  # GENERAL SETTINGS ---------------
  # tuvParams.torqueRateLimiting = True
  # tuvParams.accelerationClipping = True
  # tuvParams.printAcc = False
  # tuvParams.printSteer = False
  
  print()
  print('TUV Params:')
  print(tuvParams.description())

  Params().put_bool('ExternalControlMode', True)
  tuvParams.write()

if __name__ == '__main__':
  tuvtest_config()
