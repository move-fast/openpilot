#!/usr/bin/env python
from common.params import Params
from selfdrive.car.tuvparams import TuvParams


def tuvtest_config():
  tuvParams = TuvParams()
  # configure Tuv Params as necessary here:  

  # tuvParams.ACCEL_MAX = 1.5  # m/s2, lower than allowed 2.0 m/s2 for tuning reasons
  # tuvParams.ACCEL_MIN = -3.5  # m/s2
  # tuvParams.STEER_MAX = 1500
  # tuvParams.torqueRateLimiting = True
  # tuvParams.printAcc = False
  # tuvParams.printSteer = False
  
  print()
  print('TUV Params:')
  print(tuvParams.description())

  Params().put_bool('ExternalControlMode', True)
  tuvParams.write()

if __name__ == '__main__':
  tuvtest_config()
