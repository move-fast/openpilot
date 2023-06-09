#!/usr/bin/env python
from common.params import Params
from selfdrive.car.toyota.tuvparams import TuvParams


def tuvtest_config():
  tuvParams = TuvParams()
  # configure Tuv Params as necessary here:  

  # tuvParams.ACCEL_MAX = 1.5  # m/s2, lower than allowed 2.0 m/s2 for tuning reasons
  # tuvParams.ACCEL_MIN = -3.5  # m/s2
  # tuvParams.STEER_MAX = 1500
  # tuvParams.STEER_ERROR_MAX = 350     # max delta between torque cmd and torque motor
  # tuvParams.STEER_DELTA_UP_torque = 15       # 1.0s time to peak torque
  # tuvParams.STEER_DELTA_DOWN_torque = 25     # always lower than 45 otherwise the Rav4 faults (Prius seems ok with 50)
  # tuvParams.STEER_DELTA_UP = 10       # 1.5s time to peak torque
  # tuvParams.STEER_DELTA_DOWN = 25     # always lower than 45 otherwise the Rav4 faults (Prius seems ok with 50)
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
