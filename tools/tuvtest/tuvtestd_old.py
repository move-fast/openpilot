#!/usr/bin/env python
import os
import threading

import cereal.messaging as messaging
from common.realtime import Ratekeeper
from common.params import Params
from selfdrive.car.toyota.tuvparams import TuvParams
from tools.lib.kbhit import KBHit


class Keyboard:
  def __init__(self):
    self.kb = KBHit()
    self.validFaultStateKeys = [f'{e.value}' for e in FAULTSTATE]
    self.faultState = FAULTSTATE.NONE

  def update(self):
    key = self.kb.getch().lower()
    if key == 'c':
      self.faultState = FAULTSTATE.NONE
    elif key in self.validFaultStateKeys:
      self.faultState = FAULTSTATE(int(key))
    else:
      return False
    return True


def send_thread(keyboard):
  tuvtest_sock = messaging.pub_sock('externalControl')
  rk = Ratekeeper(100, print_delay_threshold=None)
  while 1:
    dat = messaging.new_message('externalControl')
    dat.externalControl.state = keyboard.faultState.value ******
    tuvtest_sock.send(dat.to_bytes())
    print(f'\n External Control: {keyboard.faultState.name}')
    rk.keep_time()

def tuvtest_thread():
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

  keyboard = Keyboard()

  threading.Thread(target=send_thread, args=(keyboard,), daemon=True).start()
  while True:
    keyboard.update()

if __name__ == '__main__':
  if not Params().get_bool("IsOffroad") and "ZMQ" not in os.environ and "WEB" not in os.environ:
    print("The car must be off before running tuvtestd.")
    exit()

  print()
  print('Control Keys:')
  print('\n'.join(f'Key = {e.value} -> {e.name}' for e in FAULTSTATE))

  tuvtest_thread()
