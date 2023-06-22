#!/usr/bin/env python
import os
import argparse
import threading
import pygame

import cereal.messaging as messaging
from common.realtime import Ratekeeper
from common.params import Params


class Joystick:
  def __init__(self):
    pygame.display.init()
    pygame.joystick.init()
    joystick_count = pygame.joystick.get_count()
    if joystick_count < 1:
      print('ERROR: no joystick found')
      exit()
    self.joystick = pygame.joystick.Joystick(0)
    name = self.joystick.get_name()
    print('using joystick: {}'.format(name))
    self.joystick.init()
    self.axes_values = {"ACC": 0., "STR": 0.}
    self.cancel = False

  def update(self):
    for event in pygame.event.get():
      if event.type == pygame.QUIT:
        self.cancel = True
      elif event.type == pygame.JOYBUTTONDOWN:
        print("Joystick button {} pressed.".format(event.button))

    steering_torque_cmd = self.joystick.get_axis(0) # -1.0 is left, 1.0 is right
    acceleration_cmd = self.joystick.get_axis(1) # -1.0 is left, 1.0 is right
    torque_factor = 3.0 # Must be equal or larger than the maximum input accepted by the car!
    acceleration_factor = 5.0 # Must be equal or larger than the maximum input accepted by the car!
    self.axes_values["ACC"] = -acceleration_cmd * acceleration_factor if abs(-acceleration_cmd * acceleration_factor) > 0.05 else 0.  # center can be noisy, deadzone of 5%
    self.axes_values["STR"] = -steering_torque_cmd * torque_factor if abs(-steering_torque_cmd * torque_factor) > 0.05 else 0.  # center can be noisy, deadzone of 5%

    return True


def send_thread(joystick):
  joystick_sock = messaging.pub_sock('externalControl')
  rk = Ratekeeper(100, print_delay_threshold=None)
  while 1:
    dat = messaging.new_message('externalControl')
    dat.externalControl.acceleration = joystick.axes_values["ACC"]
    dat.externalControl.steeringTorque = joystick.axes_values["STR"]
    dat.externalControl.valid = not joystick.cancel
    joystick_sock.send(dat.to_bytes())
    print('\n' + ', '.join(f'{name}: {round(v, 3)}' for name, v in joystick.axes_values.items()))
    rk.keep_time()


def joystick_thread(joystick):
  Params().put_bool('ExternalControlMode', True)
  threading.Thread(target=send_thread, args=(joystick,), daemon=True).start()
  while not joystick.cancel:
    joystick.update()


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Publishes events from your joystick to control your car.\n' +
                                               'openpilot must be offroad before starting tuvjoystickd.',
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  args = parser.parse_args()

  if not Params().get_bool("IsOffroad") and "ZMQ" not in os.environ:
    print("The car must be off before running joystickd.")
    exit()

  print()
  print('Using joystick, make sure to run cereal/messaging/bridge on your device if running over the network!')

  joystick = Joystick()
  joystick_thread(joystick)
