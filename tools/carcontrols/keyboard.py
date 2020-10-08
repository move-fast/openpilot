#!/usr/bin/env python

# This process publishes keyboard events as joystick values. Such events can be suscribed by
# mocked car controller scripts.

import cereal.messaging as messaging
from common.numpy_fast import clip
from common.realtime import Ratekeeper
import sys
import termios
from termios import (BRKINT, CS8, CSIZE, ECHO, ICANON, ICRNL, IEXTEN, INPCK,
                     ISIG, ISTRIP, IXON, PARENB, VMIN, VTIME)


# Indexes for termios list.
IFLAG = 0
OFLAG = 1
CFLAG = 2
LFLAG = 3
ISPEED = 4
OSPEED = 5
CC = 6


def getch():
  fd = sys.stdin.fileno()
  old_settings = termios.tcgetattr(fd)
  try:
    # set
    mode = termios.tcgetattr(fd)
    mode[IFLAG] = mode[IFLAG] & ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON)
    # mode[OFLAG] = mode[OFLAG] & ~(OPOST)
    mode[CFLAG] = mode[CFLAG] & ~(CSIZE | PARENB)
    mode[CFLAG] = mode[CFLAG] | CS8
    mode[LFLAG] = mode[LFLAG] & ~(ECHO | ICANON | IEXTEN | ISIG)
    mode[CC][VMIN] = 1
    mode[CC][VTIME] = 0
    termios.tcsetattr(fd, termios.TCSAFLUSH, mode)

    ch = sys.stdin.read(1)
  finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
  return ch


_STEER_STEP = 0.1
_ACC_STEP = 0.1


def keyboard_thread():
  joystick_sock = messaging.pub_sock('testJoystick')

  rk = Ratekeeper(100, print_delay_threshold=None)

  enabled = False
  steer = 0.  # -1 to 1
  gas_brake = 0.  # -1 to 1

  # -------- Main Program Loop -----------
  while True:
    # Mimic axes and buttons of joystick based on key presses
    pcm_cancel_cmd = False
    hud_alert = False
    terminate = False

    c = getch()
    print("got %s" % c)
    if c == 'e':  # enable
      enabled = True
    if c == 'x':  # disable
      enabled = False
    if c == 'c':  # cancel
      pcm_cancel_cmd = True
    if c == 'h':  # hud_alert
      hud_alert = True
    if c == 'a':  # steer left
      steer -= _STEER_STEP
    if c == 'd':  # steer right
      steer += _STEER_STEP
    if c == 'w':  # gas
      gas_brake += _ACC_STEP
    if c == 's':  # break
      gas_brake -= _ACC_STEP
    if c == 'q':
      terminate = True
      pcm_cancel_cmd = True
      enabled = False

    steer = clip(steer, -1., 1.)
    gas_brake = clip(gas_brake, -1., 1.)

    dat = messaging.new_message('testJoystick')
    axes = [0., gas_brake, 0., steer]
    dat.testJoystick.axes = axes
    buttons = [pcm_cancel_cmd, enabled, False, hud_alert]
    dat.testJoystick.buttons = buttons
    joystick_sock.send(dat.to_bytes())

    if terminate:
      exit(0)

    # Limit to 100 frames per second
    rk.keep_time()


if __name__ == "__main__":
  keyboard_thread()
