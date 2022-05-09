#!/usr/bin/env python3
import sys
import socket

from common.text_window import TextWindow
from selfdrive.manager.helpers import unblock_stdout


def get_ip():
  s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  s.settimeout(0)
  try:
    # doesn't even have to be reachable
    s.connect(('10.255.255.255', 1))
    IP = s.getsockname()[0]
  except Exception:
    IP = '127.0.0.1'
  finally:
    s.close()
  return IP

if __name__ == "__main__":
  unblock_stdout()

  with TextWindow(f"IP: {get_ip()}") as t:
    t.wait_for_exit()

  # manual exit because we are forked
  sys.exit(0)
