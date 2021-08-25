#!/usr/bin/env python3
import cereal.messaging as messaging
from common.realtime import Ratekeeper
from .detect import Detector
import numpy as np
import pygame  # pylint: disable=import-error
import cv2
import os

_BB_OFFSET = 0, 0
_BB_TO_FULL_FRAME = np.asarray([[1., 0., _BB_OFFSET[0]], [0., 1., _BB_OFFSET[1]],
                                [0., 0., 1.]])
_FULL_FRAME_SIZE = 1164, 874
SCALE = float(os.getenv("SCALE", "1"))

_DEBUG = True


def _debug(msg):
  if not _DEBUG:
    return
  print(msg)


def pygame_modules_have_loaded():
  return pygame.display.get_init() and pygame.font.get_init()


class SingsD():
  def __init__(self):
    # Init frame image holders
    self.img = np.zeros((_FULL_FRAME_SIZE[1], _FULL_FRAME_SIZE[0], 3), dtype='uint8')
    self.imgbgr = np.zeros((_FULL_FRAME_SIZE[1], _FULL_FRAME_SIZE[0], 3), dtype='uint8')
    self.imgff = np.zeros((_FULL_FRAME_SIZE[1], _FULL_FRAME_SIZE[0], 3), dtype=np.uint8)

    # Init frame viewer
    pygame.init()
    pygame.font.init()
    assert pygame_modules_have_loaded()

    size = (int(_FULL_FRAME_SIZE[0] * SCALE), int(_FULL_FRAME_SIZE[1] * SCALE))
    pygame.display.set_caption("comma one debug UI")
    self.screen = pygame.display.set_mode(size, pygame.DOUBLEBUF)

    self.camera_surface = pygame.surface.Surface((_FULL_FRAME_SIZE[0] * SCALE, _FULL_FRAME_SIZE[1] * SCALE),
                                                 0, 24).convert()

    # Detector
    self.detector = Detector(debug=_DEBUG)

  def udpate_frame(self, sm):
    sock = 'roadCameraState'
    if not sm.updated[sock] or not sm.valid[sock]:
      return

    log = sm[sock]

    # ***** frame *****
    yuv_img = log.image

    if log.transform:
      yuv_transform = np.array(log.transform).reshape(3, 3)
    else:
      # assume frame is flipped
      yuv_transform = np.array([[-1.0, 0.0, _FULL_FRAME_SIZE[0] - 1],
                                [0.0, -1.0, _FULL_FRAME_SIZE[1] - 1], [0.0, 0.0, 1.0]])

    if yuv_img and len(yuv_img) == _FULL_FRAME_SIZE[0] * _FULL_FRAME_SIZE[1] * 3 // 2:
      _debug('SignsD: Converting YUV image to RGV')
      yuv_np = np.frombuffer(
        yuv_img, dtype=np.uint8).reshape(_FULL_FRAME_SIZE[1] * 3 // 2, -1)
      cv2.cvtColor(yuv_np, cv2.COLOR_YUV2RGB_I420, dst=self.imgff)
      cv2.warpAffine(
        self.imgff,
        np.dot(yuv_transform, _BB_TO_FULL_FRAME)[:2], (self.img.shape[1], self.img.shape[0]),
        dst=self.img,
        flags=cv2.WARP_INVERSE_MAP)
    else:
      # actually RGB
      self.imgbgr = np.frombuffer(yuv_img, dtype=np.uint8).reshape((_FULL_FRAME_SIZE[1], _FULL_FRAME_SIZE[0], 3))
      self.img = self.imgbgr[:, :, ::-1]  # Convert BGR to RGB

    height, width = self.img.shape[:2]
    img_resized = cv2.resize(
      self.img, (int(SCALE * width), int(SCALE * height)), interpolation=cv2.INTER_CUBIC)

    # *** blits ***
    pygame.surfarray.blit_array(self.camera_surface, img_resized.swapaxes(0, 1))
    self.screen.blit(self.camera_surface, (0, 0))

    # this takes time...vsync or something
    pygame.display.flip()

  def detect(self):
    self.detector.run(self.imgbgr)


def signd_thread(sm=None, pm=None):
  signd = SingsD()
  rk = Ratekeeper(2., print_delay_threshold=None)  # Keeps rate at 2 hz

  # *** setup messaging
  if sm is None:
    sm = messaging.SubMaster(['roadCameraState'])

  while True:
    sm.update()
    signd.udpate_frame(sm)
    signd.detect()
    rk.keep_time()

def main(sm=None, pm=None):
  signd_thread(sm, pm)


if __name__ == "__main__":
  main()
