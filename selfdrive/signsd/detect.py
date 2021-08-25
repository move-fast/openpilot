from re import L
import sys
import os
import time
from pathlib import Path
from common.basedir import BASEDIR

import numpy as np
import torch

FILE = Path(__file__).absolute()
sys.path.append(os.path.join(BASEDIR, "yolov5"))

from yolov5.models.experimental import attempt_load
from yolov5.utils.augmentations import letterbox
from yolov5.utils.general import check_img_size, non_max_suppression, scale_coords
from yolov5.utils.torch_utils import select_device, time_sync


class Detector():
  def __init__(self, debug=False):
    self.debug = debug
    self.classes = None  # filter by class: --class 0, or --class 0 2 3
    self.conf_thres = 0.25  # confidence threshold
    self.iou_thres = 0.4  # NMS IOU threshold
    self.agnostic_nms = False  # class-agnostic NMS
    self.max_det = 1000  # maximum detections per image
    self.augment = False  # augmented inference
    self.imgsz = [512, 512]  # inference size (pixels)
    self.save_conf = False  # save confidences in --save-txt labels

    self.half = False  # use FP16 half-precision inference
    self.device = select_device()
    self.half &= self.device.type != 'cpu'  # half precision only supported on CUDA

    # Load model
    w = FILE.parents[0].as_posix() + '/best.pt'  # model.pt path(s)

    self.model = attempt_load(w, map_location=self.device)  # load FP32 model
    self.stride = int(self.model.stride.max())  # model stride
    self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names  # get class names
    if self.half:
      self.model.half()  # to FP16

    self.imgsz = check_img_size(self.imgsz, s=self.stride)  # check image size

    if self.device.type != 'cpu':
      self.model(torch.zeros(1, 3, *self.imgsz).to(self.device).type_as(next(self.model.parameters())))  # run once

  def _debug(self, msg):
    if not self.debug:
      return
    print(msg)

  @torch.no_grad()
  def run(self, img0):  # img0 = cv2.imread(path)  # BGR
      # Run inference
      # Padded resize
      img = letterbox(img0, self.imgsz, self.stride, True)[0]
      # Convert
      img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
      img = np.ascontiguousarray(img)

      t0 = time.time()
      img = torch.from_numpy(img).to(self.device)
      img = img.half() if self.half else img.float()  # uint8 to fp16/32

      img = img / 255.0  # 0 - 255 to 0.0 - 1.0
      if len(img.shape) == 3:
        img = img[None]  # expand for batch dim

      # Inference
      t1 = time_sync()
      pred = self.model(img, augment=self.augment, visualize=False)[0]

      # NMS
      pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms,
                                 max_det=self.max_det)
      t2 = time_sync()

      # Process predictions
      for i, det in enumerate(pred):  # detections per image
        s = '%gx%g' % img.shape[2:]  # print string
        s += f' original: {img0.shape}'
        gn = torch.tensor(img0.shape)[[1, 0, 1, 0]]  # normalization gain whwh

        if len(det):
          # Rescale boxes from img_size to im0 size
          det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img0.shape).round()

          # Print results
          for d in det:
            c = d[-1]
            s += f'\n{self.names[int(c)]}, xyxy: {int(d[0])}, {int(d[1])}, {int(d[2])}, {int(d[3])}, conf: {d[4]:2f}'
          # for c in det[:, -1].unique():
          #   n = (det[:, -1] == c).sum()  # detections per class
            # s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string

        # Print time (inference + NMS)
        self._debug('\n#######################')
        self._debug(f'{s}\nDone. ({t2 - t1:.3f}s)')
        self._debug('#######################\n')

      print(f'Done. ({time.time() - t0:.3f}s)')
