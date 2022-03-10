#!/usr/bin/env python3
from tqdm import tqdm

from selfdrive.boardd.boardd import can_capnp_to_can_list
from tools.lib.logreader import LogReader


if __name__ == "__main__":
  print("Loading log...")
  ROUTE = "52a8edf5ce93e696_2021-05-27--08-43-16"
  # REPLAY_SEGS = list(range(10, 16))  # route has 82 segments available
  REPLAY_SEGS = [0]
  CAN_MSGS = []
  for i in tqdm(REPLAY_SEGS):
    log_url = f'~/Downloads/{ROUTE}--{i}--rlog.bz2'
    lr = LogReader(log_url)
    CAN_MSGS += [can_capnp_to_can_list(m.can) for m in lr if m.which() == 'can']

  print(CAN_MSGS)
