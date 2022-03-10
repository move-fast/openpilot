#!/usr/bin/env python3
from tqdm import tqdm
import csv

from selfdrive.boardd.boardd import can_capnp_to_can_list
from tools.lib.logreader import LogReader


if __name__ == "__main__":
  print("Loading log...")
  header = ['Address', 'Bus Time', 'Data', 'Source Port']
  ROUTE = "52a8edf5ce93e696_2021-05-27--08-43-16"
  # REPLAY_SEGS = list(range(10, 16))  # route has 82 segments available
  DIR = "/data/iav/"
  REPLAY_SEGS = [0]
  CAN_MSGS = []
  for i in tqdm(REPLAY_SEGS):
    log_url = f'{DIR}{ROUTE}--{i}--rlog.bz2'
    csv_rul = f'{DIR}{ROUTE}-{i}.csv'
    lr = LogReader(log_url)
    CAN_MSGS += [can_capnp_to_can_list(m.can) for m in lr if m.which() == 'can']

    with open(csv_rul, 'w', encoding='UTF8') as f:
      writer = csv.writer(f)
      # write the header
      writer.writerow(header)
      for can_msg in CAN_MSGS:
        # write the data
        writer.writerow(can_msg)
