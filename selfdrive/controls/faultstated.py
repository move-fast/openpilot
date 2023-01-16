from enum import IntEnum

class FAULTSTATE(IntEnum):
  NONE = 0 #  No fault
  MAX_BREAK = 1 #  Max deceleration
  MAX_ACC = 2 #  Max acceleration
  ZERO_ACC = 3 # no acceleration
  MAX_LINKS = 4 #  max steering left
  MAX_RECHTS = 5 #  max steering right
  NO_LAT = 6 #  no steering

  def acc_value(self):  # Value in m/s2. According to dbc file valid range is -20 to 20
    match self:
      case FAULTSTATE.MAX_BREAK:
        return -4.0
      case FAULTSTATE.MAX_ACC:
        return 4.0
      case _:
        return 0
  
  def lat_value(self):  # Lat is a normalized value. Max value defined @ CarControllerParams.STEER_MAX
    match self:
      case FAULTSTATE.MAX_LINKS:
        return -1.0
      case FAULTSTATE.MAX_RECHTS:
        return 1.0
      case _:
        return 0
