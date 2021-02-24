from math import sin, cos, sqrt, atan2, radians, degrees
from enum import Enum


R = 6373.0  # approximate radius of earth in km


def distance_and_bearing(pointA, pointB):
  (latA, lonA) = tuple(map(lambda p: radians(p), pointA))
  (latB, lonB) = tuple(map(lambda p: radians(p), pointB))

  dlon = lonB - lonA
  dlat = latB - latA

  a = sin(dlat / 2)**2 + cos(latA) * cos(latB) * sin(dlon / 2)**2
  c = 2 * atan2(sqrt(a), sqrt(1 - a))

  x = sin(dlon) * cos(latB)
  y = cos(latA) * sin(latB) - (sin(latA) * cos(latB) * cos(dlon))
  bearing = degrees(atan2(x, y))
  compass_bearing = (bearing + 360) % 360

  return R * c, compass_bearing


def bearing_delta(bearingA, bearingB):
  return (bearingA - bearingB + 180) % 360 - 180


def absoule_delta_with_direction(bearing_delta):
  delta_ahead = abs(bearing_delta)
  delta_behind = abs(abs(bearing_delta) - 180)

  if delta_ahead < delta_behind:
    return (delta_ahead, DIRECTION.AHEAD)
  elif delta_ahead > delta_behind:
    return (delta_behind, DIRECTION.BEHIND)
  else:
    return (delta_ahead, DIRECTION.NONE)


class DIRECTION(Enum):
  NONE = 0
  AHEAD = 1
  BEHIND = 2
  FORWARD = 3
  BACKWARD = 4
