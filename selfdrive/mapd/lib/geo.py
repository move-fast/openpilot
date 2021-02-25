from math import sin, cos, sqrt, atan2, radians, degrees
from enum import Enum


R = 6373.0  # approximate radius of earth in km


def coordToRad(point):
  return tuple(map(lambda p: radians(p), point))


def distance(pointA, pointB):
  pointArad = coordToRad(pointA)
  pointBrad = coordToRad(pointB)
  return distance_rad(pointArad, pointBrad)


def distance_rad(pointArad, pointBrad):
  (latA, lonA) = pointArad
  (latB, lonB) = pointBrad

  dlon = lonB - lonA
  dlat = latB - latA

  a = sin(dlat / 2)**2 + cos(latA) * cos(latB) * sin(dlon / 2)**2
  c = 2 * atan2(sqrt(a), sqrt(1 - a))

  return R * c


def bearing(pointA, pointB):
  pointArad = coordToRad(pointA)
  pointBrad = coordToRad(pointB)
  return bearing_rad(pointArad, pointBrad)


def bearing_rad(pointArad, pointBrad):
  (latA, lonA) = pointArad
  (latB, lonB) = pointBrad

  dlon = lonB - lonA

  x = sin(dlon) * cos(latB)
  y = cos(latA) * sin(latB) - (sin(latA) * cos(latB) * cos(dlon))
  bearing = degrees(atan2(x, y))
  return (bearing + 360) % 360


def distance_and_bearing(pointA, pointB):
  pointArad = coordToRad(pointA)
  pointBrad = coordToRad(pointB)

  return distance_rad(pointArad, pointBrad), bearing_rad(pointArad, pointBrad)


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
