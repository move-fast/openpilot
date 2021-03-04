from math import sin, cos, sqrt, atan2, radians, degrees, pi
from enum import Enum


R = 6373.0  # approximate radius of earth in km
CURVATURE_OFFSET = .3  # 300 mts. The distance offset for curvature calculation
MAX_DIST_FOR_CURVATURE = 0.5  # Max distance between nodes for curvature calculation


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


def xy(refPoint, point):
  pointArad = coordToRad(refPoint)
  pointBrad = coordToRad(point)
  return xy_rad(pointArad, pointBrad)


def x_y_bearing_rad(pointArad, pointBrad):
  (latA, lonA) = pointArad
  (latB, lonB) = pointBrad

  dlon = lonB - lonA

  x = sin(dlon) * cos(latB)
  y = cos(latA) * sin(latB) - (sin(latA) * cos(latB) * cos(dlon))
  bearing = degrees(atan2(x, y))
  return x * R, y * R, (bearing + 360) % 360


def bearing_rad(pointArad, pointBrad):
  _, _, bearing = x_y_bearing_rad(pointArad, pointBrad)
  return (bearing + 360) % 360


def xy_rad(pointArad, pointBrad):
  x, y, _ = x_y_bearing_rad(pointArad, pointBrad)
  return x, y


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


def three_point_curvature_alt(ref, prev, next):
  # https://math.stackexchange.com/questions/2507540/numerical-way-to-solve-for-the-curvature-of-a-curve
  # https://en.wikipedia.org/wiki/Heron%27s_formula
  prev_r = (prev[0] - ref[0], prev[1] - ref[1])
  next_r = (next[0] - ref[0], next[1] - ref[1])

  prev_ang = atan2(prev_r[0], prev_r[1])
  next_ang = atan2(next_r[0], next_r[1])
  a = CURVATURE_OFFSET
  b = CURVATURE_OFFSET

  prev_n = (a * cos(prev_ang), a * sin(prev_ang))
  next_n = (b * cos(next_ang), b * sin(next_ang))

  c = xy_distance(next_n, prev_n)
  s = (a + b + c) / 2.
  A = sqrt(s * (s - a) * (s - b) * (s - c))

  return 4 * A / (a * b * c)


def three_point_tangent_angle(ref, prev, next):
  # https://www.math24.net/curvature-radius
  prev_r = (prev[0] - ref[0], prev[1] - ref[1])
  next_r = (next[0] - ref[0], next[1] - ref[1])

  prev_ang = atan2(prev_r[0], prev_r[1]) - pi
  next_ang = atan2(next_r[0], next_r[1])
  return (prev_ang + next_ang) / 2.


def three_point_curvature(ref_xy, prev_xy, next_xy, ref_tan, prev_tan, next_tan):
  prev_tan_delta = ref_tan - prev_tan if prev_tan is not None else 0
  prev_dist = max(xy_distance(prev_xy, ref_xy), MAX_DIST_FOR_CURVATURE)
  next_tan_delta = next_tan - ref_tan if next_tan is not None else 0
  next_dist = max(xy_distance(next_xy, ref_xy), MAX_DIST_FOR_CURVATURE)

  prev_curv = prev_tan_delta / prev_dist
  next_curv = next_tan_delta / next_dist

  return (prev_curv + next_curv) / 2.


def xy_distance(A, B):
  return sqrt((A[0] - B[0])**2 + (A[1] - B[1])**2)


class DIRECTION(Enum):
  NONE = 0
  AHEAD = 1
  BEHIND = 2
  FORWARD = 3
  BACKWARD = 4
