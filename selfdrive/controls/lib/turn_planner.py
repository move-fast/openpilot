import numpy as np
import math
from common.numpy_fast import interp
from common.filter_simple import FirstOrderFilter

from selfdrive.config import Conversions as CV

_EVAL_STEP = 5.  # evaluate curvature every 5mts
_EVAL_START = 15.  # start evaluating 15 mts ahead
_EVAL_LENGHT = 150.  # evaluate curvature for 150mts
_EVAL_RANGE = np.arange(_EVAL_START, _EVAL_LENGHT, _EVAL_STEP)
# Offset to target lateral acceleration on turns to prevent multiple decelerations in a single turn
_TARGET_LAT_ACC_OFFSET = 0.5

# Deceleration for turn filter
_DECEL_FOR_TURN_FILTER_TS = .45  # 0.35 Hz (1/2*Pi*f)
_DECEL_FOR_TURN_FILTER_ON_THOLD = 0.4
_DECEL_FOR_TURN_FILTER_OFF_THOLD = 0.2
_MIN_BRAKING_ACC = -3.0

# Lookup table for maximum lateral acceleration according
# to R079r4e regulation for M1 category vehicles.
_A_LAT_REG_MAX_V = [3., 3., 3., 3.]  # Currently all the same for all speed ranges
_A_LAT_REG_MAX_BP = [2.8, 16.7, 27.8, 36.1]  # 10, 60, 100, 130 km/h

# TODO: Find out appropiate value for deceleration in turns when going too fast.
TURN_SMOOTH_DECEL = -0.1  # when turning too fast, car smoothly decel at .1m/s^2.


def limit_accel_in_turns(v_ego, a_total_max, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the maximum lateral acceleration
  allowed according to R079r4e regulation base on evaluation of the immediate lateral acceleration.
  """
  a_lat_reg_max = interp(v_ego, _A_LAT_REG_MAX_BP, _A_LAT_REG_MAX_V)
  a_lat = v_ego**2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)

  if a_lat <= a_lat_reg_max:
    a_lon_allowed = math.sqrt(max(a_total_max**2 - a_lat**2, 0.))
  else:
    # Going too fast, smoothly decelerate.
    a_lon_allowed = TURN_SMOOTH_DECEL
    print(f'-> Immediate lat acceleration({a_lat: .2f}) too high. Setting top limit to: \
        {min(a_target[1], a_lon_allowed): .2f}')

  return [a_target[0], min(a_target[1], a_lon_allowed)]


def eval_lat_acc(poly, v_ego, x_vals):
  """
  This function returns a vector with the lateral acceleration based on path defined by `poly`
  for the provided speed `v_ego` evaluated on distance vector `x_vals`
  """

  # https://en.wikipedia.org/wiki/Curvature#  Local_expressions
  def lat_acc(x):
    a = v_ego**2 * abs(2 * poly[1] + 6 * poly[0] * x) / (1 + (3 * poly[0] * x**2 + 2 * poly[1] * x + poly[2])**2)**(1.5)
    return a

  return np.vectorize(lat_acc)(x_vals)


class TurnPlanner():
  def __init__(self, CP):
    self.CP = CP
    self.decel_for_turn_filter = FirstOrderFilter(0., _DECEL_FOR_TURN_FILTER_TS, CP.radarTimeStep)
    self.last_a_turn = 0.0
    self.decel_for_turn_filtered = False

  def limit_accel_for_turn_ahead(self, v_ego, v_cruise_setpoint, d_poly, limits):
    """
    This function returns a limited long acceleration allowed to limit the maximum lateral acceleration
    on the predicted path ahead to the limit specified by regulation.
    The limit will be enforced only if the calculated limit is getting close to the already provided
    minimum acceleration cruise limit.
    """
    lat_accs = eval_lat_acc(d_poly, v_ego, _EVAL_RANGE)
    max_lat_acc_idx = np.argmax(lat_accs)
    distance_to_max_lat_acc = max(max_lat_acc_idx * _EVAL_STEP + _EVAL_START, 0.00001)
    max_lat_acc = lat_accs[max_lat_acc_idx]
    a_lat_reg_max = interp(v_ego, _A_LAT_REG_MAX_BP, _A_LAT_REG_MAX_V)

    decel_for_turn = max_lat_acc >= a_lat_reg_max
    filter_value = self.decel_for_turn_filter.update(float(decel_for_turn))

    # Add hysteresis
    self.decel_for_turn_filtered = (not self.decel_for_turn_filtered and filter_value >= _DECEL_FOR_TURN_FILTER_ON_THOLD) \
        or (self.decel_for_turn_filtered and filter_value > _DECEL_FOR_TURN_FILTER_OFF_THOLD)

    if not self.decel_for_turn_filtered:
      return limits, False, v_ego, limits[1], v_cruise_setpoint

    if not decel_for_turn:
      # We need to keep limiting to descelerate while filter shifts.
      v_turn = v_ego + self.last_a_turn * 0.2  # speed in 0.2 seconds
      v_turn_future = v_ego + self.last_a_turn * 4  # speed in 4 seconds
      return [limits[0], self.last_a_turn], True, v_turn, self.last_a_turn, float(v_turn_future)

    max_curvature = max_lat_acc / max(v_ego**2, 0.000001)
    a_lat_target = max(1.0, a_lat_reg_max - _TARGET_LAT_ACC_OFFSET)  # Greater than 1.0 to prevent excesive deceleration
    v_target = min(math.sqrt(a_lat_target / max_curvature), v_cruise_setpoint)
    acc_limit = (v_target**2 - v_ego**2) / (2 * distance_to_max_lat_acc)

    a_turn = min(max(acc_limit, _MIN_BRAKING_ACC), limits[1])
    self.last_a_turn = a_turn
    v_turn = v_ego + a_turn * 0.2  # speed in 0.2 seconds
    v_turn_future = v_ego + a_turn * 4  # speed in 4 seconds

    print('-----------------------------')
    print(f'-> Ahead lat acceleration ({max_lat_acc:.2f}) in {distance_to_max_lat_acc:.0f} mts.')
  #   print(f'-> v_ego: {v_ego}, v_target: {v_target:.2f}')
  #   print(f'-> Provided acc limits: l: {limits[0]:.2f}  u: {limits[1]:.2f}')
  #   print(f'-> acc_limit: {acc_limit:.2f}, new_upper_limit: {max_lon_acc:.2f}')
    print(f'-> **** Ahead lat acceleration too high. Setting top limit to: {a_turn:.2f} ****')

    return [limits[0], min(max(acc_limit, limits[0]), limits[1])], True, v_turn, a_turn, float(v_turn_future)


class TurnSolver():
  def __init__(self, CP):
    self.CP = CP
    self.filter = FirstOrderFilter(0., _DECEL_FOR_TURN_FILTER_TS, CP.radarTimeStep)
    self.a_turn = 0.0
    self.v_turn = 0.0
    self._v_turn_future = 0.0
    self.decelerate = False
    self.v_cruise_setpoint = 0.0

  @property
  def v_turn_future(self):
    return float(self._v_turn_future) if self.decelerate else self.v_cruise_setpoint

  def update(self, enabled, v_ego, v_cruise_setpoint, d_poly):
    self.v_cruise_setpoint = v_cruise_setpoint

    if not enabled:
      self.decelerate = False
      return

    lat_accs = eval_lat_acc(d_poly, v_ego, _EVAL_RANGE)
    max_lat_acc_idx = np.argmax(lat_accs)
    distance_to_max_lat_acc = max(max_lat_acc_idx * _EVAL_STEP + _EVAL_START, 0.00001)
    max_lat_acc = lat_accs[max_lat_acc_idx]
    a_lat_reg_max = interp(v_ego, _A_LAT_REG_MAX_BP, _A_LAT_REG_MAX_V)

    decel_for_turn = max_lat_acc >= a_lat_reg_max
    filter_value = self.filter.update(float(decel_for_turn))

    # hysteresis
    self.decelerate = (not self.decelerate and filter_value >= _DECEL_FOR_TURN_FILTER_ON_THOLD) \
        or (self.decelerate and filter_value > _DECEL_FOR_TURN_FILTER_OFF_THOLD)

    if not self.decelerate:
      return

    if decel_for_turn:
      # As long as signal indicate we must decelerate, we update a_turn and calculate.
      # Otherwise, we need to keep descelerating while filtered value shifts. Use last value of a_turn
      max_curvature = max_lat_acc / max(v_ego**2, 0.000001)
      a_lat_target = max(1.0, a_lat_reg_max - _TARGET_LAT_ACC_OFFSET)  # Avoid setting too low
      v_target = min(math.sqrt(a_lat_target / max_curvature), v_cruise_setpoint)
      acc_limit = (v_target**2 - v_ego**2) / (2 * distance_to_max_lat_acc)
      self.a_turn = max(acc_limit, _MIN_BRAKING_ACC)

    self.v_turn = v_ego + self.a_turn * 0.2  # speed in 0.2 seconds
    self._v_turn_future = v_ego + self.a_turn * 4  # speed in 4 seconds

    print('-----------------------------')
    print(f'-> Ahead lat acceleration ({max_lat_acc:.2f}) in {distance_to_max_lat_acc:.0f} mts.')
