import numpy as np
import math
from common.numpy_fast import interp
from common.filter_simple import FirstOrderFilter
from common.params import Params
from common.realtime import sec_since_boot
from selfdrive.config import Conversions as CV


_EVAL_STEP = 5.  # evaluate curvature every 5mts
_EVAL_START = 0.  # start evaluating 0 mts ahead
_EVAL_LENGHT = 130.  # evaluate curvature for 130mts
_EVAL_RANGE = np.arange(_EVAL_START, _EVAL_LENGHT, _EVAL_STEP)
# Offset to target lateral acceleration on turns to prevent multiple decelerations in a single turn
_TARGET_LAT_ACC_OFFSET = 1.0

# Curvature detection
_STEERING_ANGLE_FILTER_TS = .35
_CURRENT_CURVATURE_THOLD = 0.0025  # 400mt raidus. current curvature threshold to detect vehicle turning.

# Deceleration for turn filter
_DECEL_FOR_TURN_FILTER_TS = .35  # 0.45 Hz (1/2*Pi*f)
_DECEL_FOR_TURN_FILTER_ON_THOLD = 0.3
_DECEL_FOR_TURN_FILTER_OFF_THOLD = 0.1

# Lookup table for maximum lateral acceleration according
# to R079r4e regulation for M1 category vehicles.
_A_LAT_REG_MAX_V = [3., 3., 3., 3.]  # Currently all the same for all speed ranges
_A_LAT_REG_MAX_BP = [2.8, 16.7, 27.8, 36.1]  # 10, 60, 100, 130 km/h


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


class TurnSolver():
  def __init__(self, CP):
    self.params = Params()
    self.CP = CP
    self.decelFilter = FirstOrderFilter(0., _DECEL_FOR_TURN_FILTER_TS, CP.radarTimeStep)
    self.angleFilter = FirstOrderFilter(0., _STEERING_ANGLE_FILTER_TS, CP.radarTimeStep)
    self.a_turn = 0.0
    self.v_turn = 0.0
    self._v_turn_future = 0.0
    self.decelerate = False
    self.v_cruise_setpoint = 0.0
    self.min_braking_acc = float(self.params.get("MaxDecelerationForTurns", True))
    self.last_params_update = 0.0

  @property
  def v_turn_future(self):
    return float(self._v_turn_future) if self.decelerate else self.v_cruise_setpoint

  def update_params(self):
    time = sec_since_boot()
    if time > self.last_params_update + 10.0:
      self.min_braking_acc = float(self.params.get("MaxDecelerationForTurns"))
      self.last_params_update = time
      print(f'Updated Max Decel: {self.min_braking_acc:.2f}')

  def update(self, enabled, v_ego, v_cruise_setpoint, d_poly, steering_angle):
    self.v_cruise_setpoint = v_cruise_setpoint
    angle_filtered = self.angleFilter.update(steering_angle)
    self.update_params()

    if not enabled or self.min_braking_acc >= 0.0:
      self.decelerate = False
      return

    lat_accs = eval_lat_acc(d_poly, v_ego, _EVAL_RANGE)
    max_lat_acc_idx = np.argmax(lat_accs)
    distance_to_max_lat_acc = max(max_lat_acc_idx * _EVAL_STEP + _EVAL_START, 0.00001)
    max_lat_acc = lat_accs[max_lat_acc_idx]
    a_lat_reg_max = max(1.0, interp(v_ego, _A_LAT_REG_MAX_BP, _A_LAT_REG_MAX_V) - _TARGET_LAT_ACC_OFFSET)

    # detect if we will a any point in predicted path exceed max lat acc.
    decel_for_turn = max_lat_acc >= a_lat_reg_max

    # filter value and add hysteresis
    filter_value = self.decelFilter.update(float(decel_for_turn))
    decelerate = (not self.decelerate and filter_value >= _DECEL_FOR_TURN_FILTER_ON_THOLD) \
        or (self.decelerate and filter_value > _DECEL_FOR_TURN_FILTER_OFF_THOLD)

    if not decelerate:
      if not self.decelerate:
        # Provide no solution if not deceleration necessary and no change.
        return

      # Vehicle has reached a viable turn speed. Keep this solution as long as vehicle is turning.
      current_curvature = angle_filtered * CV.DEG_TO_RAD / (self.CP.steerRatio * self.CP.wheelbase)
      print(f'-> Current Curvature: {current_curvature:.4f}, isntant angle: {steering_angle:.2f}, filt: {angle_filtered:.2f}')
      if abs(current_curvature) <= _CURRENT_CURVATURE_THOLD:
        print('VVVVVV Leaving Curve, Stop decelerating')
        # quite straight, provide no solution.
        self.decelerate = False
        return
      # Still turning, keep providing solution to keep speed.
      print(f'^^^^^ Still on turn, keep speed: {v_ego:.2f}')
      self.a_turn = 0.0
      self.v_turn = v_ego
      self._v_turn_future = v_ego
      return

    self.decelerate = True
    if decel_for_turn:
      # As long as signal indicate we must decelerate, we update a_turn and calculate.
      # Otherwise, we need to keep descelerating while filtered value shifts. Use last value of a_turn
      max_curvature = max_lat_acc / max(v_ego**2, 0.000001)
      v_target = min(math.sqrt(a_lat_reg_max / max_curvature), v_cruise_setpoint)
      acc_limit = (v_target**2 - v_ego**2) / (2 * distance_to_max_lat_acc)
      self.a_turn = max(acc_limit, self.min_braking_acc)
      print(f'^^^^^ New Limit found a_turn: {self.a_turn:.2f}')
      print(f'-> Ahead lat acceleration ({max_lat_acc:.2f}) in {distance_to_max_lat_acc:.0f} mts.')
    else:
      print(f'^^^^^ Using old Limit a_turn: {self.a_turn:.2f}')

    self.v_turn = v_ego + self.a_turn * 0.2  # speed in 0.2 seconds
    self._v_turn_future = v_ego + self.a_turn * 4  # speed in 4 seconds.
