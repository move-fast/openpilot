import numpy as np
import math
from enum import Enum
from common.numpy_fast import interp
from common.params import Params
from common.realtime import sec_since_boot
from selfdrive.config import Conversions as CV


_LON_MPC_STEP = 0.2  # Time stemp of longitudinal control (5 Hz)
_MIN_V = 5.6  # Do not operate under 20km/h

_ENTERING_CURVATURE_TH = 0.003  # Curvature threshold to trigger entering turn state.
_ENTERING_LAT_ACC_TH = 1.0  # Lat Acc threshold to trigger entering turn state.
_ABORT_ENTERING_CURVATURE_TH = 0.002  # Curvature threshold to abourt entering state if road straightens.

_TURNING_CURVATURE_TH = 0.0045  # Curvature threshold to trigger turning turn state.
_LEAVING_CURVATURE_TH = 0.0025  # Curvature threshold to trigger leaving turn state.
_FINISH_CURVATURE_TH = 0.002  # Curvature threshold to trigger the end of turn cycle.

_ENTERING_SMOOTH_DECEL = -0.2  # Smooth decel when entering curve without overshooting lat acc limits.
_TURNING_SMOOTH_DECEL = -0.2  # Smooth decel when turning.
_LEAVING_ACC = 0.0  # Allowed acceleration when leaving the turn.

_EVAL_STEP = 5.  # evaluate curvature every 5mts
_EVAL_START = 0.  # start evaluating 0 mts ahead
_EVAL_LENGHT = 195.  # evaluate curvature for 130mts
_EVAL_RANGE = np.arange(_EVAL_START, _EVAL_LENGHT, _EVAL_STEP)

# Lookup table for maximum lateral acceleration according
# to R079r4e regulation for M1 category vehicles.
_A_LAT_REG_MAX_V = [4., 4., 4., 4.]  # Currently all the same for all speed ranges
_A_LAT_REG_MAX_BP = [2.8, 16.7, 27.8, 36.1]  # 10, 60, 100, 130 km/h


def eval_curvature(poly, x_vals):
  """
  This function returns a vector with the curvature based on path defined by `poly`
  evaluated on distance vector `x_vals`
  """
  # https://en.wikipedia.org/wiki/Curvature#  Local_expressions
  def curvature(x):
    a = abs(2 * poly[1] + 6 * poly[0] * x) / (1 + (3 * poly[0] * x**2 + 2 * poly[1] * x + poly[2])**2)**(1.5)
    return a

  return np.vectorize(curvature)(x_vals)


def eval_lat_acc(v_ego, x_curv):
  """
  This function returns a vector with the lateral acceleration based
  for the provided speed `v_ego` evaluated over curvature vector `x_curv`
  """

  def lat_acc(curv):
    a = v_ego**2 * curv
    return a

  return np.vectorize(lat_acc)(x_curv)


class TurnState(Enum):
  DISABLED = 1
  ENTERING = 2
  TURNING = 3
  LEAVING = 4

  @property
  def description(self):
    if self == TurnState.DISABLED:
      return 'DISABLED'
    if self == TurnState.ENTERING:
      return 'ENTERING'
    if self == TurnState.TURNING:
      return 'TURNING'
    if self == TurnState.LEAVING:
      return 'LEAVING'


class TurnController():
  def __init__(self, CP):
    self._params = Params()
    self._CP = CP
    self._op_enabled = False
    self._min_braking_acc = float(self._params.get("MaxDecelerationForTurns", True))
    self._last_params_update = 0.0
    self._v_cruise_setpoint = 0.0
    self._v_ego = 0.0
    self._state = TurnState.DISABLED

    self._reset()

  @property
  def v_turn_future(self):
    return float(self._v_turn_future) if self.state != TurnState.DISABLED else self._v_cruise_setpoint

  @property
  def state(self):
    return self._state

  @state.setter
  def state(self, value):
    if value != self._state:
      print(f'TurnController state: {value.description}')
      if value == TurnState.DISABLED:
        self._reset()
    self._state = value

  def _update_params(self):
    time = sec_since_boot()
    if time > self._last_params_update + 10.0:
      self._min_braking_acc = float(self._params.get("MaxDecelerationForTurns"))
      self._last_params_update = time
      print(f'Updated Max Decel: {self.min_braking_acc:.2f}')

  def _reset(self):
    self._v_turn_future = 0.0
    self._current_curvature = 0.0
    self._d_poly = [0., 0., 0., 0.]
    self._max_pred_curvature = 0.0
    self._max_pred_lat_acc = 0.0
    self._distance_to_max_curv = 200.0
    self._v_target_distance = 200.0
    self._v_target = 0.0
    self._lat_acc_overshoot_ahead = False

    self.a_turn = 0.0
    self.v_turn = 0.0

  def _update_calculations(self):
    pred_curvatures = eval_curvature(self._d_poly, _EVAL_RANGE)
    max_pred_curvature_idx = np.argmax(pred_curvatures)
    self._distance_to_max_curv = max(max_pred_curvature_idx * _EVAL_STEP + _EVAL_START, _EVAL_STEP)
    self._max_pred_curvature = pred_curvatures[max_pred_curvature_idx]
    self._max_pred_lat_acc = self._v_ego**2 * self._max_pred_curvature

    a_lat_reg_max = interp(self._v_ego, _A_LAT_REG_MAX_BP, _A_LAT_REG_MAX_V)
    max_curvature_for_vego = a_lat_reg_max / max(self._v_ego, 0.1)
    lat_acc_overshoot_idxs = np.nonzero(pred_curvatures >= max_curvature_for_vego)[0]
    self._lat_acc_overshoot_ahead = len(lat_acc_overshoot_idxs) > 0

    if self._lat_acc_overshoot_ahead:
      self._v_target_distance = max(lat_acc_overshoot_idxs[0] * _EVAL_STEP + _EVAL_START, _EVAL_STEP)
      self._v_target = min(math.sqrt(a_lat_reg_max / self._max_pred_curvature), self._v_cruise_setpoint)

  def _state_transition(self):
    # In any case, if system is disabled or min braking param has been set to non negative value, disable.
    if not self._op_enabled or self._min_braking_acc >= 0.0:
      self.state = TurnState.DISABLED
      return

    # DISABLED
    if self.state == TurnState.DISABLED:
      # Do not enter a turn control cycle if speed is low.
      if self._v_ego <= _MIN_V:
        pass
      # If substantial curvature ahead is detected, and a minimum lateral
      # acceleration is predicted, then move to Entering turn state.
      elif self._max_pred_curvature >= _ENTERING_CURVATURE_TH and self._max_pred_lat_acc >= _ENTERING_LAT_ACC_TH:
        self.state = TurnState.ENTERING
    # ENTERING
    elif self.state == TurnState.ENTERING:
      # Transition to Turning if current curvature over threshold.
      if self._current_curvature >= _TURNING_CURVATURE_TH:
        self.state = TurnState.TURNING
      # Abort if road straightens.
      elif self._max_pred_curvature < _ABORT_ENTERING_CURVATURE_TH:
        self.state = TurnState.DISABLED
    # TURNING
    elif self.state == TurnState.TURNING:
      # Transition to Leaving if current curvature under threshold.
      if self._current_curvature < _LEAVING_CURVATURE_TH:
        self.state = TurnState.LEAVING
    # LEAVING
    elif self.state == TurnState.LEAVING:
      # Transition back to Turning if current curvature over threshold.
      if self._current_curvature >= _TURNING_CURVATURE_TH:
        self.state = TurnState.TURNING
      elif self._current_curvature < _FINISH_CURVATURE_TH:
        self.state = TurnState.DISABLED

  def _update_solution(self):
    a_target = self._a_ego
    if self.state == TurnState.DISABLED:
      pass
    elif self.state == TurnState.ENTERING:
      if self._lat_acc_overshoot_ahead:
        a_target = (self._v_target**2 - self._v_ego**2) / (2 * self._v_target_distance)
      else:
        a_target = _ENTERING_SMOOTH_DECEL
    elif self.state == TurnState.TURNING:
      a_target = _TURNING_SMOOTH_DECEL
    elif self.state == TurnState.LEAVING:
      a_target = _LEAVING_ACC

    self.a_turn = max(a_target, self._min_braking_acc)
    self.v_turn = self._v_ego + self.a_turn * _LON_MPC_STEP  # speed in next Longitudinal control step.
    self._v_turn_future = self._v_ego + self.a_turn * 4.  # speed in 4 seconds.

  def update(self, enabled, v_ego, a_ego, v_cruise_setpoint, d_poly, steering_angle):
    self._op_enabled = enabled
    self._v_ego = v_ego
    self._a_ego = a_ego
    self._v_cruise_setpoint = v_cruise_setpoint
    self._d_poly = d_poly
    self._current_curvature = abs(steering_angle * CV.DEG_TO_RAD / (self._CP.steerRatio * self._CP.wheelbase))

    self._update_params()
    self._update_calculations()
    self._state_transition()
    self._update_solution()
