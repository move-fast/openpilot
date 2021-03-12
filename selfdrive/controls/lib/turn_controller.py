import numpy as np
import math
from enum import Enum
from common.numpy_fast import interp
from common.params import Params
from common.realtime import sec_since_boot
from selfdrive.config import Conversions as CV


_LON_MPC_STEP = 0.2  # Time stemp of longitudinal control (5 Hz)
_MIN_V = 5.6  # Do not operate under 20km/h

_ENTERING_PRED_CURVATURE_TH = 0.003  # Predicitve curvature threshold to trigger entering turn state.
_ENTERING_PRED_LAT_ACC_TH = 1.0  # Predicted Lat Acc threshold to trigger entering turn state.
_ABORT_ENTERING_CURVATURE_TH = 0.0015  # Curvature threshold to abort entering state if road straightens.

_TURNING_CURVATURE_TH = 0.003  # Curvature threshold to trigger turning turn state.
_LEAVING_CURVATURE_TH = 0.0025  # Curvature threshold to trigger leaving turn state.
_FINISH_CURVATURE_TH = 0.002  # Curvature threshold to trigger the end of turn cycle.

_ENTERING_SMOOTH_DECEL = -0.3  # Smooth decel when entering curve without overshooting lat acc limits.
_LEAVING_ACC = 0.0  # Allowed acceleration when leaving the turn.

_EVAL_STEP = 5.  # evaluate curvature every 5mts
_EVAL_START = 0.  # start evaluating 0 mts ahead
_EVAL_LENGHT = 195.  # evaluate curvature for 195mts
_EVAL_RANGE = np.arange(_EVAL_START, _EVAL_LENGHT, _EVAL_STEP)

_MAX_DISTANCE_HORIZON = 200.  # horizon for curvature data from map data.

_MAX_JERK_ACC_INCREASE = 0.5  # Maximum jerk allowed when increasing acceleration.

# Lookup table for maximum lateral acceleration according
# to R079r4e regulation for M1 category vehicles.
_A_LAT_REG_MAX_V = [2., 2., 2., 2.]  # Currently all the same for all speed ranges
_A_LAT_REG_MAX_BP = [2.8, 16.7, 27.8, 36.1]  # 10, 60, 100, 130 km/h

# Lookup table for the minimum deceleration during the ENTERING state
# depending on the actual maximum absolute lateral acceleration predicted on the turn ahead.
_ENTERING_SMOOTH_DECEL_V = [-0.3, -1.]  # min decel value allowed on ENTERING state
_ENTERING_SMOOTH_DECEL_BP = [1., 3]  # absolute value of lat acc ahead

# Lookup table for the acceleration for the TURNING state
# depending on the current lateral acceleration of the vehicle.
_TURNING_ACC_V = [0.5, -0.2, -0.4]  # acc value
_TURNING_ACC_BP = [1., 2., 3.]  # absolute value of current lat acc


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


class TurnCalculatorPolicy(Enum):
  driving_path_priority = 'driving_path_priority'
  map_data_priority = 'map_data_priority'
  combined = 'combined'


class TurnCalculator():
  class Key(Enum):
    driving_path = 'driving_path'
    map_data = 'map_data'
    max_curvature = 'max_curvature'
    overshoot_ahead = 'overshoot_ahead'
    v_target_distance = 'v_target_distance'

  def __init__(self, v_ego, v_cruise_setpoint, d_poly, sm, policy=TurnCalculatorPolicy.map_data_priority):
    self._v_cruise_setpoint = v_cruise_setpoint
    self._v_ego = v_ego
    self._d_poly = d_poly
    self._sm = sm
    self._a_lat_reg_max = interp(v_ego, _A_LAT_REG_MAX_BP, _A_LAT_REG_MAX_V)
    self._max_curvature_for_vego = self._a_lat_reg_max / max(self._v_ego, 0.1)**2
    self._policy = policy

  def calculate(self):
    self._results = {}

    if self._policy == TurnCalculatorPolicy.combined:
      self._calculate_from_driving_path()
      self._calculate_from_map_data()
    elif self._policy == TurnCalculatorPolicy.map_data_priority:
      self._calculate_from_map_data()
      if TurnCalculator.Key.map_data not in self._results:
        self._calculate_from_driving_path()
    elif self._policy == TurnCalculatorPolicy.driving_path_priority:
        self._calculate_from_driving_path()
    self._consolidate()

  def _calculate_from_driving_path(self):
    curvatures = eval_curvature(self._d_poly, _EVAL_RANGE)
    max_curvature_idx = np.argmax(curvatures)
    lat_acc_overshoot_idxs = np.nonzero(curvatures >= self._max_curvature_for_vego)[0]
    overshoot_ahead = len(lat_acc_overshoot_idxs) > 0
    v_target_distance = max(lat_acc_overshoot_idxs[0] * _EVAL_STEP + _EVAL_START, _EVAL_STEP) if overshoot_ahead \
        else _MAX_DISTANCE_HORIZON
    self._results[TurnCalculator.Key.driving_path] = {
        TurnCalculator.Key.max_curvature: curvatures[max_curvature_idx],
        TurnCalculator.Key.overshoot_ahead: overshoot_ahead,
        TurnCalculator.Key.v_target_distance: v_target_distance
    }
    print(f'TC: ********* Driving Path Result:\n{self._results[TurnCalculator.Key.driving_path]}\n'
          f'Max curv distance: {max_curvature_idx * _EVAL_STEP + _EVAL_START}.\n'
          f'Overshoot curv: {curvatures[lat_acc_overshoot_idxs[0]] if len(lat_acc_overshoot_idxs) else None}.\n')

  def _calculate_from_map_data(self):
    sock = 'liveMapData'
    # Ignore if no live map data
    if self.sm.logMonoTime[sock] is None:
      return

    # Process Live Map data.
    map_data_age = sec_since_boot() - self.sm.logMonoTime[sock] * 1e-9
    map_data = self.sm[sock]
    curvatures = np.abs(np.array(map_data.roadCurvature))
    curvature_distances = np.array(map_data.roadCurvatureX)
    # Ignore if not curvatures fetched.
    if len(curvatures) == 0:
      return

    # Correct distances to curvature by estimation of distance traveled since map data was issued.
    distance_correction = map_data_age * (self._v_ego - self._a_ego * map_data_age / 2.)
    curvature_distances -= distance_correction
    # Reshape data to only include positive distances not greater than horizon
    condition = (curvature_distances > 0) & (curvature_distances <= _MAX_DISTANCE_HORIZON)
    curvatures = curvatures[condition]
    curvature_distances = curvature_distances[condition]
    # Ignore if not curvatures fetched.
    if len(curvatures) == 0:
      return

    # Calculate lateral accelerations overshoots.
    max_curvature_idx = np.argmax(curvatures)
    lat_acc_overshoot_idxs = np.nonzero(curvatures >= self._max_curvature_for_vego)[0]
    overshoot_ahead = len(lat_acc_overshoot_idxs) > 0
    v_target_distance = curvature_distances[lat_acc_overshoot_idxs[0]] if overshoot_ahead else _MAX_DISTANCE_HORIZON
    self._results[TurnCalculator.Key.map_data] = {
        TurnCalculator.Key.max_curvature: curvatures[max_curvature_idx],
        TurnCalculator.Key.overshoot_ahead: overshoot_ahead,
        TurnCalculator.Key.v_target_distance: v_target_distance
    }
    print(f'TC: ********* Map Data Result:\n{self._results[TurnCalculator.Key.map_data]}\n'
          f'Max curv distance: {curvature_distances[max_curvature_idx]}.\n'
          f'Overshoot curv: {curvatures[lat_acc_overshoot_idxs[0]] if len(lat_acc_overshoot_idxs) else None}.\n')

  def _consolidate(self):
    # Get max map curvature and lat acc as the maximum from both solutions
    self.max_pred_curvature = max(map(lambda k: self._results[k][TurnCalculator.Key.max_curvature], self._results))
    self.max_pred_lat_acc = self._v_ego**2 * self.max_pred_curvature
    # Overshoot ahead and target velocity and distance to it.
    self.lat_acc_overshoot_ahead = bool(max(map(lambda k: self._results[k][TurnCalculator.Key.overshoot_ahead],
                                                self._results)))
    self.v_target_distance = min(map(lambda k: self._results[k][TurnCalculator.Key.v_target_distance], self._results))
    self.v_target = min(math.sqrt(self._a_lat_reg_max / self.max_pred_curvature), self._v_cruise_setpoint) if \
        self.lat_acc_overshoot_ahead else 0.
    print('TC: ********* Consolidated Turn Calculator result:\n'
          f'Max curv: {self.max_pred_curvature}.\n'
          f'Max lat a: {self.max_pred_lat_acc}.\n'
          f'overshoot ahead: {self.lat_acc_overshoot_ahead}.\n'
          f'v target distance: {self.v_target_distance}.\n'
          f'v target: {self.v_target}.\n')


class TurnController():
  def __init__(self, CP):
    self._params = Params()
    self._CP = CP
    self._op_enabled = False
    self._min_braking_acc = float(self._params.get("MaxDecelerationForTurns", True))
    self._jerk_limits = [self._min_braking_acc, _MAX_JERK_ACC_INCREASE]
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

  @property
  def is_active(self):
    return self._state != TurnState.DISABLED

  @state.setter
  def state(self, value):
    if value != self._state:
      print(f'TurnController state: {value.description}')
      if value == TurnState.DISABLED:
        self._reset()
    self._state = value

  def _reset(self):
    self._v_turn_future = 0.0
    self._current_curvature = 0.0
    self._d_poly = [0., 0., 0., 0.]
    self._max_pred_curvature = 0.0
    self._max_pred_lat_acc = 0.0
    self._v_target_distance = _MAX_DISTANCE_HORIZON
    self._v_target = 0.0
    self._lat_acc_overshoot_ahead = False

    self.a_turn = 0.0
    self.v_turn = 0.0

  # TODO: Remove method below after validating new method spits same results
  def _update_calculations_alt(self):
    pred_curvatures = eval_curvature(self._d_poly, _EVAL_RANGE)
    max_pred_curvature_idx = np.argmax(pred_curvatures)
    max_pred_curvature = pred_curvatures[max_pred_curvature_idx]
    max_pred_lat_acc = self._v_ego**2 * max_pred_curvature

    a_lat_reg_max = interp(self._v_ego, _A_LAT_REG_MAX_BP, _A_LAT_REG_MAX_V)
    max_curvature_for_vego = a_lat_reg_max / max(self._v_ego, 0.1)**2
    lat_acc_overshoot_idxs = np.nonzero(pred_curvatures >= max_curvature_for_vego)[0]
    lat_acc_overshoot_ahead = len(lat_acc_overshoot_idxs) > 0

    if lat_acc_overshoot_ahead:
      v_target_distance = max(lat_acc_overshoot_idxs[0] * _EVAL_STEP + _EVAL_START, _EVAL_STEP)
      v_target = min(math.sqrt(a_lat_reg_max / max_pred_curvature), v_cruise_setpoint)
      print(f'Tc: OLD CALC: High Lat Acc ahead. Distance: {v_target_distance:.2f}, target v: {v_target:.2f}')

  def _update_calculations(self):
    calculator = TurnCalculator(self._v_ego, self._v_cruise_setpoint, self._d_poly, self._sm)
    calculator.calculate()
    self._max_pred_curvature = calculator.max_pred_curvature
    self._max_pred_lat_acc = calculator.max_pred_lat_acc
    self._lat_acc_overshoot_ahead = calculator.lat_acc_overshoot_ahead
    self._v_target_distance = calculator.v_target_distance
    self._v_target = calculator.v_target

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
      elif self._max_pred_curvature >= _ENTERING_PRED_CURVATURE_TH \
              and self._max_pred_lat_acc >= _ENTERING_PRED_LAT_ACC_TH:
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
    # Calculate target acceleration based on turn state.
    # DISABLED
    if self.state == TurnState.DISABLED:
      a_target = self._a_ego
    # ENTERING
    elif self.state == TurnState.ENTERING:
      entering_smooth_decel = interp(self._max_pred_lat_acc, _ENTERING_SMOOTH_DECEL_BP, _ENTERING_SMOOTH_DECEL_V)
      print(f'Overshooting {self._lat_acc_overshoot_ahead}, _entering_smooth_decel {entering_smooth_decel:.2f}')
      if self._lat_acc_overshoot_ahead:
        a_target = min((self._v_target**2 - self._v_ego**2) / (2 * self._v_target_distance), entering_smooth_decel)
      else:
        a_target = entering_smooth_decel
    # TURNING
    elif self.state == TurnState.TURNING:
      current_lat_acc = self._current_curvature * self._v_ego**2
      a_target = interp(current_lat_acc, _TURNING_ACC_BP, _TURNING_ACC_V)
    # LEAVING
    elif self.state == TurnState.LEAVING:
      a_target = _LEAVING_ACC

    # smooth out acceleration using jerk limits.
    j_limits = np.array(self._jerk_limits)
    a_limits = self._a_ego + j_limits * _LON_MPC_STEP
    a_target = max(min(a_target, a_limits[1]), a_limits[0])

    # calculate solution values.
    self.a_turn = max(a_target, self._min_braking_acc)  # acceleration in next Longitudinal control step.
    self.v_turn = self._v_ego + self.a_turn * _LON_MPC_STEP  # speed in next Longitudinal control step.
    self._v_turn_future = self._v_ego + self.a_turn * 4.  # speed in 4 seconds.

  def update(self, enabled, v_ego, a_ego, v_cruise_setpoint, d_poly, sm):
    self._op_enabled = enabled
    self._v_ego = v_ego
    self._a_ego = a_ego
    self._v_cruise_setpoint = v_cruise_setpoint
    self._d_poly = d_poly
    self._sm = sm
    self._current_curvature = abs(sm['carState'].steeringAngle * CV.DEG_TO_RAD / (self._CP.steerRatio *
                                                                                  self._CP.wheelbase))

    self._update_calculations()
    self._update_calculations_alt()
    self._state_transition()
    self._update_solution()
