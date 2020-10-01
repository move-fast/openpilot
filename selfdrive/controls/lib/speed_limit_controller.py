import numpy as np
from enum import Enum
from selfdrive.controls.lib.speed_smoother import speed_smoother
from cereal import car


_LON_MPC_STEP = 0.2  # Time stemp of longitudinal control (5 Hz)

_MIN_ADAPTING_BRAKE_ACC = -2.0  # Minimum acceleration allowed when adapting to lower speed limit.
_SPEED_OFFSET_TH = -5.0  # Maximum offset between speed limit and current speed for ADAPTING state.
_LIMIT_ADAPT_TIME = 4.0  # Ideal time (s) to adapt to lower speed limit. i.e. braking.


class LimitState(Enum):
  INACTIVE = 1  # No speed limit set.
  ADAPTING = 2  # Reducing speed to match new speed limit.
  ACTIVE = 3  # Cruising at speed limit.


class SpeedLimitController():
  def __init__(self):
    self._op_enabled = False
    self._active_jerk_limits = [0.0, 0.0]
    self._active_accel_limits = [0.0, 0.0]
    self._adapting_jerk_limits = [_MIN_ADAPTING_BRAKE_ACC, 1.0]
    self._v_ego = 0.0
    self._a_ego = 0.0
    self._v_offset = 0.0
    self._speed_limit = 0.0
    self._speed_limit_offset = 0.0
    self._state = LimitState.INACTIVE
    self._adapting_cycles = 0

    self.v_limit = 0.0
    self.a_limit = 0.0
    self.v_limit_future = 0.0

  @property
  def is_active(self):
    return self._state != LimitState.INACTIVE

  @property
  def speed_limit(self):
    return self._speed_limit + self._speed_limit_offset

  def _update_calculations(self):
    # Update speed limit offset: reset to 0 while inactive, react to button events otherwise.
    if not self.is_active:
      self._speed_limit_offset = 0.0
    else:
      for b in self._button_events:
        if not b.pressed:
          if b.type == car.CarState.ButtonEvent.Type.accelCruise:
            self._speed_limit_offset += 1
          elif b.type == car.CarState.ButtonEvent.Type.decelCruise:
            self._speed_limit_offset -= 1
    # Update current velocity offset (error)
    self._v_offset = self.speed_limit - self._v_ego

  def _state_transition(self):
    # In any case, if system is disabled or the reported speed limit is 0, deactivate.
    if not self._op_enabled or self._speed_limit == 0:
      self._state = LimitState.INACTIVE
      return

    # INACTIVE
    if self._state == LimitState.INACTIVE:
      # If the limit speed offset is negative (i.e. reduce speed) and lower than threshold
      # we go to ADAPTING state to quickly reduce speed, otherwise we go directly to ACTIVE
      if self._v_offset < _SPEED_OFFSET_TH:
        self._state = LimitState.ADAPTING
        self._adapting_cycles = 0
      else:
        self._state = LimitState.ACTIVE
    # ADAPTING
    elif self._state == LimitState.ADAPTING:
      self._adapting_cycles += 1
      # Go to ACTIVE once the speed offset is over threshold.
      if self._v_offset >= _SPEED_OFFSET_TH:
        self._state = LimitState.ACTIVE
    # ACTIVE
    elif self._state == LimitState.ACTIVE:
      # Go to ADAPTING if the speed offset goes below threshold.
      if self._v_offset < _SPEED_OFFSET_TH:
        self._state = LimitState.ADAPTING
        self._adapting_cycles = 0

  def _update_solution(self):
    # INACTIVE
    if self._state == LimitState.INACTIVE:
      # Preserve values
      self.v_limit = self._v_ego
      self.a_limit = self._a_ego
      self.v_limit_future = self._v_ego
    # ADAPTING
    elif self._state == LimitState.ADAPTING:
      # Calculate to adapt speed on target time.
      adapting_time = _LIMIT_ADAPT_TIME - self._adapting_cycles * _LON_MPC_STEP
      adapting_distance = max(self._v_ego * adapting_time, 5.0)  # minimum adapting distance is 5m.
      a_target = (self.speed_limit**2 - self._v_ego**2) / (2 * adapting_distance)
      # smooth out acceleration using jerk limits.
      j_limits = np.array(self._adapting_jerk_limits)
      a_limits = self._a_ego + j_limits * _LON_MPC_STEP
      a_target = max(min(a_target, a_limits[1]), a_limits[0])
      # calculate the solution values
      self.a_limit = max(a_target, _MIN_ADAPTING_BRAKE_ACC)  # acceleration in next Longitudinal control step.
      self.v_limit = self._v_ego + self.a_limit * _LON_MPC_STEP  # speed in next Longitudinal control step.
      self.v_limit_future = self._v_ego + self.a_limit * 4.  # speed in 4 seconds.
    # ACTIVE
    elif self._state == LimitState.ACTIVE:
      # Calculate following same cruise logic in planner.py
      self.v_limit, self.a_limit = speed_smoother(self._v_ego, self._a_ego, self.speed_limit,
                                                  self._active_accel_limits[1], self._active_accel_limits[0],
                                                  self._active_jerk_limits[1], self._active_jerk_limits[0],
                                                  _LON_MPC_STEP)
      self.v_limit = max(self.v_limit, 0.)
      self.v_limit_future = self._speed_limit

  def update(self, enabled, v_ego, a_ego, car_state, accel_limits, jerk_limits):
    self._op_enabled = enabled
    self._v_ego = v_ego
    self._a_ego = a_ego
    self._speed_limit = car_state.cruiseState.speedLimit
    self._active_accel_limits = accel_limits
    self._active_jerk_limits = jerk_limits
    self._button_events = car_state.buttonEvents

    self._update_calculations()
    self._state_transition()
    self._update_solution()

  def deactivate(self):
    self._state = LimitState.INACTIVE
