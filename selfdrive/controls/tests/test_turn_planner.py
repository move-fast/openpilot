import numpy as np
from common.numpy_fast import interp
import unittest

from selfdrive.controls.lib.turn_planner import eval_lat_acc, limit_accel_in_turns, limit_accel_for_turn_ahead, \
    _EVAL_RANGE, _ACC_SAFETY_OFFSET, TURN_SMOOTH_DECEL
from selfdrive.controls.lib.planner import _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V
from cereal import car

_POLY_STRONG_LEFT = [0.0001, -0.001, 0.01, 0.1]
_POLY_MILD_LEFT = [0.00001, -0.001, 0.01, 0.1]
_POLY_STRONG_RIGHT = [-0.0001, 0.001, -0.01, -0.1]
_POLY_MILD_RIGHT = [-0.00001, 0.001, -0.01, -0.1]

_V_EGO_TEST = 15.
_A_MAX_TOTAL = interp(_V_EGO_TEST, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)


def mock_cp():
  cp = car.CarParams.new_message()
  cp.steerRatio = 15.
  cp.wheelbase = 3.
  return cp


class TestTurnPlanner(unittest.TestCase):
  def test_lat_acc_calculation(self):
    results = eval_lat_acc(_POLY_STRONG_LEFT, _V_EGO_TEST, _EVAL_RANGE)
    np.testing.assert_array_almost_equal(
        results,
        np.array([
            0.45, 0.22, 0.9, 1.57, 2.22, 2.83, 3.35, 3.73, 3.92, 3.89, 3.66, 3.28, 2.83, 2.36, 1.93, 1.55, 1.23, 0.98,
            0.78, 0.62, 0.49, 0.39, 0.32, 0.26, 0.21, 0.17, 0.14, 0.12, 0.1, 0.08, 0.07, 0.06, 0.05, 0.04, 0.04, 0.03,
            0.03, 0.02, 0.02, 0.02
        ]), 2)

  def test_turn_ahead_must_decelerate(self):
    safety_decel = -0.6605
    lower_limit = safety_decel - _ACC_SAFETY_OFFSET + 0.01
    limits = [lower_limit, 2.]
    for poly in [_POLY_STRONG_RIGHT, _POLY_STRONG_LEFT]:
      new_limits = limit_accel_for_turn_ahead(_V_EGO_TEST, poly, limits)
      self.assertAlmostEqual(new_limits[1], safety_decel, 3)
      self.assertEqual(new_limits[0], limits[0])

  def test_turn_ahead_postpone_decelerate(self):
    safety_decel = -0.6605
    lower_limit = safety_decel - _ACC_SAFETY_OFFSET - 0.01
    limits = [lower_limit, 2.]
    for poly in [_POLY_STRONG_RIGHT, _POLY_STRONG_LEFT]:
      new_limits = limit_accel_for_turn_ahead(_V_EGO_TEST, poly, limits)
      self.assertEqual(new_limits, limits)

  def test_turn_ahead_no_deceleration(self):
    limits = [-0.7, 1.5]
    for poly in [_POLY_MILD_LEFT, _POLY_MILD_RIGHT]:
      new_limits = limit_accel_for_turn_ahead(_V_EGO_TEST, poly, limits)
      self.assertEqual(new_limits, limits)

  def test_turn_must_decelerate(self):
    limits = [-0.7, 1.5]
    new_limits = limit_accel_in_turns(_V_EGO_TEST, _A_MAX_TOTAL, 90., limits, mock_cp())
    self.assertEqual(new_limits[0], limits[0])
    self.assertEqual(new_limits[1], TURN_SMOOTH_DECEL)

  def test_turn_aceleration_ok_but_reduce_limit(self):
    limits = [-0.7, 1.5]
    new_limits = limit_accel_in_turns(_V_EGO_TEST, _A_MAX_TOTAL, 15., limits, mock_cp())
    self.assertEqual(new_limits[0], limits[0])
    self.assertAlmostEqual(new_limits[1], 1.0846, 3)

  def test_turn_aceleration_ok(self):
    limits = [-0.7, 1.5]
    new_limits = limit_accel_in_turns(_V_EGO_TEST, _A_MAX_TOTAL, 5., limits, mock_cp())
    self.assertEqual(new_limits, limits)


if __name__ == "__main__":
  unittest.main()
