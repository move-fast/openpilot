import numpy as np
import math
from common.numpy_fast import interp

from selfdrive.config import Conversions as CV

_EVAL_STEP = 5.  # evaluate curvature every 5mts
_EVAL_LENGHT = 200.  # evaluate curvature for 200mts
_EVAL_RANGE = np.arange(0., _EVAL_LENGHT, _EVAL_STEP)
_ACC_SAFETY_OFFSET = 0.3  # Offset to cruise min acc where turn ahead limit kicks in.

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


def limit_accel_for_turn_ahead(v_ego, v_cruise_setpoint, d_poly, limits):
  """
  This function returns a limited long acceleration allowed to limit the maximum lateral acceleration
  on the predicted path ahead to the limit specified by regulation.
  The limit will be enforced only if the calculated limit is getting close to the already provided
  minimum acceleration cruise limit.
  """
  lat_accs = eval_lat_acc(d_poly, v_ego, _EVAL_RANGE)
  max_lat_acc_idx = np.argmax(lat_accs)
  distance_to_max_lat_acc = max(max_lat_acc_idx * _EVAL_STEP, 0.00001)
  max_lat_acc = lat_accs[max_lat_acc_idx]
  a_lat_reg_max = interp(v_ego, _A_LAT_REG_MAX_BP, _A_LAT_REG_MAX_V)
  decel_for_turn = max_lat_acc >= a_lat_reg_max

  if decel_for_turn:
    max_curvature = max_lat_acc / max(v_ego**2, 0.000001)
    v_target = min(math.sqrt(a_lat_reg_max / max_curvature), v_cruise_setpoint)
    acc_limit = (v_target**2 - v_ego**2) / (2 * distance_to_max_lat_acc)

    max_lon_acc = min(max(acc_limit, limits[0]), limits[1])
    v_turn = v_ego + max_lon_acc * 0.2  # speed in 0.2 seconds
    v_turn_future = v_ego + max_lon_acc * 4  # speed in 4 seconds

    print('-----------------------------')
    print(f'-> Ahead lat acceleration ({max_lat_acc:.2f}) in {distance_to_max_lat_acc:.0f} mts.')
#   print(f'-> v_ego: {v_ego}, v_target: {v_target:.2f}')
#   print(f'-> Provided acc limits: l: {limits[0]:.2f}  u: {limits[1]:.2f}')
#   print(f'-> acc_limit: {acc_limit:.2f}, new_upper_limit: {max_lon_acc:.2f}')
    print(f'-> **** Ahead lat acceleration too high. Setting top limit to: {max_lon_acc:.2f} ****')
  else:
    max_lon_acc = limits[1]
    v_turn = v_ego  # This value will have no efect.
    v_turn_future = v_ego  # This value will have no efect.

  return [limits[0], max_lon_acc], decel_for_turn, v_turn, max_lon_acc, float(v_turn_future)
