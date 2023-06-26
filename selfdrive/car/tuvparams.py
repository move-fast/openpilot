from common.params import Params


class TuvParams:
  def __init__(self):
    # Defaults to Toyota values with Torque limiting and Acc clipping.
    self.ACCEL_MAX = 1.5  # m/s2, lower than allowed 2.0 m/s2 for tuning reasons
    self.ACCEL_MIN = -3.5  # m/s2
    self.STEER_MAX = 1500
    self.torqueRateLimiting = True
    self.accelerationClipping = True
    self.printAcc = False
    self.printSteer = False

  def load(self):
    tuv_params_str = Params().get('TuvTestParams', encoding='utf-8')
    if tuv_params_str is not None:
      tuv_params_arr = tuv_params_str.split(',')
      if len(tuv_params_arr) == 11:
        self.ACCEL_MAX = float(tuv_params_arr[0])
        self.ACCEL_MIN = float(tuv_params_arr[1])
        self.STEER_MAX = int(tuv_params_arr[2])
        self.torqueRateLimiting = bool(tuv_params_arr[3])
        self.accelerationClipping = bool(tuv_params_arr[4])
        self.printAcc = bool(tuv_params_arr[5])
        self.printSteer = bool(tuv_params_arr[6])
  
  def get_params_array(self):
    return [
      f'{self.ACCEL_MAX:.1f}',
      f'{self.ACCEL_MIN:.1f}',
      f'{self.STEER_MAX}',
      f'{int(self.torqueRateLimiting)}',
      f'{int(self.accelerationClipping)}',
      f'{int(self.printAcc)}',
      f'{int(self.printSteer)}'
    ]

  def get_params_names(self):
    return [
      'ACCEL_MAX',
      'ACCEL_MIN',
      'STEER_MAX',
      'torqueRateLimiting',
      'accelerationClipping',
      'printAcc',
      'printSteer'
    ]
  
  def write(self):
    tuv_params_str = ','.join(self.get_params_array())
    Params().put('TuvTestParams', tuv_params_str)

  def description(self):
    params_arr = self.get_params_array()
    params_names = self.get_params_names()
    return '\n'.join([f'{params_names[i]} -> {params_arr[i]}' for i in range(len(params_names))])