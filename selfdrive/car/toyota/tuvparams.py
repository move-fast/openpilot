from common.params import Params


class TuvParams:
  def __init__(self):
    self.ACCEL_MAX = 1.5  # m/s2, lower than allowed 2.0 m/s2 for tuning reasons
    self.ACCEL_MIN = -3.5  # m/s2
    self.STEER_MAX = 1500
    self.STEER_ERROR_MAX = 350     # max delta between torque cmd and torque motor
    self.STEER_DELTA_UP_torque = 15       # 1.0s time to peak torque
    self.STEER_DELTA_DOWN_torque = 25     # always lower than 45 otherwise the Rav4 faults (Prius seems ok with 50)
    self.STEER_DELTA_UP = 10       # 1.5s time to peak torque
    self.STEER_DELTA_DOWN = 25     # always lower than 45 otherwise the Rav4 faults (Prius seems ok with 50)
    self.torqueRateLimiting = True
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
        self.STEER_ERROR_MAX = int(tuv_params_arr[3])
        self.STEER_DELTA_UP_torque = int(tuv_params_arr[4])
        self.STEER_DELTA_DOWN_torque = int(tuv_params_arr[5])
        self.STEER_DELTA_UP = int(tuv_params_arr[6])
        self.STEER_DELTA_DOWN = int(tuv_params_arr[7])
        self.torqueRateLimiting = bool(tuv_params_arr[8])
        self.printAcc = bool(tuv_params_arr[9])
        self.printSteer = bool(tuv_params_arr[10])
  
  def get_params_array(self):
    return [
      f'{self.ACCEL_MAX:.1f}',
      f'{self.ACCEL_MIN:.1f}',
      f'{self.STEER_MAX}',
      f'{self.STEER_ERROR_MAX}',
      f'{self.STEER_DELTA_UP_torque}',
      f'{self.STEER_DELTA_DOWN_torque}',
      f'{self.STEER_DELTA_UP}',
      f'{self.STEER_DELTA_DOWN}',
      f'{int(self.torqueRateLimiting)}',
      f'{int(self.printAcc)}',
      f'{int(self.printSteer)}'
    ]

  def get_params_names(self):
    return [
      'ACCEL_MAX',
      'ACCEL_MIN',
      'STEER_MAX',
      'STEER_ERROR_MAX',
      'STEER_DELTA_UP_torque',
      'STEER_DELTA_DOWN_torque',
      'STEER_DELTA_UP',
      'STEER_DELTA_DOWN',
      'torqueRateLimiting',
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