from cereal import car
from openpilot.common.numpy_fast import clip, interp
from openpilot.selfdrive.car import apply_meas_steer_torque_limits, apply_std_steer_angle_limits, common_fault_avoidance, \
                          create_gas_interceptor_command, make_can_msg
from openpilot.selfdrive.car.retrofit import retrofitcan
from openpilot.selfdrive.car.retrofit.values import CAR, STATIC_DSU_MSGS, MIN_ACC_SPEED, PEDAL_TRANSITION, CarControllerParams, ToyotaFlags
from opendbc.can.packer import CANPacker

SteerControlType = car.CarParams.SteerControlType
VisualAlert = car.CarControl.HUDControl.VisualAlert

# LKA limits
# EPS faults if you apply torque while the steering rate is above 100 deg/s for too long
MAX_STEER_RATE = 100  # deg/s
MAX_STEER_RATE_FRAMES = 18  # tx control frames needed before torque can be cut

# EPS allows user torque above threshold for 50 frames before permanently faulting
MAX_USER_TORQUE = 500

# LTA limits
# EPS ignores commands above this angle and causes PCS to fault
MAX_STEER_ANGLE = 94.9461  # deg
MAX_DRIVER_TORQUE_ALLOWANCE = 150  # slightly above steering pressed allows some resistance when changing lanes


class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.params = CarControllerParams(self.CP)
    self.frame = 0
    self.last_steer = 0
    self.last_angle = 0
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.steer_rate_counter = 0

    self.packer = CANPacker(dbc_name)
    self.gas = 0
    self.accel = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel
    lat_active = CC.latActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE

    # *** control msgs ***
    can_sends = []

    # *** steer torque ***
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    apply_steer = apply_meas_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, self.params)

    # >100 degree/sec steering fault prevention
    self.steer_rate_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringRateDeg) >= MAX_STEER_RATE, CC.latActive,
                                                                      self.steer_rate_counter, MAX_STEER_RATE_FRAMES)

    if not CC.latActive:
      apply_steer = 0

    apply_brake = actuators.brake * 7
    if not CC.enabled:
      apply_brake = 0

    # *** steer angle ***
    if self.CP.steerControlType == SteerControlType.angle:
      # If using LTA control, disable LKA and set steering angle command
      apply_steer = 0
      apply_steer_req = False
      if self.frame % 2 == 0:
        # EPS uses the torque sensor angle to control with, offset to compensate
        apply_angle = actuators.steeringAngleDeg + CS.out.steeringAngleOffsetDeg

        # Angular rate limit based on speed
        apply_angle = apply_std_steer_angle_limits(apply_angle, self.last_angle, CS.out.vEgo, self.params)

        if not lat_active:
          apply_angle = CS.out.steeringAngleDeg + CS.out.steeringAngleOffsetDeg

        self.last_angle = clip(apply_angle, -MAX_STEER_ANGLE, MAX_STEER_ANGLE)

    self.last_steer = apply_steer

    # toyota can trace shows this message at 42Hz, with counter adding alternatively 1 and 2;
    # sending it at 100Hz seem to allow a higher rate limit, as the rate limit seems imposed
    # on consecutive messages
    can_sends.append(retrofitcan.create_toyota_steer_command(self.packer, apply_steer, apply_steer_req))
    
    MAX_INTERCEPTOR_GAS = 0.5
    PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.4, 0.5, 0.0])
    pedal_offset = interp(CS.out.vEgo, [0.0, 2.3, MIN_ACC_SPEED + PEDAL_TRANSITION], [-.4, 0.0, 0.2])
    pedal_command = PEDAL_SCALE * (actuators.accel + pedal_offset)
    interceptor_gas_cmd = clip(pedal_command, 0., MAX_INTERCEPTOR_GAS)

    if self.frame % 2 == 0:
      # send exactly zero if gas cmd is zero. Interceptor will send the max between read value and gas cmd.
      # This prevents unexpected pedal range rescaling
      can_sends.append(retrofitcan.create_gas_command(self.packer, interceptor_gas_cmd, self.frame // 2))
      self.gas = interceptor_gas_cmd
      can_sends.append(retrofitcan.create_brake_cmd(self.packer, CC.enabled, apply_brake, self.frame // 2))

    # *** static msgs ***
    for addr, cars, bus, fr_step, vl in STATIC_DSU_MSGS:
      if self.frame % fr_step == 0 and self.CP.enableDsu and self.CP.carFingerprint in cars:
        can_sends.append(make_can_msg(addr, vl, bus))

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.steerOutputCan = apply_steer
    new_actuators.steeringAngleDeg = self.last_angle
    new_actuators.accel = self.accel
    new_actuators.gas = self.gas

    self.frame += 1
    return new_actuators, can_sends
