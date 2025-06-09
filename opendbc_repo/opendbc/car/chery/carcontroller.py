import numpy as np
from opendbc.can.packer import CANPacker
from opendbc.car import Bus, DT_CTRL, apply_std_steer_angle_limits, structs
from opendbc.car.chery import cherycan
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.chery.values import DBC, CarControllerParams
from opendbc.car.interfaces import CarControllerBase

VisualAlert = structs.CarControl.HUDControl.VisualAlert
NetworkLocation = structs.CarParams.NetworkLocation
LongCtrlState = structs.CarControl.Actuators.LongControlState
BUTTONS_STATES = ["accelCruise", "decelCruise", "cancel", "resumeCruise"]

# Camera cancels up to 0.1s after brake is pressed, ECM allows 0.5s
CAMERA_CANCEL_DELAY_FRAMES = 10
# Enforce a minimum interval between steering messages to avoid a fault
MIN_STEER_MSG_INTERVAL_MS = 15

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.CP = CP
    self.CAN = cherycan.CanBus(CP)
    self.car_fingerprint = CP.carFingerprint
    self.packer = CANPacker(DBC[CP.carFingerprint][Bus.pt])
    self.params = CarControllerParams(self.CP)
    self.frame = 0

    self.start_time = 0.
    self.apply_steer_last = 0
    self.apply_gas = 0
    self.apply_brake = 0
    self.apply_angle_last = 0
    self.last_steer_frame = 0
    self.last_button_frame = 0
    self.brake_counter = 0
    self.cancel_counter = 0

    self.lka_steering_cmd_counter = 0
    self.lka_steering_cmd_counter_last = -1

    self.lka_icon_status_last = (False, False)

    self.speed_limit_control_enabled = False
    self.last_speed_limit_sign_tap = False
    self.last_speed_limit_sign_tap_prev = False
    self.speed_limit = 0.
    self.speed_limit_offset = 0
    self.timer = 0
    self.final_speed_kph = 0
    self.init_speed = 0
    self.current_speed = 0
    self.v_set_dis = 0
    self.v_cruise_min = 0
    self.button_type = 0
    self.button_select = 0
    self.button_count = 0
    self.target_speed = 0
    self.t_interval = 7
    self.slc_active_stock = False
    self.sl_force_active_timer = 0
    self.v_tsc_state = 0
    self.slc_state = 0
    self.m_tsc_state = 0
    self.cruise_button = None
    self.speed_diff = 0
    self.v_tsc = 0
    self.m_tsc = 0
    self.steady_speed = 0
    self.steering_pressed_counter = 0
    self.steering_unpressed_counter = 0
    self.steerDisableTemp = False

    self.prev_gas = 0
    self.prev_accel = 0

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel
    experimentalMode = True
    # hud_control = CC.hudControl
    # hud_alert = hud_control.visualAlert
    # hud_v_cruise = hud_control.setSpeed

    ### STEER ###
    steer_hud_alert = 1 if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw) else 0

    if CC.cruiseControl.cancel and (self.frame % self.params.BUTTONS_STEP) == 0:
      # can_sends.append(cherycan.create_button_msg(self.packer, self.CAN.camera,self.frame, CS.buttons_stock_values, cancel=True))
      print('Send Cancel')

    elif (CC.cruiseControl.resume) and (self.frame % self.params.BUTTONS_STEP) == 0:
      # can_sends.append(cherycan.create_button_msg(self.packer, self.CAN.camera, self.frame, CS.buttons_stock_values, resume=True))
      print('Send Resume')
    else:
      self.brake_counter = 0

    self.steering_pressed_counter = self.steering_pressed_counter + 1 if abs(CS.out.steeringTorque) >= 50 else 0
    # Make LKA Temporary disable when driver try to override
    if self.steering_pressed_counter * DT_CTRL > 1:
      self.steerDisableTemp = True
      self.steering_unpressed_counter = 0
    else:
      self.steering_unpressed_counter += 1
      if self.steering_unpressed_counter * DT_CTRL > 1:
        self.steerDisableTemp = False

    ### lateral control ###
    # send steer msg at 50Hz
    apply_steer_req = False
    if  (self.frame  % self.params.STEER_STEP) == 0:
      if CC.latActive and not self.steerDisableTemp:
        apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg, CC.latActive, CarControllerParams.ANGLE_LIMITS)
        # print('Apply angle:',apply_angle)
        # apply_steer_req = CC.latActive and not CS.out.standstill
        apply_steer_req = CC.latActive
      else:
        apply_angle = CS.out.steeringAngleDeg
          # ovveride human steer
      # if abs(CS.out.steeringTorque) >= 50:
      #   apply_angle = CS.out.steeringAngleDeg

      self.apply_angle_last = apply_angle
      self.last_steer_frame = self.frame

      # print('Apply steer.',apply_steer)
      can_sends.append(cherycan.create_steering_control_lkas(self.packer, self.CAN.main, apply_angle, self.frame, apply_steer_req, CS.lkas_cmd))

    # if  (self.frame  % self.params.LKAS_HUD_STEP) == 0:
    #   can_sends.append(cherycan.create_lkas_state(self.packer, 0, self.frame, CC.latActive, CS.lkas_state))

    ### longitudinal control ###
    # send acc msg at 50Hz
    if self.CP.openpilotLongitudinalControl and (self.frame % CarControllerParams.ACC_CONTROL_STEP) == 0:
      full_stop = CC.longActive and CS.out.standstill
      accel = int(round(np.interp(actuators.accel, self.params.ACCEL_LOOKUP_BP, self.params.ACCEL_LOOKUP_V)))
      gas = accel

      if gas > 0:
        full_stop = 0

      self.prev_gas = gas

      # full_stop = 0

      if not CC.longActive:
        gas = CarControllerParams.INACTIVE_GAS
      else:
        print(f'actuator accell {actuators.accel}, accel {accel}, gas {gas}, full_stop {full_stop}, CC.longActive {CC.longActive}, CS.out.standstill {CS.out.standstill}' )
      stopping = CC.actuators.longControlState == LongCtrlState.stopping
      if experimentalMode:
        can_sends.append(cherycan.create_longitudinal_control(self.packer, self.CAN.main, CS.acc_md, self.frame, CC.longActive, gas, accel, stopping, full_stop))
      else:
        can_sends.append(cherycan.create_longitudinal_controlBypass(self.packer, self.CAN.main, CS.acc_md, self.frame))

    if self.frame % 20 == 0:
      # ldw = CC.hudControl.visualAlert == VisualAlert.ldw
      # steer_required = CC.hudControl.visualAlert == VisualAlert.steerRequired
      can_sends.append(cherycan.create_lkas_state_hud(self.packer, self.CAN.main, self.frame, CS.lkas_state, apply_steer_req))

    new_actuators = CC.actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    # new_actuators.accel = accel

    self.frame += 1
    return new_actuators, can_sends
