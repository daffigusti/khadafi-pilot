from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import interp
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car import apply_std_steer_angle_limits
from openpilot.selfdrive.car.chery import cherycan
from openpilot.selfdrive.car.chery.values import CanBus, DBC, CarControllerParams
import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CHERY_V_CRUISE_MIN

VisualAlert = car.CarControl.HUDControl.VisualAlert
NetworkLocation = car.CarParams.NetworkLocation
LongCtrlState = car.CarControl.Actuators.LongControlState
BUTTONS_STATES = ["accelCruise", "decelCruise", "cancel", "resumeCruise"]
ButtonType = car.CarState.ButtonEvent.Type


class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.start_time = 0.
    self.apply_steer_last = 0
    self.apply_gas = 0
    self.apply_brake = 0
    self.frame = 0
    self.apply_angle_last = 0
    self.last_steer_frame = 0
    self.last_button_frame = 0
    self.brake_counter = 0
    self.CAN = CanBus(CP)

    self.cancel_counter = 0

    self.lka_steering_cmd_counter = 0
    self.lka_steering_cmd_counter_last = -1

    self.lka_icon_status_last = (False, False)

    self.params = CarControllerParams(self.CP)
    self.packer_pt = CANPacker(DBC[self.CP.carFingerprint]['pt'])
    self.param_s = Params()

    self.sm = messaging.SubMaster(['longitudinalPlan'])
    self.is_metric = self.param_s.get_bool("IsMetric")
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

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators

    if not self.CP.pcmCruiseSpeed:
      self.sm.update(0)

      if self.sm.updated['longitudinalPlanSP']:
        self.v_tsc_state = self.sm['longitudinalPlanSP'].visionTurnControllerState
        self.slc_state = self.sm['longitudinalPlanSP'].speedLimitControlState
        self.m_tsc_state = self.sm['longitudinalPlanSP'].turnSpeedControlState
        self.speed_limit = self.sm['longitudinalPlanSP'].speedLimit
        self.speed_limit_offset = self.sm['longitudinalPlanSP'].speedLimitOffset
        self.v_tsc = self.sm['longitudinalPlanSP'].visionTurnSpeed
        self.m_tsc = self.sm['longitudinalPlanSP'].turnSpeed

      if self.frame % 200 == 0:
        self.speed_limit_control_enabled = self.param_s.get_bool("EnableSlc")
        self.is_metric = self.param_s.get_bool("IsMetric")
      self.last_speed_limit_sign_tap = self.param_s.get_bool("LastSpeedLimitSignTap")
      self.v_cruise_min = CHERY_V_CRUISE_MIN[self.is_metric] * (CV.KPH_TO_MPH if not self.is_metric else 1)

    # hud_control = CC.hudControl
    # hud_alert = hud_control.visualAlert
    # hud_v_cruise = hud_control.setSpeed


    # if CC.cruiseControl.cancel and (self.frame % self.params.BUTTONS_STEP) == 0:
    #   can_sends.append(cherycan.create_button_msg(self.packer_pt, self.CAN.camera, CS.buttons_stock_values, cancel=True))
    #   # can_sends.append(cherycan.create_button_msg(self.packer_pt, self.CAN.main, CS.buttons_stock_values, cancel=True))
    # elif CC.cruiseControl.resume and (self.frame % self.params.BUTTONS_STEP) == 0:
    #   can_sends.append(cherycan.create_button_msg(self.packer_pt, self.CAN.camera, CS.buttons_stock_values, resume=True))
    #   # can_sends.append(cherycan.create_button_msg(self.packer_pt, self.CAN.main, CS.buttons_stock_values, resume=True))
    # else:
    #   self.brake_counter = 0

    self.steering_pressed_counter = self.steering_pressed_counter + 1 if abs(CS.out.steeringTorque) >= 50 else 0
    # Make LKA Temporary disable when driver try to override
    if self.steering_pressed_counter * DT_CTRL > 2:
      self.steerDisableTemp = True
      self.steering_unpressed_counter = 0
      print('Steer Pressed')
    else:
      self.steering_unpressed_counter += 1
      if self.steering_unpressed_counter * DT_CTRL > 1:
        self.steerDisableTemp = False

    ### lateral control ###
    # send steer msg at 50Hz

    if  (self.frame  % self.params.STEER_STEP) == 0:
      if CC.latActive and not self.steerDisableTemp:
        apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgo, CarControllerParams)
        print('Apply angle:',apply_angle)
        apply_steer_req = True
      else:
        apply_angle = CS.out.steeringAngleDeg
        apply_steer_req = False

      self.apply_angle_last = apply_angle
      self.last_steer_frame = self.frame

      # print('Apply steer.',apply_steer)
      can_sends.append(cherycan.create_steering_control_lkas(self.packer_pt, apply_angle, self.frame, apply_steer_req, CS.lkas_cmd))

    # if  (self.frame  % self.params.LKAS_HUD_STEP) == 0:
    #   can_sends.append(cherycan.create_lkas_state(self.packer_pt, 0, self.frame, CC.latActive, CS.lkas_state))

    ### longitudinal control ###
    # send acc msg at 50Hz
    # print('Log ',now_nanos)
    if self.CP.openpilotLongitudinalControl and (self.frame % CarControllerParams.ACC_CONTROL_STEP) == 0:
      accel = int(round(interp(actuators.accel, self.params.ACCEL_LOOKUP_BP, self.params.ACCEL_LOOKUP_V)))
      # print('Log  Long',now_nanos)
      gas = accel
      if not CC.longActive:
        gas = CarControllerParams.INACTIVE_GAS
      else:
        print('Actuator accel : ',actuators.accel)
      stopping = CC.actuators.longControlState == LongCtrlState.stopping
      can_sends.append(cherycan.create_longitudinal_control(self.packer_pt, self.CAN.main, CS.acc_md, self.frame, CC.longActive, gas, accel, stopping))

    new_actuators = CC.actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    # new_actuators.accel = accel
    # new_actuators.gas = self.apply_gas
    # new_actuators.brake = self.apply_brake

    self.frame += 1
    return new_actuators, can_sends

  # multikyd methods, sunnyhaibin logic
  def get_cruise_buttons_status(self, CS):
    if not CS.out.cruiseState.enabled:
      for be in CS.out.buttonEvents:
        if be.type in (ButtonType.accelCruise, ButtonType.resumeCruise,
                       ButtonType.decelCruise, ButtonType.setCruise) and be.pressed:
          self.timer = 40
        elif be.type == ButtonType.gapAdjustCruise and be.pressed:
          self.timer = 300
    elif self.timer:
      self.timer -= 1
    else:
      return 1
    return 0

  def get_target_speed(self, v_cruise_kph_prev):
    v_cruise_kph = v_cruise_kph_prev
    if self.slc_state > 1:
      v_cruise_kph = (self.speed_limit + self.speed_limit_offset) * CV.MS_TO_KPH
      if not self.slc_active_stock:
        v_cruise_kph = v_cruise_kph_prev
    return v_cruise_kph

  def get_curve_speed(self, target_speed_kph, v_cruise_kph_prev):
    if self.v_tsc_state != 0:
      vision_v_cruise_kph = self.v_tsc * CV.MS_TO_KPH
      if int(vision_v_cruise_kph) == int(v_cruise_kph_prev):
        vision_v_cruise_kph = 255
    else:
      vision_v_cruise_kph = 255
    if self.m_tsc_state > 1:
      map_v_cruise_kph = self.m_tsc * CV.MS_TO_KPH
      if int(map_v_cruise_kph) == 0.0:
        map_v_cruise_kph = 255
    else:
      map_v_cruise_kph = 255
    curve_speed = self.curve_speed_hysteresis(min(vision_v_cruise_kph, map_v_cruise_kph) + 2 * CV.MPH_TO_KPH)
    return min(target_speed_kph, curve_speed)
