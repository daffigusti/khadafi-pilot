#!/usr/bin/env python3
from cereal import car
from panda import Panda
from openpilot.common.conversions import Conversions as CV

from openpilot.selfdrive.car import create_button_events, get_safety_config, create_mads_event
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.selfdrive.car.wuling.values import CAR, CruiseButtons, PREGLOBAL_CARS, CarControllerParams, CanBus
from common.params import Params
from common.op_params import opParams

ButtonType = car.CarState.ButtonEvent.Type
TransmissionType = car.CarParams.TransmissionType
GearShifter = car.CarState.GearShifter
EventName = car.CarEvent.EventName
ENABLE_BUTTONS = (CruiseButtons.RES_ACCEL, CruiseButtons.DECEL_SET, CruiseButtons.CANCEL)

BUTTONS_DICT = {CruiseButtons.RES_ACCEL: ButtonType.accelCruise, CruiseButtons.DECEL_SET: ButtonType.decelCruise,
                CruiseButtons.MAIN: ButtonType.setCruise, CruiseButtons.RES_ACCEL: ButtonType.resumeCruise,
                CruiseButtons.GAP_UP: ButtonType.gapAdjustCruise, CruiseButtons.GAP_DOWN: ButtonType.gapAdjustCruise,
                CruiseButtons.CANCEL: ButtonType.cancel}

CRUISE_OVERRIDE_SPEED_MIN = 5 * CV.KPH_TO_MS

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)

    self.dp_cruise_speed = 0. # km/h
    self.dp_override_speed_last = 0. # km/h
    self.dp_override_speed = 0. # m/s

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "wuling"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.wuling)]
    ret.radarUnavailable = True
    ret.dashcamOnly = candidate in PREGLOBAL_CARS
    
    op_params = opParams("wuling car_interface.py for lateral override")
    tire_stiffness_factor = 0.444
    
    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.25
    
    ret.mass = 1950.
    ret.wheelbase = 2.75
    ret.steerRatio = op_params.get('steer_ratio', force_update=True)
    ret.centerToFront = ret.wheelbase * 0.4
    ret.tireStiffnessFactor = 0.82

    ret.transmissionType = TransmissionType.automatic

    ret.minEnableSpeed = -1
    ret.minSteerSpeed = -1
    
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    ret.pcmCruise = not ret.openpilotLongitudinalControl
    ret.customStockLongAvailable = True
    ret.stoppingControl = True
    ret.startingState = True
    
    return ret

 
  # returns a car.CarState
  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam, self.cp_loopback)
    self.CS = self.sp_update_params(self.CS)

    buttonEvents = []

    # Don't add event if transitioning from INIT, unless it's to an actual button
    if self.CS.cruise_buttons != CruiseButtons.UNPRESS or self.CS.prev_cruise_buttons != CruiseButtons.INIT:
      buttonEvents = create_button_events(self.CS.cruise_buttons, self.CS.prev_cruise_buttons, BUTTONS_DICT,
                                          unpressed_btn=CruiseButtons.UNPRESS)
    self.CS.mads_enabled = self.get_sp_cruise_main_state(ret, self.CS)

    if not self.CP.pcmCruise:
      if any(b.type == ButtonType.accelCruise and b.pressed for b in buttonEvents):
        self.CS.accEnabled = True

    self.CS.accEnabled, buttonEvents = self.get_sp_v_cruise_non_pcm_state(ret, self.CS.accEnabled,
                                                                          buttonEvents, c.vCruise)

    if ret.cruiseState.available:
      if self.enable_mads:
        if not self.CS.prev_mads_enabled and self.CS.mads_enabled:
          self.CS.madsEnabled = True
        if self.CS.prev_lkas_enabled != 1 and self.CS.lkas_enabled == 1:
          self.CS.madsEnabled = not self.CS.madsEnabled
        self.CS.madsEnabled = self.get_acc_mads(ret.cruiseState.enabled, self.CS.accEnabled, self.CS.madsEnabled)
      ret, self.CS = self.toggle_gac(ret, self.CS, bool(self.CS.gap_dist_button), 1, 3, 3, "-")
    else:
      self.CS.madsEnabled = False

    if not self.CP.pcmCruise or (self.CP.pcmCruise and self.CP.minEnableSpeed > 0):
      if any(b.type == ButtonType.cancel for b in buttonEvents):
        self.CS.madsEnabled, self.CS.accEnabled = self.get_sp_cancel_cruise_state(self.CS.madsEnabled)
    if self.get_sp_pedal_disengage(ret):
      self.CS.madsEnabled, self.CS.accEnabled = self.get_sp_cancel_cruise_state(self.CS.madsEnabled)
      ret.cruiseState.enabled = False if self.CP.pcmCruise else self.CS.accEnabled

    if self.CP.pcmCruise and self.CP.minEnableSpeed > 0 and self.CP.pcmCruiseSpeed:
      if ret.gasPressed and not ret.cruiseState.enabled:
        self.CS.accEnabled = False
      self.CS.accEnabled = ret.cruiseState.enabled or self.CS.accEnabled

    ret, self.CS = self.get_sp_common_state(ret, self.CS, gap_button=bool(self.CS.gap_dist_button))

    # MADS BUTTON
    if self.CS.out.madsEnabled != self.CS.madsEnabled:
      if self.mads_event_lock:
        buttonEvents.append(create_mads_event(self.mads_event_lock))
        self.mads_event_lock = False
    else:
      if not self.mads_event_lock:
        buttonEvents.append(create_mads_event(self.mads_event_lock))
        self.mads_event_lock = True

    ret.buttonEvents = buttonEvents

    # The ECM allows enabling on falling edge of set, but only rising edge of resume
    events = self.create_common_events(ret, c, extra_gears=[GearShifter.sport, GearShifter.low,
                                                         GearShifter.eco, GearShifter.manumatic],
                                       pcm_enable=False, enable_buttons=(ButtonType.decelCruise,))
    #if not self.CP.pcmCruise:
    #  if any(b.type == ButtonType.accelCruise and b.pressed for b in ret.buttonEvents):
    #    events.add(EventName.buttonEnable)

    events, ret = self.create_sp_events(self.CS, ret, events, enable_pressed=self.CS.accEnabled,
                                        enable_buttons=(ButtonType.decelCruise,))

    # Enabling at a standstill with brake is allowed
    # TODO: verify 17 Volt can enable for the first time at a stop and allow for all GMs
    below_min_enable_speed = ret.vEgo < self.CP.minEnableSpeed or self.CS.moving_backward
    if below_min_enable_speed and not (ret.standstill and ret.brake >= 20):
      events.add(EventName.belowEngageSpeed)
    if ret.cruiseState.standstill:
      events.add(EventName.resumeRequired)
    if ret.vEgo < self.CP.minSteerSpeed and self.CS.madsEnabled:
      events.add(EventName.belowSteerSpeed)

    ret.events = events.to_msg()

    return ret

  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos)
