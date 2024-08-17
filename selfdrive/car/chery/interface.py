#!/usr/bin/env python3
from cereal import car, custom
from openpilot.common.conversions import Conversions as CV
from panda import Panda

from openpilot.selfdrive.car import create_button_events, get_safety_config
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.selfdrive.car.chery.values import CAR, CanBus, CarControllerParams
from openpilot.common.params import Params

ButtonType = car.CarState.ButtonEvent.Type
TransmissionType = car.CarParams.TransmissionType
GearShifter = car.CarState.GearShifter
EventName = car.CarEvent.EventName
# BUTTONS_DICT = {CruiseButtons.RES_ACCEL: ButtonType.accelCruise, CruiseButtons.DECEL_SET: ButtonType.decelCruise,
#                 CruiseButtons.MAIN: ButtonType.altButton3, CruiseButtons.CANCEL: ButtonType.cancel}

CRUISE_OVERRIDE_SPEED_MIN = 5 * CV.KPH_TO_MS

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)

    self.dp_cruise_speed = 0. # km/h
    self.dp_override_speed_last = 0. # km/h
    self.dp_override_speed = 0. # m/s

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "chery"

    CAN = CanBus(fingerprint=fingerprint)
    cfgs = [get_safety_config(car.CarParams.SafetyModel.cheryCanFd)]
    if CAN.main >= 4:
      cfgs.insert(0, get_safety_config(car.CarParams.SafetyModel.elm327))
    ret.safetyConfigs = cfgs

    ret.radarUnavailable = True

    ret.experimentalLongitudinalAvailable = True
    if experimental_long:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_CHERY_LONG_CONTROL
      ret.openpilotLongitudinalControl = True

    ret.pcmCruise = not ret.openpilotLongitudinalControl

    ret.wheelbase = 2.63
    ret.tireStiffnessFactor = 0.8
    ret.centerToFront = ret.wheelbase * 0.4

    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.2
    ret.steerControlType = car.CarParams.SteerControlType.angle

    ret.transmissionType = TransmissionType.automatic

    ret.stopAccel = CarControllerParams.ACCEL_MIN
    ret.stoppingDecelRate = 0.5
    ret.vEgoStarting = 0.1
    ret.vEgoStopping = 0.25
    # ret.longitudinalActuatorDelay = 0.5 # s
    # ret.startAccel = 1.0

    ret.enableBsm = 0x4B1 in fingerprint[CAN.main] and 0x4B3 in fingerprint[CAN.main]

    ret.minEnableSpeed = -1
    ret.minSteerSpeed = -1

    return ret

  # returns a car.CarState
  def _update(self, c):

    ret = self.CS.update(self.cp, self.cp_cam, self.cp_loopback)

    self.CS.button_events = [
      *self.CS.button_events,
      *create_button_events(self.CS.distance_button, self.CS.prev_distance_button, {1: ButtonType.gapAdjustCruise})
    ]
    self.CS.mads_enabled = self.get_sp_cruise_main_state(ret, self.CS)


    self.CS.accEnabled = self.get_sp_v_cruise_non_pcm_state(ret, self.CS.accEnabled,
                                                            self.CS.button_events, c.vCruise,
                                                            enable_buttons=(ButtonType.setCruise, ButtonType.resumeCruise))

    if ret.cruiseState.available:
      if self.enable_mads:
        if not self.CS.prev_mads_enabled and self.CS.mads_enabled:
          self.CS.madsEnabled = True
        self.CS.madsEnabled = self.get_acc_mads(ret.cruiseState.enabled, self.CS.accEnabled, self.CS.madsEnabled)
    else:
      self.CS.madsEnabled = False
    self.CS.madsEnabled = self.get_sp_started_mads(ret, self.CS)

    if not self.CP.pcmCruise or (self.CP.pcmCruise and self.CP.minEnableSpeed > 0) or not self.CP.pcmCruiseSpeed:
      if any(b.type == ButtonType.cancel for b in self.CS.button_events):
        self.CS.madsEnabled, self.CS.accEnabled = self.get_sp_cancel_cruise_state(self.CS.madsEnabled)
    if self.get_sp_pedal_disengage(ret):
      self.CS.madsEnabled, self.CS.accEnabled = self.get_sp_cancel_cruise_state(self.CS.madsEnabled)
      ret.cruiseState.enabled = ret.cruiseState.enabled if not self.enable_mads else False if self.CP.pcmCruise else self.CS.accEnabled

    if self.CP.pcmCruise and self.CP.minEnableSpeed > 0 and self.CP.pcmCruiseSpeed:
      if ret.gasPressed and not ret.cruiseState.enabled:
        self.CS.accEnabled = False
      self.CS.accEnabled = ret.cruiseState.enabled or self.CS.accEnabled

    ret, self.CS = self.get_sp_common_state(ret, self.CS,
                                            gap_button=any(b.type == ButtonType.gapAdjustCruise and b.pressed for b in self.CS.button_events))

    ret.buttonEvents = [
      *self.CS.button_events,
      *self.button_events.create_mads_event(self.CS.madsEnabled, self.CS.out.madsEnabled)  # MADS BUTTON
    ]

    events = self.create_common_events(ret, c, extra_gears=[GearShifter.eco, GearShifter.sport],
                                       pcm_enable=False,
                                       enable_buttons=(ButtonType.setCruise, ButtonType.resumeCruise))

    events, ret = self.create_sp_events(self.CS, ret, events,
                                        enable_buttons=(ButtonType.setCruise, ButtonType.resumeCruise))


    ret.events = events.to_msg()
    return ret
