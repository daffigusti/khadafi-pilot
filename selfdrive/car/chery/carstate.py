import copy
from cereal import car, custom
from collections import deque
import cereal.messaging as messaging

from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import mean
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.interfaces import CarStateBase
from openpilot.selfdrive.car.chery.values import DBC, CanBus, CarControllerParams
from openpilot.common.params import Params

GearShifter = car.CarState.GearShifter

TransmissionType = car.CarParams.TransmissionType
NetworkLocation = car.CarParams.NetworkLocation
STANDSTILL_THRESHOLD = 10 * 0.0311 * CV.KPH_TO_MS

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.frame = 0
    self.angleSensorLast = 0
    self.direction= 1
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.params = CarControllerParams(CP)
    self.shifter_values = can_define.dv["ENGINE_DATA"]["GEAR"]
    self.prev_distance_button = 0
    self.distance_button = 0
    self.prev_main_button = 0
    self.main_button = 0
    self.button_states = {button.event_type: False for button in self.params.BUTTONS}
    self.lkas_enabled = False
    self.prev_lkas_enabled = False
    self.mainEnabled = False


  def create_button_events(self, pt_cp, buttons):
    button_events = []

    for button in buttons:
      state = pt_cp.vl[button.can_addr][button.can_msg] in button.values

      if self.button_states[button.event_type] != state:
        event = car.CarState.ButtonEvent.new_message()
        event.type = button.event_type
        event.pressed = state
        button_events.append(event)
      self.button_states[button.event_type] = state
    return button_events

  def update(self, pt_cp, cam_cp, loopback_cp):
    ret = car.CarState.new_message()

    # car speed
    ret.wheelSpeeds = self.get_wheel_speeds(
      pt_cp.vl["WHEEL_SPEED_FRNT"]["WHEEL_SPEED_FR"],
      pt_cp.vl["WHEEL_SPEED_FRNT"]["WHEEL_SPEED_FL"],
      pt_cp.vl["WHEEL_SPEED_REAR"]["WHEEL_SPEED_RR"],
      pt_cp.vl["WHEEL_SPEED_REAR"]["WHEEL_SPEED_RL"],
    )

    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])  * self.params.HUD_MULTIPLIER
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.vEgoCluster = ret.vEgo
    ret.standstill = ret.vEgoRaw < 0.1

    self.acc_md = copy.copy(cam_cp.vl["ACC_CMD"])
    self.lkas = copy.copy(pt_cp.vl["LKAS"])
    self.lkas_state = copy.copy(cam_cp.vl["LKAS_STATE"])
    self.lkas_cmd = copy.copy(cam_cp.vl["LKAS_CAM_CMD_345"])

    # steer_angle = pt_cp.vl["STEER_SENSOR"]["ANGLE"]
    # steer_angle_fraction = pt_cp.vl["STEER_SENSOR"]["FRACTION"]

    # gas pedal
    self.gasPos = pt_cp.vl["ENGINE_DATA"]["GAS_POS"]
    ret.gas = 0 if self.gasPos >= 2559 or self.gasPos<=0 else self.gasPos
    # ret.gasPressed = ret.gas > 1
    ret.gasPressed = (cam_cp.vl["ACC_CMD"]["GAS_PRESSED"]==1) if (cam_cp.vl["ACC"]["ACC_ACTIVE"] != 0) else (ret.gas > 1)

    # brake pedal
    ret.brake = pt_cp.vl["BRAKE_DATA"]["BRAKE_POS"]
    ret.brakePressed = pt_cp.vl["ENGINE_DATA"]["BRAKE_PRESS"] != 0

    # gear
    # ret.gearShifter = GearShifter.drive
    gear = self.shifter_values.get(pt_cp.vl["ENGINE_DATA"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(gear)
    # button presses
    ret.leftBlinker = pt_cp.vl["BCM_SIGNAL_1"]["SIGN_SIGNAL"] == 2
    ret.rightBlinker = pt_cp.vl["BCM_SIGNAL_1"]["SIGN_SIGNAL"] == 1

    # steering wheel
    self.agleSensor = pt_cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"]/10

    if  (self.frame  % 10) == 0:
      if(self.agleSensor<self.angleSensorLast):
        self.direction = -1
      else:
        self.direction = 1
      self.angleSensorLast = self.agleSensor

    # ret.steeringAngleDeg = (int(steer_angle_fraction) << 8) + steer_angle - 2048
    ret.steeringAngleDeg = self.agleSensor

    ret.steeringTorque = pt_cp.vl["STEER_SENSOR_2"]["TORQUE_DRIVER"] * self.direction
    ret.steeringPressed = abs(ret.steeringTorque) > self.params.STEER_THRESHOLD

    self.steerTemporaryUnvailable = False
    self.button_events = self.create_button_events(pt_cp, self.params.BUTTONS)
    # cruise state
    # ret.cruiseState.available = cam_cp.vl["ACC_CMD"]["ACC_STATE"] != 1 or cam_cp.vl["ACC"]["ACC_ACTIVE"] != 0
    # ret.cruiseState.available =  cam_cp.vl["ACC"]["ACC_ACTIVE"] != 0
    # ret.cruiseState.available =  True
    ret.cruiseState.available = cam_cp.vl["ACC_CMD"]["ACC_STATE"] != 1
    ret.cruiseState.enabled = cam_cp.vl["ACC"]["ACC_ACTIVE"] != 0 or cam_cp.vl["ACC_CMD"]["STOPPED"] == 1
    ret.cruiseState.speed = cam_cp.vl["SETTING"]["CC_SPEED"]
    # ret.cruiseState.enabled = cam_cp.vl["LKAS_STATE"]["STATE"] != 0
    self.cruise_decreased_previously = self.cruise_decreased
    self.cruise_decreased = pt_cp.vl["STEER_BUTTON"]["RES_MINUS"]
    self.cruise_increased_previously = self.cruise_increased
    self.cruise_increased = pt_cp.vl["STEER_BUTTON"]["RES_PLUS"]

    self.prev_distance_button = self.distance_button
    self.distance_button = pt_cp.vl["STEER_BUTTON"]["GAP_ADJUST_UP"]
    self.prev_main_button = self.main_button
    self.main_button = pt_cp.vl["STEER_BUTTON"]["ACC"]

    self.buttons_stock_values = pt_cp.vl["STEER_BUTTON"]
    # FrogPilot CarState functions
    self.lkas_previously_enabled = self.lkas_enabled
    self.lkas_enabled = cam_cp.vl["LKAS_STATE"]["LKA_ACTIVE"] != 0

    # blindspot sensors
    if self.CP.enableBsm:
      ret.leftBlindspot = pt_cp.vl["BSM_LEFT"]["BSM_LEFT_DETECT"] != 0
      ret.rightBlindspot = pt_cp.vl["BSM_RIGHT"]["BSM_RIGHT_DETECT"] != 0

    # lock info
    ret.doorOpen = False
    ret.seatbeltUnlatched = False

    ret.brakeLightsDEPRECATED = bool(ret.brakePressed)

    if self.CP.openpilotLongitudinalControl:
          if self.prev_main_button != 1:
            if self.main_button == 1:
              self.mainEnabled = not self.mainEnabled
          ret.cruiseState.available = ret.cruiseState.available and self.mainEnabled
    self.prev_mads_enabled = self.mads_enabled
    self.prev_lkas_enabled = self.lkas_enabled

    self.mads_enabled = False if not self.control_initialized else ret.cruiseState.available

    # print('Steer Fraction: ', steer_angle_fraction)
    # print('retsteeringTorque: ', ret.steeringTorque)
    # print('brakePressed: ', ret.brakePressed)
    # print('agle sensor 1: ', ret.steeringAngleDeg)
    # print('agle sensor 2: ', self.agleSensor)
    # print('Steer Sensor Torque: ', ret.steeringTorque)
    # print('Engine: ', pt_cp.vl["ENGINE_DATA"])

    self.frame += 1
    return ret

  @staticmethod
  def get_cam_can_parser(CP):
    messages = [
      ("ACC_CMD", 50),
      ("ACC", 50),
      ("LKAS_CAM_CMD_345", 50),
      ("LKAS_STATE", 20),
      ("SETTING", 20),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CanBus(CP).camera)

  @staticmethod
  def get_can_parser(CP):

    messages = [
       ("STEER_ANGLE_SENSOR", 100),
       ("STEER_SENSOR", 100),
       ("WHEEL_SPEED_FRNT", 50),
       ("WHEEL_SPEED_REAR", 50),
       ("BCM_SIGNAL_1", 50),
       ("BCM_SIGNAL_2", 50),
       ("BRAKE_DATA", 50),
       ("LKAS", 100),
       ("ENGINE_DATA", 100),
       ("STEER_SENSOR_2", 100),
       ("STEER_BUTTON", 20),

    ]
    print('CanBus Main: ', CanBus(CP).main)
    print('CanBus Cam: ', CanBus(CP).camera)
    print('Enable BSM: ', CP.enableBsm)

    if CP.enableBsm:
      messages += [
        ("BSM_LEFT", 10),
        ("BSM_RIGHT", 10),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CanBus(CP).main)


