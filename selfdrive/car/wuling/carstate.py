import copy
from cereal import car
from collections import deque

from common.conversions import Conversions as CV
from common.numpy_fast import mean
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.wuling.values import DBC, AccState, CanBus, CAR, PREGLOBAL_CARS,CarControllerParams

TransmissionType = car.CarParams.TransmissionType
NetworkLocation = car.CarParams.NetworkLocation
STANDSTILL_THRESHOLD = 10 * 0.0311 * CV.KPH_TO_MS

TORQUE_SAMPLES = 12

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["ECMPRDNL"]["TRANSMISSION_STATE"]
    self.cluster_speed_hyst_gap = CV.KPH_TO_MS / 2.
    self.cluster_min_speed = CV.KPH_TO_MS / 2.
    self.is_cruise_latch = False

    self.lka_steering_cmd_counter = 0

    self.loopback_lka_steering_cmd_updated = False
    self.loopback_lka_steering_cmd_ts_nanos = 0
    self.pt_lka_steering_cmd_counter = 0
    self.cam_lka_steering_cmd_counter = 0
    self.buttons_counter = 0
    self.engineRPM = 0
    self.cruise_speed = 30 * CV.KPH_TO_MS
    self.resume_alert = False

    self.lkas_enabled = False
    self.prev_lkas_enabled = False

    self.crz_btns_counter = 0
    self.is_cruise_latch = False
    self.params = CarControllerParams(CP)



  def update(self, pt_cp, cam_cp, loopback_cp):
    ret = car.CarState.new_message()

    self.prev_cruise_buttons = self.cruise_buttons
    self.cruise_buttons = pt_cp.vl["STEER_BTN"]["ACC_BTN_1"]
    self.buttons_counter = pt_cp.vl["STEER_BTN"]["COUNTER_1"]

    self.engineRPM = pt_cp.vl["ECMEngineStatus"]['EngineRPM']
    # self.prev_mads_enabled = self.mads_enabled
    # self.prev_lkas_enabled = self.lkas_enabled

   # Variables used for avoiding LKAS faults
    self.loopback_lka_steering_cmd_updated = len(loopback_cp.vl_all["STEERING_LKA"]["COUNTER"]) > 0
    if self.loopback_lka_steering_cmd_updated:
      self.loopback_lka_steering_cmd_ts_nanos = loopback_cp.ts_nanos["STEERING_LKA"]["COUNTER"]
    if self.CP.networkLocation == NetworkLocation.fwdCamera:
      self.pt_lka_steering_cmd_counter = pt_cp.vl["STEERING_LKA"]["COUNTER"]
      self.cam_lka_steering_cmd_counter = cam_cp.vl["STEERING_LKA"]["COUNTER"]


    ret.wheelSpeeds = self.get_wheel_speeds(
      pt_cp.vl["EBCMWheelSpdFront"]["FLWheelSpd"],
      pt_cp.vl["EBCMWheelSpdFront"]["FLWheelSpd"],
      pt_cp.vl["EBCMWheelSpdRear"]["RLWheelSpd"],
      pt_cp.vl["EBCMWheelSpdRear"]["RLWheelSpd"],
    )

    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])  * self.params.HUD_MULTIPLIER
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.vEgoCluster = ret.vEgo
    # sample rear wheel speeds, standstill=True if ECM allows engagement with brake
    # ret.standstill = ret.wheelSpeeds.rl <= STANDSTILL_THRESHOLD and ret.wheelSpeeds.rr <= STANDSTILL_THRESHOLD
    ret.standstill = ret.vEgoRaw < 0.1

    ret.steeringAngleDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelAngle"]
    ret.steeringTorque = pt_cp.vl["PSCMSteeringAngle"]["SteeringTorque"]
    ret.steeringTorqueEps = pt_cp.vl["STEER_RELATED"]["STEER_TORQUE"]

    ret.seatbeltUnlatched = pt_cp.vl["BCMDoorBelt"]["RIGHTSEATBEALT"] == 0
    ret.doorOpen = (pt_cp.vl["BCMDoorBeltStatus"]["FrontLeftDoor"] == 1 or
                pt_cp.vl["BCMDoorBeltStatus"]["FrontRightDoor"] == 1 or
                pt_cp.vl["BCMDoorBeltStatus"]["RearLeftDoor"] == 1 or
                pt_cp.vl["BCMDoorBeltStatus"]["RearRightDoor"] == 1)

    ret.brake = pt_cp.vl["BRAKE_PEDAL"]["BRAKE_POS"]

    ret.brakePressed = pt_cp.vl["ECMEngineStatus"]["Brake_Pressed"] != 0
    ret.brakeHoldActive = pt_cp.vl["EPBStatus"]["AVH_STATUS"] != 0

    # ret.leftBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 1
    # ret.rightBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 2

    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(40,  pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 1, pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 2)

    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["ECMPRDNL"]["TRANSMISSION_STATE"], None))

    # print('Gear Shifter :  %s' % ret.gearShifter)
    ret.gas = pt_cp.vl["GAS_PEDAL"]["GAS_POS"]
    ret.gasPressed = ret.gas > 0

    ret.parkingBrake = bool(pt_cp.vl["EPBStatus"]["EPBSTATUS"])
    self.park_brake = pt_cp.vl["EPBStatus"]["EPBSTATUS"]
    self.pcm_acc_status = pt_cp.vl["ASCMActiveCruiseControlStatus"]["ACCSTATE"]

    ret.cruiseState.available = pt_cp.vl["AccStatus"]["CruiseMainOn"] != 0 or pt_cp.vl["AccStatus"]["CruiseState"] != 0
    ret.cruiseState.enabled = pt_cp.vl["AccStatus"]["CruiseState"] != 0

    self.is_cruise_latch = pt_cp.vl["AccStatus"]["CruiseMainOn"] != 0 or pt_cp.vl["AccStatus"]["CruiseState"] != 0

    if pt_cp.vl["AccStatus"]["CruiseMainOn"] != 0 and ret.brakePressed:
      self.is_cruise_latch = False
    else:
      pt_cp.vl["AccStatus"]["CruiseMainOn"] != 0 and not ret.brakePressed
      self.is_cruise_latch = True

    if not ret.cruiseState.available:
      self.is_cruise_latch = False


    # ret.cruiseState.enabled = self.is_cruise_latch

    self.resume_alert = pt_cp.vl["ASCMActiveCruiseControlStatus"]["ACCResumeAlert"]

    ret.cruiseState.speed = pt_cp.vl["ASCMActiveCruiseControlStatus"]["ACCSpeedSetpoint"] * CV.KPH_TO_MS
    # ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > self.params.STEER_THRESHOLD, 5)

    ret.genericToggle = bool(pt_cp.vl["BCMTurnSignals"]["HighBeamsActive"])

    ret.cruiseState.standstill = ret.cruiseState.enabled == 0 and ret.cruiseState.available != 0
    self.lkas_status = 0
    self.crz_btns_counter = pt_cp.vl["ASCMActiveCruiseControlStatus"]["COUNTER_1"];
    ret.brakeLights = bool(ret.brakePressed or ret.brakeHoldActive)

    # self.steeringTorqueSamples.append(ret.steeringTorque)
    # if ret.steeringPressed:
    #   print("Steering pressed")
    # print('Cruise speed :  %s' % ret.cruiseState.speed)
    # print('Cruise state enable :  %s' % ret.cruiseState.enabled)
    # print('Cruise state available :  %s' % ret.cruiseState.available)

    # print('Steering Torque EPS :  %d' %  ret.steeringTorqueEps)
    #trans state 15 "PARKING" 1 "DRIVE" 14 "BACKWARD" 13 "NORMAL"
    return ret

  @staticmethod
  def get_cam_can_parser(CP):
    signals = []
    checks = []
    if CP.networkLocation == NetworkLocation.fwdCamera:
      signals += [
        ("CHECKSUM", "STEERING_LKA"),
        ("STEER_REQUEST", "STEERING_LKA"),
        ("STEER_TORQUE_CMD", "STEERING_LKA"),
        ("COUNTER", "STEERING_LKA"),
      ]
      checks += [
        ("STEERING_LKA", 50),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.CAMERA)

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address
     ("TurnSignals", "BCMTurnSignals"),
      ("HighBeamsActive", "BCMTurnSignals"),
      ("SteeringWheelAngle", "PSCMSteeringAngle"),
      ("SteeringWheelRate", "PSCMSteeringAngle"),
      ("SteeringTorque", "PSCMSteeringAngle"),

      ("FLWheelSpd", "EBCMWheelSpdFront"),
      ("FRWheelSpd", "EBCMWheelSpdFront"),
      ("RLWheelSpd", "EBCMWheelSpdRear"),
      ("RRWheelSpd", "EBCMWheelSpdRear"),
      ("EngineRPM", "ECMEngineStatus"),

      ("FrontLeftDoor", "BCMDoorBeltStatus"),
      ("FrontRightDoor", "BCMDoorBeltStatus"),
      ("RearLeftDoor", "BCMDoorBeltStatus"),
      ("RearRightDoor", "BCMDoorBeltStatus"),
      ("LeftSeatBelt", "BCMDoorBeltStatus"),
      ("RightSeatBelt", "BCMDoorBeltStatus"),
      ("RIGHTSEATBEALT", "BCMDoorBelt"),
      ("LEFTSEATBEALT", "BCMDoorBelt"),
      ("EPBClosed", "EPBStatus"),
      ("Brake_Pressed", "ECMEngineStatus"),
      ("EPBSTATUS", "EPBStatus"),
      ("AVH_STATUS", "EPBStatus"),

      ("ACCBUTTON", "ASCMActiveCruiseControlStatus"),
      ("ACCSTATE", "ASCMActiveCruiseControlStatus"),
      ("ACCSpeedSetpoint", "ASCMActiveCruiseControlStatus"),
      ("ACCResumeAlert", "ASCMActiveCruiseControlStatus"),
      ("COUNTER_1", "ASCMActiveCruiseControlStatus"),

      ("TRANSMISSION_STATE", "ECMPRDNL"),
      # ("LKAS_STATE", "LkasHud"),
      # ("LKA_ACTIVE", "LkasHud"),
      ("CruiseMainOn", "AccStatus"),
      ("CruiseState", "AccStatus"),
      ("STEER_TORQUE", "STEER_RELATED"),
      ("GAS_POS", "GAS_PEDAL"),
      ("BRAKE_POS", "BRAKE_PEDAL"),

      ("ACC_BTN_1", "STEER_BTN"),
      ("COUNTER_1", "STEER_BTN"),
    ]

    checks = [
      ("ECMEngineStatus", 10),
      ("EPBStatus", 10),
      ("ECMPRDNL", 10),
      ("BCMDoorBeltStatus", 10),
      ("BCMDoorBelt", 10),
      ("EBCMWheelSpdFront", 20),
      ("EBCMWheelSpdRear", 20),
      ("PSCMSteeringAngle", 100),
      ("ASCMActiveCruiseControlStatus", 10),
      # ("LkasHud", 20),
      ("AccStatus", 20),
      ("STEER_RELATED", 20),
      ("GAS_PEDAL", 10),
      ("BRAKE_PEDAL", 50),
      ("BCMTurnSignals", 30),
      ("STEER_BTN", 50),

    ]

     # Used to read back last counter sent to PT by camera
    if CP.networkLocation == NetworkLocation.fwdCamera:
      signals += [
        ("COUNTER", "STEERING_LKA"),
      ]
      checks += [
        ("STEERING_LKA", 0),
      ]


    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.POWERTRAIN)

  @staticmethod
  def get_loopback_can_parser(CP):
    signals = [
      ("COUNTER", "STEERING_LKA"),
    ]

    checks = [
      ("STEERING_LKA", 0),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.LOOPBACK, enforce_checks=False)
