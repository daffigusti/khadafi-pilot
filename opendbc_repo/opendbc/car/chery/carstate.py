import copy

from opendbc.car.common.conversions import Conversions as CV
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.chery.values import DBC, CarControllerParams
from opendbc.car.chery.cherycan import CanBus
from opendbc.sunnypilot.car.chery.mads import MadsCarState

ButtonType = structs.CarState.ButtonEvent.Type

class CarState(CarStateBase, MadsCarState):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    MadsCarState.__init__(self, CP, CP_SP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.params = CarControllerParams(CP)

    self.shifter_values = can_define.dv["ENGINE_DATA"]["GEAR"]
    self.button_states = {button.event_type: False for button in self.params.BUTTONS}

    self.frame = 0
    self.angleSensorLast = 0
    self.direction = 1
    self.prev_distance_button = 0
    self.distance_button = 0
    self.cruise_decreased = 0
    self.cruise_increased = 0
    self.prev_main_button = 0
    self.lkas_status = 0
    self.main_button = 0
    self.lkas_enabled = False
    self.prev_lkas_enabled = False
    self.mainEnabled = False
    self.mads_enabled = False
    self.last_change_time = 0.0

    # Detect if servo stop responding to steering command.
    self.cruiseState_enabled_prev = False
    self.eps_torque_timer = 0

  def update_button_enable(self, buttonEvents: list[structs.CarState.ButtonEvent]):
    if not self.CP.pcmCruise:
      for b in buttonEvents:
        # Enable OP long on falling edge of enable buttons
        if b.type in (ButtonType.setCruise, ButtonType.resumeCruise) and not b.pressed:
          return True
    return False
  def create_button_events(self, cp, buttons):
    button_events = []

    for button in buttons:
      state = cp.vl[button.can_addr][button.can_msg] in button.values

      if self.button_states[button.event_type] != state:
        event = structs.CarState.ButtonEvent.new_message()
        event.type = button.event_type
        event.pressed = state
        button_events.append(event)
      self.button_states[button.event_type] = state
    return button_events

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    loopback_cp = can_parsers[Bus.loopback]

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()
    # car speed
    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEED_FRNT"]["WHEEL_SPEED_FR"],
      cp.vl["WHEEL_SPEED_FRNT"]["WHEEL_SPEED_FL"],
      cp.vl["WHEEL_SPEED_REAR"]["WHEEL_SPEED_RR"],
      cp.vl["WHEEL_SPEED_REAR"]["WHEEL_SPEED_RL"],
    )
    ret.standstill = ret.vEgoRaw < 1e-3

    self.acc_md = copy.copy(cp_cam.vl["ACC_CMD"])
    self.lkas = copy.copy(cp.vl["LKAS"])
    self.lkas_state = copy.copy(cp_cam.vl["LKAS_STATE"])  # Read from camera bus (stock ECU)
    self.setting = copy.copy(cp_cam.vl["SETTING"])
    self.lkas_cmd = copy.copy(cp_cam.vl["LKAS_CAM_CMD_345"])


    # Debug: List all camera bus messages (print once at frame 100)
    if self.frame == 100:
      print(f"[CHERY DEBUG] Frame {self.frame}: ACC_CMD={self.acc_md}, LKAS={self.lkas}, LKAS_STATE={self.lkas_state}, SETTING={self.setting}, LKAS_CAM_CMD_345={self.lkas_cmd}")
      print(f"[CHERY DEBUG] === Camera bus (bus 2) messages received ===")
      print(f"[CHERY DEBUG] message_states keys type: {type(list(cp_cam.message_states.keys())[0]) if cp_cam.message_states else 'empty'}")
      print(f"[CHERY DEBUG] Total message_states: {len(cp_cam.message_states)}")
      for msg_name, state in cp_cam.message_states.items():
        if state.timestamps:
          print(f"[CHERY DEBUG]   ✓ '{msg_name}' (type={type(msg_name).__name__}): {len(state.timestamps)} msgs, freq={state.frequency:.1f}Hz")
        else:
          print(f"[CHERY DEBUG]   ✗ '{msg_name}': NEVER received")

      # Check specifically for LKAS_STATE by name
      print(f"[CHERY DEBUG] Checking 'LKAS_STATE' directly: {cp_cam.message_states.get('LKAS_STATE')}")
      print(f"[CHERY DEBUG] Checking '775' by ID: {cp_cam.message_states.get('775')}")
      print(f"[CHERY DEBUG] Checking 775 (int): {cp_cam.message_states.get(775)}")

    # Debug: Check LKAS_STATE message validity from CAMERA BUS (print every 100 frames to reduce spam)
    if self.frame % 100 == 0:
      # message_states is keyed by ID (int), not name (str)
      lkas_state_valid = cp_cam.message_states.get(775)  # Use ID 775, not name
      if lkas_state_valid:
        print(f"[CHERY DEBUG] Frame {self.frame}: LKAS_STATE (775) valid={lkas_state_valid.valid(cp_cam._last_update_nanos, cp_cam.bus_timeout)}, "
              f"freq={lkas_state_valid.frequency:.1f}Hz, "
              f"LKA_ACTIVE={self.lkas_state.get('LKA_ACTIVE', 'N/A')}")
      else:
        print(f"[CHERY DEBUG] Frame {self.frame}: LKAS_STATE (775) message_states not found!")

      # Verify vl["LKAS_STATE"] works for accessing values
      try:
        lka_val = cp_cam.vl["LKAS_STATE"]["LKA_ACTIVE"]
        print(f"[CHERY DEBUG] Frame {self.frame}: vl['LKAS_STATE']['LKA_ACTIVE'] = {lka_val} ✅")
      except Exception as e:
        print(f"[CHERY DEBUG] Frame {self.frame}: vl['LKAS_STATE'] ERROR: {e}")

      # Debug: Check ACC_ACTIVE bit that should set controls_allowed in panda
      acc_active = cp_cam.vl["ACC"]["ACC_ACTIVE"]
      print(f"[CHERY DEBUG] Frame {self.frame}: ACC['ACC_ACTIVE'] bit (should trigger controls_allowed in panda) = {acc_active}")
      print(f"[CHERY DEBUG] Frame {self.frame}: CANParser camera valid={cp_cam.can_valid}, bus_timeout={cp_cam.bus_timeout}")

    # gas pedal
    self.gasPos = cp.vl["ENGINE_DATA"]["GAS"]
    ret.gasPressed = (cp_cam.vl["ACC_CMD"]["GAS_PRESSED"]==1) if (cp_cam.vl["ACC"]["ACC_ACTIVE"] != 0) else (self.gasPos > 1)

    # brake pedal
    ret.brake = cp.vl["BRAKE_DATA"]["BRAKE_POS"]
    ret.brakePressed = cp.vl["ENGINE_DATA"]["BRAKE_PRESS"] != 0

    # gear
    can_gear = int(cp.vl["ENGINE_DATA"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    # button presses
    ret.leftBlinker = cp.vl["BCM_SIGNAL_1"]["SIGN_SIGNAL"] == 2
    ret.rightBlinker = cp.vl["BCM_SIGNAL_1"]["SIGN_SIGNAL"] == 1
    ret.stockAeb = cp_cam.vl["ACC"]["AEB_ACTIVE"] == 1

    # steering wheel
    self.angleSensor = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"]

    if (self.frame % 10) == 0:
      if self.angleSensor < self.angleSensorLast:
        self.direction = -1
      else:
        self.direction = 1
      self.angleSensorLast = self.angleSensor

    ret.steeringAngleDeg = self.angleSensor

    ret.steeringTorque = cp.vl["STEER_SENSOR_2"]["TORQUE_DRIVER"] * self.direction

    ret.steeringTorqueEps = cp.vl["STEER_ANGLE_SENSOR"]['TORQUE']

    ret.steeringPressed = abs(ret.steeringTorque) > CarControllerParams.STEER_THRESHOLD

    self.steerTemporaryUnavailable = False
    self.lkas_status_before = self.lkas_status
    self.lkas_status = cp.vl["LKAS"]['NEW_SIGNAL_1']

    if ret.cruiseState.enabled and ret.vEgo > self.CP.minSteerSpeed:
      # Reset counter on entry
      if self.cruiseState_enabled_prev != ret.cruiseState.enabled:
        self.eps_torque_timer = 0
      # Count up when no torque from servo detected.
      if loopback_cp.vl["LKAS_STATE"]['LKA_ACTIVE'] == 1 and cp.vl["LKAS"]['LKAS_CMD'] == -1 and self.lkas_status == 1:
        self.eps_torque_timer += 1
      else:
        self.eps_torque_timer = 0
      # Set fault if above threshold
      ret.steerFaultTemporary = self.eps_torque_timer >= CarControllerParams.STEER_TIMEOUT

    self.cruiseState_enabled_prev = ret.cruiseState.enabled

    # cruise state
    ret.cruiseState.available = True
    ret.cruiseState.enabled = cp_cam.vl["ACC"]["ACC_ACTIVE"] != 0 or cp_cam.vl["ACC_CMD"]["STOPPED"] == 1
    self.lead_front = (cp_cam.vl["LEAD_FRONT"]["LEAD_DISTANCE"]) if (cp_cam.vl["LEAD_FRONT"]["VALID_SIGNAL"] == 1) else 0

    # Debug: Check final cruiseState after calculation (every 100 frames)
    if self.frame % 100 == 0:
      print(f"[CHERY DEBUG] Frame {self.frame}: AFTER calc - cruiseState.enabled = {ret.cruiseState.enabled}, ACC_ACTIVE={cp_cam.vl['ACC']['ACC_ACTIVE']}, STOPPED={cp_cam.vl['ACC_CMD']['STOPPED']}")

    self.needResume = cp_cam.vl["ACC"]["ACC_ACTIVE"] == 0 and cp_cam.vl["ACC_CMD"]["STOPPED"] == 1
    ret.cruiseState.speed = cp_cam.vl["SETTING"]["CC_SPEED"] * CV.KPH_TO_MS
    ret.cruiseState.standstill = ret.standstill

    self.cruise_decreased_previously = self.cruise_decreased
    self.cruise_decreased = cp.vl["STEER_BUTTON"]["RES_MINUS"]
    self.cruise_increased_previously = self.cruise_increased
    self.cruise_increased = cp.vl["STEER_BUTTON"]["RES_PLUS"]

    self.prev_distance_button = self.distance_button
    self.distance_button = cp.vl["STEER_BUTTON"]["GAP_ADJUST_UP"]
    self.prev_main_button = self.main_button
    self.main_button = cp.vl["STEER_BUTTON"]["ACC"]

    self.buttons_stock_values = cp.vl["STEER_BUTTON"]
    # FrogPilot CarState functions
    self.lkas_previously_enabled = self.lkas_enabled
    self.lkas_enabled = cp_cam.vl["LKAS_STATE"]["LKA_ACTIVE"] != 0  # Read from camera bus (stock ECU)
    self.lkas_active =  cp.vl["LKAS"]['LKAS_CMD']
    self.acc_available = cp_cam.vl["SETTING"]["ACC_AVAILABLE"]

    # TODO: get the real value
    ret.stockAeb = False
    ret.stockFcw = False
    # blindspot sensors
    if self.CP.enableBsm:
      ret.leftBlindspot = cp.vl["BSM_LEFT"]["BSM_LEFT_DETECT"] != 0
      ret.rightBlindspot = cp.vl["BSM_RIGHT"]["BSM_RIGHT_DETECT"] != 0

    # TODO: get the real value
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

    self.mads_enabled = ret.cruiseState.available

    ret.buttonEvents = self.create_button_events(cp, self.params.BUTTONS)

    # Update MADS button states
    MadsCarState.update_mads(self, ret, can_parsers)

    self.frame += 1
    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    pt_messages = [
       ("STEER_ANGLE_SENSOR", 100),
       ("STEER_SENSOR", 100),
       ("WHEEL_SPEED_FRNT", 50),
       ("WHEEL_SPEED_REAR", 50),
       ("BCM_SIGNAL_1", 50),
       ("BCM_SIGNAL_2", 50),
       ("BRAKE_DATA", 50),
       ("LKAS", 100),
       ("ENGINE_DATA", 100),
       ("STEER_SENSOR_2", 59),
       ("STEER_BUTTON", 20),
    ]

    if CP.enableBsm:
      pt_messages += [
        ("BSM_LEFT", 10),
        ("BSM_RIGHT", 10),
      ]

    cam_messages = [
      ("ACC_CMD", 50),
      ("ACC", 50),
      ("LKAS_CAM_CMD_345", 50),
      ("LKAS_STATE", 20),  # Stock ECU camera sends at ~20 Hz
      ("SETTING", 20),
      ("LEAD_FRONT", 20),
    ]
    loopback_messages = [
      ("LKAS_STATE", 0),  # Keep loopback for debugging (frequency 0 = no timeout)
    ]

    can_bus = CanBus(CP)

    # Debug: Print bus numbers to verify offset calculation
    print(f"[CHERY DEBUG] Bus offset: {can_bus.offset}")
    print(f"[CHERY DEBUG] Bus numbers - main: {can_bus.main}, camera: {can_bus.camera}, loopback: {can_bus.loopback}")
    print(f"[CHERY DEBUG] Safety configs count: {len(CP.safetyConfigs) if hasattr(CP, 'safetyConfigs') else 'N/A'}")

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, can_bus.main),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, can_bus.camera),
      Bus.loopback: CANParser(DBC[CP.carFingerprint][Bus.pt], loopback_messages, can_bus.loopback),
    }

