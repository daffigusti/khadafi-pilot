from dataclasses import dataclass, field
from enum import Enum, IntFlag
from collections import defaultdict, namedtuple

from opendbc.car import Bus, DT_CTRL, PlatformConfig, DbcDict, Platforms, CarSpecs, structs
from opendbc.car.structs import CarParams
from opendbc.car.lateral import AngleSteeringLimits

from opendbc.car.docs_definitions import CarDocs, CarFootnote, CarHarness, CarParts, Column
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu
Button = namedtuple('Button', ['event_type', 'can_addr', 'can_msg', 'values'])

class CarControllerParams:
  STEER_STEP = 2
  LKAS_HUD_STEP = 5
  BUTTONS_STEP = 5
  ACC_CONTROL_STEP = 2

  HUD_MULTIPLIER = 1
  STEER_DRIVER_MULTIPLIER = 3              # weight driver torque heavily
  STEER_DRIVER_FACTOR = 1                  # from dbc

  STEER_DELTA_UP = 2
  STEER_DELTA_DOWN = 3

  STEER_THRESHOLD = 70               # Nm, threshold for steering torque to be considered active
  STEER_DRIVER_ALLOWANCE = 15    # Driver intervention threshold, Nm

  # Temporary steer fault timeout
  STEER_TIMEOUT = 30 / DT_CTRL

  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    # When output steering Angle not within range -1311 and 1310,
    #   CANPacker packs wrong angle output to be decoded by panda
    300,  # deg, reasonable limit
    ([], []),  # rate_up_bp, rate_up_v - handled by custom angle limits
    ([], []),  # rate_down_bp, rate_down_v - handled by custom angle limits
  )

  ACCEL_MAX = 2.0               # m/s^2 max acceleration
  ACCEL_MAX_PLUS = 4.0          # m/s^2 max acceleration
  ACCEL_MIN = -3.5              # m/s^2 max deceleration
  MIN_GAS = -24
  INACTIVE_GAS = -24

  GAS_MAX = 511
  GAS_MIN = -511

  ACCEL_LOOKUP_BP = [ACCEL_MIN, 0, ACCEL_MAX]
  ACCEL_LOOKUP_V = [GAS_MIN, -24, GAS_MAX]

  # Smoothing parameters for angle control based on vehicle speed
  SMOOTHING_ANGLE_VEGO_MATRIX = [0, 8.5, 11, 13.8, 22.22]
  SMOOTHING_ANGLE_ALPHA_MATRIX = [0.05, 0.1, 0.3, 0.6, 1]

  def __init__(self, CP):
    self.BUTTONS = [
      Button(structs.CarState.ButtonEvent.Type.setCruise, "STEER_BUTTON", "ACC", [1]),
      Button(structs.CarState.ButtonEvent.Type.resumeCruise, "STEER_BUTTON", "RES_PLUS", [1]),
      Button(structs.CarState.ButtonEvent.Type.accelCruise, "STEER_BUTTON", "RES_PLUS", [1]),
      Button(structs.CarState.ButtonEvent.Type.decelCruise, "STEER_BUTTON", "RES_MINUS", [1]),
      # Button(car.CarState.ButtonEvent.Type.cancel, "STEER_BUTTON", "ACC", [1]),
      Button(structs.CarState.ButtonEvent.Type.gapAdjustCruise, "STEER_BUTTON", "GAP_ADJUST_UP", [1]),
      # Button(structs.CarState.ButtonEvent.Type.gapAdjustCruise, "STEER_BUTTON", "GAP_ADJUST_DOWN", [1]),
    ]

class CherySafetyFlags(IntFlag):
  LONG_CONTROL = 1
  CANFD = 2

class CheryFlags(IntFlag):
  # Static flags
  CANFD = 1

class CanBus:
  main = 0
  alt = 1
  camera = 2
  loopback = 128

@dataclass
class CheryCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))

  def init(self):
    super().init()
    self.flags |= CheryFlags.CANFD

@dataclass(frozen=True)
class CheryCarSpecs(CarSpecs):
  centerToFrontRatio: float = 0.44
  steerRatio: float = 17.

@dataclass
class CheryPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: "chery_canfd"})


class CAR(Platforms):
  CHERY_OMODA_E5 = CheryPlatformConfig(
    [CheryCarDocs("Chery Omoda E5", video="https://youtu.be/9kGGh8sLcHc")],
    CheryCarSpecs(mass=1785, wheelbase=2.63, steerRatio=17.5)
  )


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    # TODO: check data to ensure ABS does not skip ISO-TP frames on bus 0
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    )
  ],
)

DBC = CAR.create_dbc_map()
