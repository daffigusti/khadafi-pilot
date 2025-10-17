#!/usr/bin/env python3
"""
Test suite for Chery vehicle implementation
Tests fingerprints, parameters, longitudinal tuning, and safety features
"""

import pytest
import numpy as np
from typing import Any

from opendbc.car import Bus, structs
from opendbc.car.structs import CarParams
from opendbc.car.chery.fingerprints import FW_VERSIONS, FINGERPRINTS
from opendbc.car.chery.values import CAR, DBC, CarControllerParams, CheryFlags, CherySafetyFlags, CanBus
from opendbc.car.chery.interface import CarInterface
from opendbc.car.chery.carstate import CarState
from opendbc.car.chery.carcontroller import CarController
from opendbc.car.chery.cherycan import calculate_crc
from opendbc.car.fw_versions import build_fw_dict
from opendbc.car.interfaces import get_interface_attr


class TestCheryFingerprint:
    """Tests for Chery fingerprinting and firmware versions"""

    def test_fingerprints_valid(self):
        """Verify all fingerprints have valid structure"""
        for car_model, fingerprints in FINGERPRINTS.items():
            assert car_model in CAR
            for fingerprint in fingerprints:
                # Check that fingerprint is a dictionary
                assert isinstance(fingerprint, dict)
                # Check that all keys are integers (CAN IDs)
                assert all(isinstance(can_id, int) for can_id in fingerprint.keys())
                # Check that all values are integers (message lengths)
                assert all(isinstance(msg_len, int) for msg_len in fingerprint.values())

    def test_fw_versions_format(self):
        """Test firmware version format and structure"""
        for car_model, ecus in FW_VERSIONS.items():
            assert car_model in CAR
            for ecu, fw_versions in ecus.items():
                # Check ECU tuple format (ecu_type, address, sub_address)
                assert len(ecu) == 3
                assert isinstance(ecu[0], CarParams.Ecu)
                assert isinstance(ecu[1], int)  # Address
                assert ecu[2] is None or isinstance(ecu[2], int)  # Sub-address

                # Check firmware versions are bytes
                for fw in fw_versions:
                    assert isinstance(fw, bytes)
                    # Basic check that FW version is not empty
                    assert len(fw) > 0

    def test_dbc_files_exist(self):
        """Verify DBC mappings are valid"""
        for car_model, dbc_dict in DBC.items():
            assert car_model in CAR
            # Chery uses "chery_canfd" as the DBC name
            assert Bus.pt in dbc_dict
            assert dbc_dict[Bus.pt] == "chery_canfd"


class TestCheryInterface:
    """Tests for Chery CarInterface"""

    def test_get_params_returns_valid(self):
        """Test that get_params returns valid CarParams"""
        # Use empty fingerprint and firmware for testing
        fingerprint = {0: {}, 1: {}, 2: {}, 3: {}, 4: {}, 5: {}, 6: {}, 7: {}}
        car_fw = []

        params = CarInterface.get_params(
            CAR.CHERY_OMODA_E5,
            fingerprint,
            car_fw,
            alpha_long=True,
            is_release=False,
            docs=False
        )

        # Basic parameter checks
        assert params.brand == "chery"
        assert params.radarUnavailable == True
        assert params.steerControlType == structs.CarParams.SteerControlType.angle
        assert params.alphaLongitudinalAvailable == True

        # Safety config checks
        assert len(params.safetyConfigs) > 0
        assert params.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.cheryCanFd

    def test_longitudinal_tuning_valid(self):
        """Test longitudinal tuning parameters are valid"""
        fingerprint = {0: {}, 1: {}, 2: {}, 3: {}, 4: {}, 5: {}, 6: {}, 7: {}}
        params = CarInterface.get_params(
            CAR.CHERY_OMODA_E5,
            fingerprint,
            [],
            alpha_long=True,
            is_release=False,
            docs=False
        )

        # Check array lengths match
        assert len(params.longitudinalTuning.kiBP) == len(params.longitudinalTuning.kiV)
        assert len(params.longitudinalTuning.kpBP) == len(params.longitudinalTuning.kpV)

        # Check breakpoints are in ascending order
        if len(params.longitudinalTuning.kiBP) > 1:
            assert all(params.longitudinalTuning.kiBP[i] < params.longitudinalTuning.kiBP[i+1]
                      for i in range(len(params.longitudinalTuning.kiBP)-1))

        # Check gain values are positive and reasonable
        assert all(0 <= ki <= 2.5 for ki in params.longitudinalTuning.kiV)

        # Specific Chery tuning checks
        assert params.longitudinalTuning.kiBP == [0., 5., 35.]
        assert params.longitudinalTuning.kiV == [0.5, 0.4, 0.2]

    def test_init_and_deinit_methods_exist(self):
        """Test that init and deinit methods are defined"""
        assert hasattr(CarInterface, 'init')
        assert hasattr(CarInterface, 'deinit')
        assert callable(CarInterface.init)
        assert callable(CarInterface.deinit)

        # Test they don't raise errors when called
        CarInterface.init(None, None, None, None)
        CarInterface.deinit(None, None, None)

    def test_actuator_limits(self):
        """Test actuator limits are within safe ranges"""
        params = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)
        ctrl_params = CarControllerParams(params)

        # Acceleration limits
        assert -4.0 <= ctrl_params.ACCEL_MIN <= -2.0
        assert 1.0 <= ctrl_params.ACCEL_MAX <= 3.0
        assert ctrl_params.ACCEL_MIN < ctrl_params.ACCEL_MAX

        # Steering angle limits
        assert 0 < ctrl_params.STEER_ANGLE_MAX <= 360

        # Gas/throttle limits
        assert ctrl_params.GAS_MIN < ctrl_params.GAS_MAX
        assert ctrl_params.INACTIVE_GAS <= ctrl_params.GAS_MIN

    def test_angle_control_configuration(self):
        """Test angle control specific settings"""
        params = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)

        # Chery uses angle control
        assert params.steerControlType == structs.CarParams.SteerControlType.angle

        # Check angle-specific parameters
        assert params.steerActuatorDelay == 0.1
        assert params.steerLimitTimer == 1.0


class TestCheryCanBus:
    """Tests for Chery CAN bus implementation"""

    def test_canbus_properties(self):
        """Test CanBus class properties"""
        can_bus = CanBus()

        # Test all required properties exist and return correct values
        assert hasattr(can_bus, 'main')
        assert hasattr(can_bus, 'radar')
        assert hasattr(can_bus, 'camera')
        assert hasattr(can_bus, 'loopback')

        # Test values (assuming offset is 0 for base test)
        assert can_bus.main == 0
        assert can_bus.radar == 1
        assert can_bus.camera == 2
        assert can_bus.loopback == 128

    def test_static_canbus_values(self):
        """Test static CanBus values in values.py"""
        assert hasattr(CanBus, 'main')
        assert hasattr(CanBus, 'alt')
        assert hasattr(CanBus, 'camera')
        assert hasattr(CanBus, 'loopback')

        assert CanBus.main == 0
        assert CanBus.alt == 1
        assert CanBus.camera == 2
        assert CanBus.loopback == 128

    def test_crc_calculation(self):
        """Test CRC calculation function"""
        # Test with known data
        test_data = bytes([0x01, 0x02, 0x03, 0x04])
        poly = 0x1D
        xor_output = 0xA

        # Calculate CRC
        crc_result = calculate_crc(test_data, poly, xor_output)

        # Check CRC is a byte value
        assert 0 <= crc_result <= 255

        # Test empty data
        empty_crc = calculate_crc(bytes(), poly, xor_output)
        assert empty_crc == xor_output  # XOR with 0 should return xor_output


class TestCheryCarState:
    """Tests for Chery CarState implementation"""

    def test_carstate_initialization(self):
        """Test CarState initializes properly"""
        params = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)
        params_sp = structs.CarParamsSP()

        # Should not raise any exceptions
        car_state = CarState(params, params_sp)

        # Check required attributes exist
        assert hasattr(car_state, 'frame')
        assert hasattr(car_state, 'button_states')
        assert hasattr(car_state, 'lkas_enabled')
        assert hasattr(car_state, 'mads_enabled')

    def test_get_can_parsers(self):
        """Test CAN parser configuration"""
        params = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)
        params_sp = structs.CarParamsSP()

        parsers = CarState.get_can_parsers(params, params_sp)

        # Check all required buses are present
        assert Bus.pt in parsers
        assert Bus.cam in parsers
        assert Bus.loopback in parsers


class TestCheryCarController:
    """Tests for Chery CarController implementation"""

    def test_carcontroller_initialization(self):
        """Test CarController initializes properly"""
        params = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)
        params_sp = structs.CarParamsSP()
        dbc_names = {Bus.pt: "chery_canfd"}

        # Should not raise any exceptions
        car_controller = CarController(dbc_names, params, params_sp)

        # Check required attributes exist
        assert hasattr(car_controller, 'frame')
        assert hasattr(car_controller, 'apply_angle_last')
        assert hasattr(car_controller, 'accel')
        assert hasattr(car_controller, 'VM')  # Vehicle model


class TestCheryLongitudinal:
    """Specific tests for longitudinal control tuning"""

    @pytest.mark.parametrize("ki_values", [
        [0.5, 0.4, 0.2],      # Current
        [0.6, 0.5, 0.3],      # Alternative 1
        [0.5, 0.4, 0.15],     # Alternative 2
        [0.55, 0.45, 0.25],   # Balanced recommendation
    ])
    def test_longitudinal_tuning_options(self, ki_values):
        """Test different longitudinal tuning configurations"""
        # Check values are in descending order (more aggressive at low speed)
        assert all(ki_values[i] >= ki_values[i+1] for i in range(len(ki_values)-1))

        # Check all values are within safe range
        assert all(0.1 <= ki <= 2.5 for ki in ki_values)

        # Check the spread is reasonable (not too aggressive)
        spread = ki_values[0] - ki_values[-1]
        assert 0.2 <= spread <= 1.5

    def test_speed_interpolation(self):
        """Test speed-based gain interpolation logic"""
        # Test interpolation at different speeds
        bp = [0., 5., 35.]  # Breakpoints
        v = [0.5, 0.4, 0.2]  # Values

        # At 0 m/s
        assert np.interp(0, bp, v) == 0.5

        # At 5 m/s
        assert np.interp(5, bp, v) == 0.4

        # At 35 m/s
        assert np.interp(35, bp, v) == 0.2

        # At 20 m/s (interpolated)
        interpolated = np.interp(20, bp, v)
        assert 0.2 < interpolated < 0.4
        assert abs(interpolated - 0.3) < 0.01  # Should be close to 0.3


class TestCherySafety:
    """Safety-related tests"""

    def test_safety_flags(self):
        """Test safety flags are properly defined"""
        # Check CherySafetyFlags values
        assert hasattr(CherySafetyFlags, 'LONG_CONTROL')
        assert hasattr(CherySafetyFlags, 'CANFD')

        # Check they have expected values
        assert CherySafetyFlags.LONG_CONTROL == 1
        assert CherySafetyFlags.CANFD == 2

    def test_chery_flags(self):
        """Test Chery-specific flags"""
        assert hasattr(CheryFlags, 'CANFD')
        assert CheryFlags.CANFD == 1

    def test_steering_limits(self):
        """Test steering safety limits"""
        params = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)
        ctrl_params = CarControllerParams(params)

        # Check steering angle max is reasonable
        assert ctrl_params.STEER_ANGLE_MAX == 300  # degrees

        # Check driver allowance
        assert ctrl_params.STEER_DRIVER_ALLOWANCE == 15
        assert ctrl_params.STEER_THRESHOLD == 70

    def test_stop_behavior_params(self):
        """Test stopping behavior parameters"""
        params = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)

        # Check stop/start thresholds
        assert params.vEgoStopping == 0.1
        assert params.vEgoStarting == 0.1
        assert params.stoppingDecelRate == 0.3


if __name__ == "__main__":
    # Run tests with pytest
    pytest.main([__file__, "-v"])