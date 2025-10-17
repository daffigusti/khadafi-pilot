#!/usr/bin/env python3
"""
Basic test runner for Chery implementation
Can be run without pytest for quick validation
"""

import sys
import traceback
from pathlib import Path

# Add parent directories to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent.parent))
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))


def test_imports():
    """Test that all Chery modules can be imported"""
    print("Testing imports...")
    try:
        from opendbc.car.chery import CAR
        from opendbc.car.chery.values import CarControllerParams, CheryFlags
        from opendbc.car.chery.interface import CarInterface
        from opendbc.car.chery.carstate import CarState
        from opendbc.car.chery.carcontroller import CarController
        from opendbc.car.chery.cherycan import CanBus, calculate_crc
        from opendbc.car.chery.fingerprints import FINGERPRINTS, FW_VERSIONS
        print("✅ All imports successful")
        return True
    except Exception as e:
        print(f"❌ Import failed: {e}")
        traceback.print_exc()
        return False


def test_canbus_properties():
    """Test CanBus properties exist"""
    print("\nTesting CanBus properties...")
    try:
        from opendbc.car.chery.cherycan import CanBus

        can_bus = CanBus()

        # Test properties
        assert hasattr(can_bus, 'main'), "Missing 'main' property"
        assert hasattr(can_bus, 'radar'), "Missing 'radar' property"
        assert hasattr(can_bus, 'camera'), "Missing 'camera' property"
        assert hasattr(can_bus, 'loopback'), "Missing 'loopback' property"

        # Test values
        assert can_bus.main == 0, f"main should be 0, got {can_bus.main}"
        assert can_bus.radar == 1, f"radar should be 1, got {can_bus.radar}"
        assert can_bus.camera == 2, f"camera should be 2, got {can_bus.camera}"
        assert can_bus.loopback == 128, f"loopback should be 128, got {can_bus.loopback}"

        print("✅ CanBus properties valid")
        return True
    except AssertionError as e:
        print(f"❌ CanBus test failed: {e}")
        return False
    except Exception as e:
        print(f"❌ CanBus test error: {e}")
        traceback.print_exc()
        return False


def test_longitudinal_tuning():
    """Test longitudinal tuning configuration"""
    print("\nTesting longitudinal tuning...")
    try:
        from opendbc.car.chery.interface import CarInterface
        from opendbc.car.chery.values import CAR

        # Get parameters
        params = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)

        # Check longitudinal tuning
        kiBP = params.longitudinalTuning.kiBP
        kiV = params.longitudinalTuning.kiV

        # Validate array lengths
        assert len(kiBP) == len(kiV), f"Mismatched array lengths: kiBP={len(kiBP)}, kiV={len(kiV)}"

        # Check expected values
        assert kiBP == [0., 5., 35.], f"Unexpected kiBP: {kiBP}"
        assert kiV == [0.5, 0.4, 0.2], f"Unexpected kiV: {kiV}"

        # Check ascending order
        for i in range(len(kiBP)-1):
            assert kiBP[i] < kiBP[i+1], f"kiBP not in ascending order at index {i}"

        # Check gain values are reasonable
        for ki in kiV:
            assert 0 <= ki <= 2.5, f"ki value {ki} out of safe range"

        print("✅ Longitudinal tuning valid")
        return True
    except AssertionError as e:
        print(f"❌ Longitudinal tuning test failed: {e}")
        return False
    except Exception as e:
        print(f"❌ Longitudinal tuning test error: {e}")
        traceback.print_exc()
        return False


def test_interface_methods():
    """Test CarInterface methods exist"""
    print("\nTesting interface methods...")
    try:
        from opendbc.car.chery.interface import CarInterface

        # Check methods exist
        assert hasattr(CarInterface, 'init'), "Missing 'init' method"
        assert hasattr(CarInterface, 'deinit'), "Missing 'deinit' method"
        assert hasattr(CarInterface, 'get_pid_accel_limits'), "Missing 'get_pid_accel_limits' method"
        assert hasattr(CarInterface, '_get_params'), "Missing '_get_params' method"
        assert hasattr(CarInterface, '_get_params_sp'), "Missing '_get_params_sp' method"

        # Test init/deinit don't raise errors
        CarInterface.init(None, None, None, None)
        CarInterface.deinit(None, None, None)

        print("✅ Interface methods valid")
        return True
    except AssertionError as e:
        print(f"❌ Interface methods test failed: {e}")
        return False
    except Exception as e:
        print(f"❌ Interface methods test error: {e}")
        traceback.print_exc()
        return False


def test_safety_parameters():
    """Test safety parameters"""
    print("\nTesting safety parameters...")
    try:
        from opendbc.car.chery.interface import CarInterface
        from opendbc.car.chery.values import CAR, CarControllerParams

        params = CarInterface.get_non_essential_params(CAR.CHERY_OMODA_E5)
        ctrl_params = CarControllerParams(params)

        # Test acceleration limits
        assert -4.0 <= ctrl_params.ACCEL_MIN <= -2.0, f"ACCEL_MIN {ctrl_params.ACCEL_MIN} out of range"
        assert 1.0 <= ctrl_params.ACCEL_MAX <= 3.0, f"ACCEL_MAX {ctrl_params.ACCEL_MAX} out of range"
        assert ctrl_params.ACCEL_MIN < ctrl_params.ACCEL_MAX, "ACCEL_MIN not less than ACCEL_MAX"

        # Test steering limits
        assert 0 < ctrl_params.STEER_ANGLE_MAX <= 360, f"STEER_ANGLE_MAX {ctrl_params.STEER_ANGLE_MAX} out of range"

        # Test specific values
        assert ctrl_params.STEER_ANGLE_MAX == 300, f"Expected STEER_ANGLE_MAX=300, got {ctrl_params.STEER_ANGLE_MAX}"
        assert ctrl_params.ACCEL_MIN == -3.5, f"Expected ACCEL_MIN=-3.5, got {ctrl_params.ACCEL_MIN}"
        assert ctrl_params.ACCEL_MAX == 2.0, f"Expected ACCEL_MAX=2.0, got {ctrl_params.ACCEL_MAX}"

        print("✅ Safety parameters valid")
        return True
    except AssertionError as e:
        print(f"❌ Safety parameters test failed: {e}")
        return False
    except Exception as e:
        print(f"❌ Safety parameters test error: {e}")
        traceback.print_exc()
        return False


def test_crc_function():
    """Test CRC calculation function"""
    print("\nTesting CRC calculation...")
    try:
        from opendbc.car.chery.cherycan import calculate_crc

        # Test with sample data
        test_data = bytes([0x01, 0x02, 0x03, 0x04])
        poly = 0x1D
        xor_output = 0xA

        crc = calculate_crc(test_data, poly, xor_output)

        # Check CRC is valid byte
        assert 0 <= crc <= 255, f"CRC {crc} out of byte range"

        # Test empty data
        empty_crc = calculate_crc(bytes(), poly, xor_output)
        assert empty_crc == xor_output, f"Empty CRC should be {xor_output}, got {empty_crc}"

        print("✅ CRC calculation valid")
        return True
    except Exception as e:
        print(f"❌ CRC test error: {e}")
        traceback.print_exc()
        return False


def main():
    """Run all basic tests"""
    print("=" * 60)
    print("CHERY IMPLEMENTATION BASIC TESTS")
    print("=" * 60)

    tests = [
        test_imports,
        test_canbus_properties,
        test_longitudinal_tuning,
        test_interface_methods,
        test_safety_parameters,
        test_crc_function,
    ]

    results = []
    for test in tests:
        results.append(test())

    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)

    passed = sum(results)
    total = len(results)

    print(f"Passed: {passed}/{total}")

    if passed == total:
        print("✅ All basic tests passed!")
        return 0
    else:
        print(f"❌ {total - passed} test(s) failed")
        return 1


if __name__ == "__main__":
    sys.exit(main())