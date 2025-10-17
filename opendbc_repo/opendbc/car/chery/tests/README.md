# Chery Test Suite

This directory contains comprehensive tests for the Chery vehicle implementation in openpilot/sunnypilot.

## Test Coverage

### 1. **Fingerprint Tests** (`TestCheryFingerprint`)
- Validates fingerprint structure and format
- Verifies firmware version formatting
- Checks DBC file mappings

### 2. **Interface Tests** (`TestCheryInterface`)
- Validates `get_params()` returns correct values
- Tests longitudinal tuning parameter validity
- Verifies actuator limits are within safe ranges
- Confirms angle control configuration

### 3. **CAN Bus Tests** (`TestCheryCanBus`)
- Tests CanBus class properties (main, radar, camera, loopback)
- Validates static CanBus values
- Tests CRC calculation function

### 4. **CarState Tests** (`TestCheryCarState`)
- Tests CarState initialization
- Validates CAN parser configuration

### 5. **CarController Tests** (`TestCheryCarController`)
- Tests CarController initialization
- Validates required attributes

### 6. **Longitudinal Control Tests** (`TestCheryLongitudinal`)
- Tests different tuning configurations
- Validates speed-based gain interpolation
- Checks tuning safety bounds

### 7. **Safety Tests** (`TestCherySafety`)
- Validates safety flags
- Tests steering limits
- Checks stopping behavior parameters

## Running the Tests

### Run all Chery tests:
```bash
# From openpilot root directory
python -m pytest opendbc/car/chery/tests/

# With verbose output
python -m pytest opendbc/car/chery/tests/ -v

# With coverage report
python -m pytest opendbc/car/chery/tests/ --cov=opendbc.car.chery
```

### Run specific test class:
```bash
# Test only longitudinal tuning
python -m pytest opendbc/car/chery/tests/test_chery.py::TestCheryLongitudinal -v

# Test only safety features
python -m pytest opendbc/car/chery/tests/test_chery.py::TestCherySafety -v
```

### Run individual test:
```bash
# Test specific function
python -m pytest opendbc/car/chery/tests/test_chery.py::TestCheryInterface::test_longitudinal_tuning_valid -v
```

## Test Requirements

The tests require the following packages:
- pytest
- numpy
- opendbc (with Chery implementation)

## Adding New Tests

When adding new features to the Chery implementation, please add corresponding tests:

1. **For new parameters**: Add validation in `TestCheryInterface`
2. **For new CAN messages**: Add tests in `TestCheryCanBus`
3. **For safety changes**: Update `TestCherySafety`
4. **For longitudinal tuning**: Add to `TestCheryLongitudinal`

## Continuous Integration

These tests should be run:
- Before committing changes
- After modifying any Chery-related files
- When updating longitudinal tuning values
- Before creating pull requests

## Test Data

Test data for fingerprints and firmware versions is taken from:
- `opendbc/car/chery/fingerprints.py`
- `opendbc/car/chery/values.py`

## Common Test Failures and Solutions

### Longitudinal Tuning Tests Fail
- Check that `kiBP` and `kiV` arrays have the same length
- Ensure breakpoints are in ascending order
- Verify gain values are within safe ranges (0.1 - 2.5)

### CAN Bus Tests Fail
- Ensure `loopback` property is implemented in `cherycan.py`
- Check that all bus offsets are correct

### Safety Tests Fail
- Review acceleration limits in `CarControllerParams`
- Check steering angle limits
- Verify safety flags are properly set

## Contact

For questions about these tests, refer to:
- Openpilot Discord: #vehicle-tuning channel
- Sunnypilot community for Chery-specific discussions