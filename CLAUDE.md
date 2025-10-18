# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is **sunnypilot**, a fork of comma.ai's openpilot - an open source driver assistance system. sunnypilot supports 300+ car makes and models with modified driving assist behaviors while complying with comma.ai's safety rules. The project is written primarily in Python and C++, with some C components.

**Key Repositories:**

- Main repo: <https://github.com/sunnyhaibin/sunnypilot>
- Documentation: <https://docs.sunnypilot.ai/>
- Discord: <https://discord.gg/sunnypilot>

## Development Environment Setup

**System Requirements:**

- Python 3.11-3.12 (specified in pyproject.toml)
- SCons build system for native components
- Hardware: comma three, C3X devices, or development setup

**Installation:**

```bash
# Install Python dependencies
pip install -e .

# Install development dependencies
pip install -e ".[dev,testing]"

# Build native components (if not using prebuilt)
cd system/manager && ./build.py
```

## Common Development Commands

### Running the System
```bash
# Main entry point - launches the full openpilot system
./launch_openpilot.sh
# This executes ./launch_chffrplus.sh which starts system/manager/manager.py

# Run specific processes
cd system/manager && ./manager.py
```

### Testing
```bash
# Run all tests (uses pytest with parallel execution)
python -m pytest

# Run tests for specific component
python -m pytest selfdrive/controls/tests/
python -m pytest common/tests/
python -m pytest system/tests/

# Run specific test
python -m pytest selfdrive/test/test_onroad.py

# Run tests with coverage
python -m pytest --cov=selfdrive --cov=common --cov=system
```

### Code Quality
```bash
# Run linting (uses ruff)
ruff check .

# Run type checking
mypy .

# Run codespell for spelling errors
codespell

# Format code
ruff format .
```

### Building
```bash
# Build manager (main process orchestrator)
cd system/manager && ./build.py

# Build panda firmware (CAN interface)
cd panda && ./test.sh

# Build specific native components using SCons
scons -j8
```

## Architecture Overview

### Core System Components

**Process Management:**
- `system/manager/manager.py` - Main process orchestrator that manages all other processes
- `system/manager/process_config.py` - Defines which processes run under what conditions
- Uses a sophisticated process lifecycle management system with health monitoring

**Vehicle Interface:**
- `selfdrive/car/` - Vehicle-specific code and interfaces for 300+ supported car models
- `selfdrive/pandad/` - Interface to Panda device (CAN bus communication)
- Vehicle communication happens through CAN bus via the Panda hardware device

**Control Systems:**
- `selfdrive/controls/controlsd.py` - Main control loop (lateral and longitudinal vehicle control)
- `selfdrive/controls/plannerd.py` - Path planning and decision making
- `selfdrive/controls/radard.py` - Radar data processing for object detection
- Uses MPC (Model Predictive Control) for both lateral and longitudinal control

**Machine Learning Models:**
- `selfdrive/modeld/` - Vision model for road understanding, lane detection, and path prediction
- `sunnypilot/modeld*/` - sunnypilot-specific model variants and configurations
- `sunnypilot/models/` - Model management system with support for different model versions
- Models run via tinygrad framework (included as submodule in tinygrad_repo/)

**Localization & Mapping:**
- `selfdrive/locationd/` - GPS, IMU, and camera-based localization
- `selfdrive/locationd/calibrationd.py` - Camera calibration and mounting position estimation
- `sunnypilot/mapd/` - sunnypilot-specific mapping and navigation features

**User Interface:**
- `selfdrive/ui/` - Qt-based user interface for the device screen
- `system/ui/` - System-level UI components and utilities
- Supports multiple languages with translation files in selfdrive/ui/translations/

### sunnypilot-Specific Extensions

**Enhanced Features:**
- `sunnypilot/mads/` - Modified Automatic Driver Assistance System (MADS) - enhanced engagement behaviors
- `sunnypilot/sunnylink/` - Cloud connectivity and data synchronization service
- `sunnypilot/livedelay/` - Live delay management for improved system responsiveness
- `sunnypilot/navd/` - Navigation and routing enhancements

**Model System:**
- Multiple model variants supported through `sunnypilot/models/`
- Model fetching and management via `sunnypilot/models/fetcher.py`
- Support for both v1 and v2 model architectures in separate `modeld/` and `modeld_v2/` directories

### Data Flow Architecture

The system follows a publish-subscribe pattern using cereal (Cap'n Proto messaging):
1. **Hardware Layer:** Panda device interfaces with car's CAN bus
2. **Data Processing:** Raw sensor data (camera, radar, GPS) processed by specialized daemons
3. **Planning:** Control algorithms make driving decisions based on processed sensor data
4. **Execution:** Commands sent back to vehicle through CAN interface
5. **UI/Logging:** User interface updates and data logging happen in parallel

### Build System

- **Primary:** Python setuptools with pyproject.toml configuration
- **Native Components:** SCons build system for C/C++ components
- **Submodules:** Multiple git submodules (opendbc, panda, cereal, tinygrad, etc.) each with their own build systems
- **Cross-compilation:** Supports building for both x86_64 (development) and aarch64 (device) architectures

### Testing Strategy

- **Unit Tests:** pytest-based with extensive mocking for hardware components
- **Integration Tests:** Process replay tests using real driving data
- **Hardware-in-Loop:** Tests that require actual Panda device (`@pytest.mark.tici`)
- **Performance Tests:** Timing and resource usage validation
- **Code Quality:** Type checking (mypy), linting (ruff), and spelling (codespell)

## Development Workflow

1. **Local Development:** Most development can be done on PC using simulation/replay
2. **Hardware Testing:** Final validation requires comma device (comma three/C3X)
3. **Process Replay:** Use `tools/replay/` to replay driving segments for testing
4. **Continuous Integration:** Automated testing pipeline validates all changes
5. **Branch Structure:** Development against `master-new` branch for new features

## Chery Implementation Focus

**This repository has a special focus on Chery Omoda E5 implementation and tuning.**

### Overview

The Chery implementation uses an advanced **Vehicle Model (VM) based safety approach** with:
- Physics-based lateral control using actual vehicle dynamics
- ISO 11270 compliant safety limits (~3.6 m/s² lateral acceleration)
- Dual-layer safety: openpilot-layer smoothing + Panda hardware validation
- Speed-dependent angle smoothing for passenger comfort
- Comprehensive override detection and recovery

### Key Documentation

All Chery-specific documentation is located in `opendbc_repo/opendbc/car/chery/`:

1. **[README.md](opendbc_repo/opendbc/car/chery/README.md)** - Start here! Quick start guide and overview
2. **[LATERAL_SAFETY_IMPLEMENTATION.md](opendbc_repo/opendbc/car/chery/LATERAL_SAFETY_IMPLEMENTATION.md)** - Complete technical documentation
   - Safety architecture (openpilot + Panda layers)
   - Implementation details with code examples
   - Testing procedures and validation
   - Troubleshooting guide
3. **[VEHICLE_MODEL_PARAMETERS.md](opendbc_repo/opendbc/car/chery/VEHICLE_MODEL_PARAMETERS.md)** - Parameter reference
   - All vehicle specifications and calculations
   - Slip factor derivation (−0.000503295541)
   - Validation methods and tuning guidance
4. **[LATERAL_TUNING_GUIDE.md](opendbc_repo/opendbc/car/chery/LATERAL_TUNING_GUIDE.md)** - User tuning guide
   - Step-by-step tuning instructions
   - Smoothing adjustment procedures
   - Common issues and solutions
5. **[LONGITUDINAL_TUNING_GUIDE.md](opendbc_repo/opendbc/car/chery/LONGITUDINAL_TUNING_GUIDE.md)** - Longitudinal control tuning

### Development Focus Areas

When working on Chery implementation, prioritize:

1. **Lateral Control Tuning**
   - Smoothing parameters (`smoothing_factor`, `SMOOTHING_ANGLE_*_MATRIX`)
   - Override detection sensitivity
   - Re-engagement behavior

2. **Safety Validation**
   - Panda safety checks in `opendbc_repo/opendbc/safety/modes/chery.h`
   - VM parameter accuracy (slip_factor, steer_ratio, wheelbase)
   - Testing at various speeds and conditions

3. **Vehicle Model Refinement**
   - Validate steer ratio (currently 17.5, estimated)
   - Fine-tune mass if needed (currently 1785 kg, conservative)
   - Adjust centerToFrontRatio if weight distribution data available

4. **Documentation Maintenance**
   - Update docs when parameters change
   - Add new test scenarios as discovered
   - Document any tuning adjustments made

### Chery-Specific Commands

```bash
# Build opendbc with Chery safety code
cd opendbc_repo
scons -j8

# Run Chery-specific tests
python -m pytest opendbc/car/chery/tests/

# Test Panda safety compilation (if panda hardware available)
cd panda
./test.sh

# View Chery safety parameters
grep -A 10 "CHERY_STEERING" opendbc_repo/opendbc/safety/modes/chery.h

# Check current vehicle parameters
grep "CheryCarSpecs" opendbc_repo/opendbc/car/chery/values.py
```

### File Locations

**Core Implementation:**
- `opendbc_repo/opendbc/car/chery/carcontroller.py` - Main control logic with VM-based angle limiting
- `opendbc_repo/opendbc/car/chery/values.py` - Vehicle parameters and constants
- `opendbc_repo/opendbc/car/chery/interface.py` - Vehicle interface setup
- `opendbc_repo/opendbc/safety/modes/chery.h` - Panda hardware safety validation

**Key Parameters:**
```python
# values.py
CheryCarSpecs(
    mass=1785,           # kg (conservative, official: 1776 kg)
    wheelbase=2.63,      # m (verified accurate)
    steerRatio=17.5      # estimated, needs validation
)

# carcontroller.py
smoothing_factor = 0.3                           # Global smoothing
SMOOTHING_ANGLE_VEGO_MATRIX = [0, 8.5, 11, 13.8, 22.22]  # Speed breakpoints (m/s)
SMOOTHING_ANGLE_ALPHA_MATRIX = [0.05, 0.1, 0.3, 0.6, 1]   # Smoothing factors

# chery.h (Panda safety)
slip_factor = -0.000503295541    # Calculated from vehicle dynamics
max_angle = 30000                # 300° * 100 (conversion factor)
frequency = 50U                  # 50 Hz steering command rate
```

### Important Chery-Specific Notes

⚠️ **Safety-Critical:**
- VM-based safety is **hardware-enforced** in Panda firmware as last line of defense
- Even if openpilot crashes, Panda blocks unsafe steering commands
- Safety limits (lateral accel, jerk, max angle) are **non-negotiable** per ISO 11270

✅ **Validated Parameters:**
- Wheelbase: 2.63 m (exact match with official specs)
- Mass: 1785 kg (slightly conservative, official tare weight: 1776 kg)
- Slip factor: Calculated from vehicle dynamics, indicates moderate understeer (typical for FWD)

❓ **Needs Validation:**
- Steer ratio: 17.5 is estimated (measure lock-to-lock turns on actual vehicle)
- Center to front ratio: 0.44 is reasonable but not verified

🔧 **Tuning Philosophy:**
- **DO modify:** Smoothing factors, override thresholds (comfort/behavior)
- **DO NOT modify:** Safety limits, max angles, lateral accel/jerk (safety-critical)
- Always test changes in safe, low-traffic environments first

### Quick Troubleshooting

Common issues and where to look:

| Issue | Check | Reference |
|-------|-------|-----------|
| Steering feels sluggish | `smoothing_factor` in carcontroller.py | LATERAL_TUNING_GUIDE.md |
| System disengages often | Override threshold (line ~165) | LATERAL_TUNING_GUIDE.md |
| "TAKE CONTROL" warnings | Panda safety logs, speed sensor | LATERAL_SAFETY_IMPLEMENTATION.md |
| Path tracking issues | Lateral PID tuning (interface.py) | sunnypilot Discord |
| Oscillations in curves | High-speed smoothing matrix | LATERAL_TUNING_GUIDE.md |

For detailed troubleshooting, see the documentation files listed above.

---

## Important Notes

- This is safety-critical automotive software - all changes must prioritize safety
- The system runs in real-time with strict timing requirements
- Hardware dependencies mean not all functionality can be tested without actual comma device
- Vehicle-specific code requires careful validation to avoid unsafe behavior
- sunnypilot maintains compatibility with comma.ai's safety standards and protocols