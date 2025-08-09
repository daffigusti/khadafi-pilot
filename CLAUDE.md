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

## Important Notes

- This is safety-critical automotive software - all changes must prioritize safety
- The system runs in real-time with strict timing requirements
- Hardware dependencies mean not all functionality can be tested without actual comma device
- Vehicle-specific code requires careful validation to avoid unsafe behavior
- sunnypilot maintains compatibility with comma.ai's safety standards and protocols