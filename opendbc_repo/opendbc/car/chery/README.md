# Chery openpilot Implementation

**Vehicle:** Chery Omoda E5
**Status:** Production-ready with VM-based safety
**Last Updated:** 2025-10-18

---

## Overview

This directory contains the complete openpilot implementation for Chery vehicles, featuring:

- ✅ **Lateral control** with VM-based angle limiting
- ✅ **Longitudinal control** (alpha/experimental)
- ✅ **Hardware safety** validation in Panda firmware
- ✅ **Advanced smoothing** for passenger comfort
- ✅ **ISO 11270 compliance** for lateral safety

---

## Quick Start

### Prerequisites

- Chery Omoda E5 with stock LKAS capability
- comma three or C3X device
- sunnypilot fork installed

### Installation

1. Clone opendbc repository (included with sunnypilot)
2. Verify Chery is detected:
   ```bash
   # SSH into device
   cd /data/openpilot
   ./launch_openpilot.sh
   # Check logs for "Chery Omoda E5" detection
   ```

3. Enable longitudinal control (optional):
   ```bash
   # In sunnypilot settings UI:
   # Settings → Experimental Mode → Alpha Longitudinal
   ```

### First Drive

1. Start engine and engage openpilot with main cruise button
2. Verify green "openpilot" on screen
3. Test on low-traffic residential street first
4. Gradually test on highways after confirming functionality

---

## Documentation

### Core Documentation

1. **[LATERAL_SAFETY_IMPLEMENTATION.md](./LATERAL_SAFETY_IMPLEMENTATION.md)**
   - Complete safety architecture overview
   - Multi-layer defense explanation
   - Implementation details for both openpilot and Panda layers
   - Safety guarantees and failure modes
   - **Start here** to understand the system

2. **[VEHICLE_MODEL_PARAMETERS.md](./VEHICLE_MODEL_PARAMETERS.md)**
   - All vehicle specifications and parameters
   - Slip factor calculation and derivations
   - Parameter validation methods
   - Reference tables and conversion factors
   - **Read this** before tuning parameters

3. **[LATERAL_TUNING_GUIDE.md](./LATERAL_TUNING_GUIDE.md)**
   - Step-by-step tuning instructions
   - Smoothing adjustments for comfort
   - Override behavior customization
   - Common issues and solutions
   - **Use this** to optimize driving feel

4. **[LONGITUDINAL_TUNING_GUIDE.md](./LONGITUDINAL_TUNING_GUIDE.md)**
   - Acceleration and braking tuning
   - Following distance optimization
   - Longitudinal control parameters
   - **Reference this** for speed control tuning

---

## File Structure

```
opendbc/car/chery/
├── README.md                         (this file)
├── __init__.py                        Package initialization
├── carcontroller.py                   Main controller (lateral + longitudinal)
├── carstate.py                        Vehicle state parsing
├── cherycan.py                        CAN message encoding/decoding
├── interface.py                       Vehicle interface and parameters
├── values.py                          Constants and vehicle specs
├── fingerprints.py                    CAN fingerprints for detection
│
├── tests/                             Unit and integration tests
│   ├── test_chery.py                  Basic functionality tests
│   └── ...
│
└── docs/
    ├── LATERAL_SAFETY_IMPLEMENTATION.md
    ├── VEHICLE_MODEL_PARAMETERS.md
    ├── LATERAL_TUNING_GUIDE.md
    └── LONGITUDINAL_TUNING_GUIDE.md
```

---

## Key Features

### Lateral Control

**VM-Based Safety:**
- Physics-based angle limiting using vehicle dynamics model
- ISO 11270 compliant lateral acceleration (~3.6 m/s²)
- Road roll compensation for banked curves
- Hardware-level validation in Panda firmware

**Advanced Smoothing:**
- Speed-dependent filtering (heavy at low speeds, minimal at high speeds)
- User-adjustable smoothing factor
- Smooth override detection and recovery
- Temporary disable on driver intervention

**Safety Layers:**
1. Path planning (openpilot)
2. Angle limiting with smoothing (carcontroller.py)
3. Hardware validation (Panda firmware)
4. Vehicle mechanical limits

### Longitudinal Control

**Experimental Features:**
- Adaptive cruise control (ACC)
- Full stop and resume capability
- Speed-based acceleration limiting
- Smooth throttle control

**Safety:**
- Conservative acceleration limits
- Gradual braking for passenger comfort
- Stop-and-go traffic support

---

## Safety Parameters

### Lateral Safety (Fixed - DO NOT MODIFY)

| Parameter | Value | Standard |
|-----------|-------|----------|
| Max Lateral Accel | 3.6 m/s² | ISO 11270 + margin |
| Max Lateral Jerk | 3.6 m/s³ | Passenger comfort |
| Max Steering Angle | ±150° | Mechanical limit |
| Max Angle Rate | 5°/20ms | Oscillation prevention |

### Vehicle Model Parameters

| Parameter | Value | Source |
|-----------|-------|--------|
| Mass | 1785 kg | Manufacturer |
| Wheelbase | 2.63 m | Manufacturer |
| Steer Ratio | 17.5:1 | Manufacturer |
| Slip Factor | -0.000503 | Calculated |

See [VEHICLE_MODEL_PARAMETERS.md](./VEHICLE_MODEL_PARAMETERS.md) for complete parameter list.

---

## Tuning Quick Reference

### Common Adjustments

**Too smooth/sluggish:**
```python
# In carcontroller.py
self.smoothing_factor = 0.1  # Decrease from 0.3
```

**Too jerky at low speeds:**
```python
# In values.py
SMOOTHING_ANGLE_ALPHA_MATRIX = [0.03, 0.08, 0.25, 0.6, 1]  # Lower values
```

**Disengages too easily:**
```python
# In carcontroller.py, line ~165
if abs(CS.out.steeringTorque) >= 70 else 0  # Increase from 50
```

See [LATERAL_TUNING_GUIDE.md](./LATERAL_TUNING_GUIDE.md) for detailed tuning instructions.

---

## Testing & Validation

### Pre-Deployment Checklist

- [ ] Build compiles without errors: `scons -j4`
- [ ] Vehicle fingerprint detected correctly
- [ ] openpilot engages successfully on test drive
- [ ] Steering responds appropriately
- [ ] No constant safety warnings
- [ ] Override and re-engagement work smoothly

### Test Scenarios

1. **Parking lot** (5-10 kph): Tight turns, low-speed handling
2. **Residential** (30-50 kph): Gentle curves, smoothness
3. **Highway** (80-120 kph): Lane changes, responsiveness
4. **Override test**: Driver intervention and recovery
5. **Edge cases**: Sharp curves, wind gusts, varying speeds

---

## Troubleshooting

### Common Issues

**Issue:** Steering feels delayed
- **Solution:** Reduce `smoothing_factor` or adjust alpha matrix
- **Reference:** [LATERAL_TUNING_GUIDE.md#issue-1](./LATERAL_TUNING_GUIDE.md#issue-1-steering-feels-sluggish-at-all-speeds)

**Issue:** System disengages frequently
- **Solution:** Increase override threshold
- **Reference:** [LATERAL_TUNING_GUIDE.md#issue-3](./LATERAL_TUNING_GUIDE.md#issue-3-system-disengages-constantly)

**Issue:** Oscillates in curves
- **Solution:** Add high-speed smoothing
- **Reference:** [LATERAL_TUNING_GUIDE.md#issue-4](./LATERAL_TUNING_GUIDE.md#issue-4-oscillates-in-gentle-curves)

**Issue:** "TAKE CONTROL" warnings
- **Possible causes:**
  1. Speed sensor issue
  2. Safety parameter mismatch
  3. Excessive angle requests
- **Reference:** [LATERAL_SAFETY_IMPLEMENTATION.md#troubleshooting](./LATERAL_SAFETY_IMPLEMENTATION.md#troubleshooting)

### Getting Help

1. **Check documentation** in this directory first
2. **Search sunnypilot Discord** for similar issues
3. **Collect logs** before asking for help:
   ```bash
   # SSH into device
   cd /data/openpilot
   grep "Chery" /data/openpilot/log/* > /data/chery_debug.log
   ```
4. **Ask in Discord** with:
   - Vehicle model and year
   - Issue description
   - Steps to reproduce
   - Relevant log excerpts

---

## Development

### Running Tests

```bash
cd /data/openpilot/opendbc_repo
python -m pytest opendbc/car/chery/tests/
```

### Making Changes

1. **Backup current config:**
   ```bash
   git diff opendbc/car/chery/ > /data/my_changes.patch
   ```

2. **Make changes** to parameters or code

3. **Test thoroughly** in safe environment

4. **Document changes** and reasoning

5. **Consider contributing** back to sunnypilot

### Code Style

- Follow existing code style (PEP 8 for Python, LLVM for C)
- Add comments explaining non-obvious logic
- Update documentation when changing parameters
- Write tests for new functionality

---

## Safety Warning

⚠️ **IMPORTANT SAFETY NOTICE** ⚠️

- openpilot is a driver assistance system, **not autonomous**
- Always keep hands on wheel and eyes on road
- Be prepared to take control at any time
- Do not modify safety limits (lateral accel/jerk/angle)
- Test changes in safe, low-traffic environments first
- Understand changes before deploying to your vehicle

**YOU are responsible for safe operation of your vehicle.**

---

## Contributing

Improvements and bug fixes welcome!

**Before submitting:**
1. Test changes thoroughly on actual vehicle
2. Update relevant documentation
3. Follow existing code style
4. Include test results and validation data

**Where to contribute:**
- sunnypilot repository for general improvements
- This documentation for Chery-specific updates

---

## References

### Standards

- **ISO 11270:2014** - Lane keeping assistance systems (LKAS)
- **ISO 15622:2018** - Adaptive cruise control systems (ACC)

### Related Implementations

- **VW/Volkswagen:** Similar VM-based approach
- **Nissan:** Another VM-based implementation
- **Hyundai:** Advanced smoothing techniques

### External Resources

- [sunnypilot Documentation](https://docs.sunnypilot.ai/)
- [sunnypilot Discord](https://discord.gg/sunnypilot)
- [openpilot Wiki](https://github.com/commaai/openpilot/wiki)

---

## Version History

### Current (2025-10-18)
- Added VM-based Panda safety checks
- Comprehensive documentation suite
- Advanced smoothing with override handling
- Dual VM architecture (actual + baseline)
- Production-ready implementation

### Previous
- Basic LKAS functionality
- Experimental longitudinal control
- Initial parameter tuning

---

## License

This code is part of openpilot and follows the MIT license.
See [LICENSE](../../../../LICENSE) for details.

---

## Acknowledgments

- **comma.ai** for openpilot framework
- **sunnypilot team** for enhanced features
- **Community contributors** for testing and feedback
- **VW/Nissan teams** for VM-based safety approach reference

---

**For detailed technical information, see the documentation files listed above.**

**Happy driving! 🚗**
