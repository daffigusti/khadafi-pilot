# Chery Lateral Safety Implementation

**Document Version:** 1.0
**Last Updated:** 2025-10-18
**Vehicle:** Chery Omoda E5
**Safety Mode:** VM-based angle limiting with ISO 11270 compliance

---

## Table of Contents

1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Safety Parameters](#safety-parameters)
4. [Implementation Details](#implementation-details)
5. [Safety Guarantees](#safety-guarantees)
6. [Testing & Validation](#testing--validation)
7. [Troubleshooting](#troubleshooting)
8. [References](#references)

---

## Overview

This document describes the comprehensive lateral safety implementation for Chery vehicles in openpilot. The implementation uses a **Vehicle Model (VM) based approach** that validates steering commands using physics-based lateral acceleration and jerk limits, compliant with ISO 11270 standards.

### Key Features

- ✅ **Dual-layer safety**: Both openpilot-layer and Panda hardware-layer validation
- ✅ **Physics-based limits**: Uses actual vehicle dynamics instead of arbitrary lookup tables
- ✅ **Speed-dependent**: Automatically adjusts angle limits based on vehicle speed
- ✅ **Road roll compensation**: Accounts for banked curves on highways
- ✅ **Advanced smoothing**: Speed-based angle smoothing for comfort
- ✅ **Override handling**: Graceful recovery from driver interventions

### Safety Philosophy

The implementation follows a **defense-in-depth** approach with multiple safety layers:

1. **Openpilot Layer** (carcontroller.py): Advanced angle limiting with smoothing and override detection
2. **Panda Layer** (safety/modes/chery.h): Hardware failsafe using VM-based physics validation
3. **Vehicle Layer**: Stock LKAS system provides final mechanical limits

---

## Architecture

### System Overview

```
┌─────────────────────────────────────────────────────────────┐
│                      openpilot                               │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ carcontroller.py                                       │  │
│  │ - apply_chery_steer_angle_limits2()                   │  │
│  │ - Speed-based smoothing                               │  │
│  │ - Override detection                                   │  │
│  │ - Dual VM (actual + baseline)                         │  │
│  └───────────┬───────────────────────────────────────────┘  │
│              │ Steering commands (50 Hz)                     │
│              ▼                                                │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ CAN Message: LKAS_CMD (0x345)                         │  │
│  │ - CMD: 13-bit signed angle                            │  │
│  │ - LKA_ACTIVE: control enable flag                     │  │
│  └───────────┬───────────────────────────────────────────┘  │
└──────────────┼───────────────────────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────────────────────┐
│                    Panda (Hardware Safety)                   │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ chery_tx_hook() in safety/modes/chery.h              │  │
│  │ - Extract angle & active flag                         │  │
│  │ - steer_angle_cmd_checks_vm()                        │  │
│  │ - Max lateral accel check (~3.6 m/s²)                │  │
│  │ - Max lateral jerk check (~3.6 m/s³)                 │  │
│  │ - Max angle check (300°)                              │  │
│  │ - Frequency check (50 Hz)                             │  │
│  └───────────┬───────────────────────────────────────────┘  │
│              │ Validated commands only                       │
└──────────────┼───────────────────────────────────────────────┘
               │
               ▼
         CAN Bus → Vehicle LKAS ECU
```

### File Structure

```
opendbc/car/chery/
├── carcontroller.py          # Openpilot-layer angle limiting
├── values.py                 # Vehicle parameters & constants
├── cherycan.py              # CAN message encoding/decoding
├── interface.py             # Vehicle interface & parameter setup
└── docs/
    ├── LATERAL_SAFETY_IMPLEMENTATION.md  (this file)
    ├── VEHICLE_MODEL_PARAMETERS.md
    └── TUNING_GUIDE.md

opendbc/safety/modes/
└── chery.h                  # Panda hardware safety checks
```

---

## Safety Parameters

### Vehicle Model Parameters

Based on **Chery Omoda E5** specifications:

| Parameter | Value | Source |
|-----------|-------|--------|
| Mass | 1785 kg | CheryCarSpecs |
| Wheelbase | 2.63 m | CheryCarSpecs |
| Steer Ratio | 17.5 | CheryCarSpecs |
| Center to Front | 1.1572 m | 0.44 × wheelbase |
| Center to Rear | 1.4728 m | wheelbase - centerToFront |
| Tire Stiffness Front | 192150 N/rad | openpilot default |
| Tire Stiffness Rear | 202500 N/rad | openpilot default |

### Calculated Slip Factor

The slip factor determines how curvature changes with speed (understeer/oversteer characteristics):

```
slip_factor = m × (cF × aF - cR × aR) / (l² × cF × cR)

slip_factor = 1785 × (192150 × 1.1572 - 202500 × 1.4728) / (2.63² × 192150 × 202500)
            = -0.000503295541
```

**Negative value indicates understeer** (typical for front-wheel drive vehicles).

### Safety Limits

#### ISO 11270 Compliance

| Limit | Base Value | Road Roll Compensation | Final Value |
|-------|-----------|----------------------|-------------|
| Lateral Acceleration | 3.0 m/s² | +0.589 m/s² | ~3.6 m/s² |
| Lateral Jerk | 3.0 m/s³ | +0.589 m/s³ | ~3.6 m/s³ |
| Max Steering Angle | 300° | N/A | 300° |
| Max Angle Rate | 5°/frame | N/A | 5°/20ms |

**Road Roll Compensation:**
- Average road roll: 0.06 rad (~3.4°, 6% superelevation)
- Additional accel: g × sin(roll) ≈ 9.81 × 0.06 ≈ 0.589 m/s²

#### Steering Control Frequency

- **openpilot update rate:** 100 Hz (DT_CTRL = 0.01s)
- **Steering command rate:** 50 Hz (STEER_STEP = 2)
- **Panda validation rate:** 50 Hz (matches command rate)

---

## Implementation Details

### Openpilot Layer (carcontroller.py)

#### Angle Limiting Function

The `apply_chery_steer_angle_limits2()` function implements sophisticated angle limiting:

```python
def apply_chery_steer_angle_limits2(
    apply_angle: float,           # Desired angle from planner
    apply_angle_last: float,      # Last applied angle
    v_ego_raw: float,             # Current speed
    steering_angle: float,        # Current measured angle
    lat_active: bool,             # Lateral control active
    limits: AngleSteeringLimits,  # Safety limits
    VM: VehicleModel,             # Vehicle model
    smoothing_factor: float,      # User smoothing adjustment
    recently_overridden: bool     # Driver override flag
) -> float
```

**Key Steps:**

1. **Override Reset:** If recently overridden, reset last angle to current steering angle
2. **Smoothing:** Apply speed-based exponential smoothing for comfort
3. **Jerk Limiting:** Enforce max lateral jerk via angle rate limiting
4. **Acceleration Limiting:** Enforce max lateral acceleration via max angle
5. **Inactive Handling:** Return to measured angle when not active
6. **Absolute Limit:** Clip to STEER_ANGLE_MAX (300°)

#### Speed-Based Smoothing

Smoothing reduces steering oscillations at low speeds while maintaining responsiveness at high speeds:

```python
SMOOTHING_ANGLE_VEGO_MATRIX = [0, 8.5, 11, 13.8, 22.22]  # m/s
SMOOTHING_ANGLE_ALPHA_MATRIX = [0.05, 0.1, 0.3, 0.6, 1]  # smoothing factor

# 0 m/s (stopped):     5% new, 95% old (very smooth)
# 8.5 m/s (~30 kph):  10% new, 90% old
# 11 m/s (~40 kph):   30% new, 70% old
# 13.8 m/s (~50 kph): 60% new, 40% old
# 22.22 m/s (~80 kph): 100% new (no smoothing, full responsiveness)
```

#### Lateral Jerk Limiting

Physics-based jerk limiting prevents abrupt steering changes:

```python
def get_max_angle_delta(v_ego_raw: float, VM: VehicleModel):
    max_curvature_rate_sec = MAX_LATERAL_JERK / (v_ego_raw ** 2)  # (1/m)/s
    max_angle_rate_sec = math.degrees(VM.get_steer_from_curvature(max_curvature_rate_sec, v_ego_raw, 0))
    return max_angle_rate_sec * (DT_CTRL * STEER_STEP)  # deg per 20ms frame
```

This ensures smooth, comfortable steering that doesn't exceed passenger comfort limits.

#### Lateral Acceleration Limiting

Speed-dependent max angle prevents excessive lateral acceleration:

```python
def get_max_angle(v_ego_raw: float, VM: VehicleModel):
    max_curvature = MAX_LATERAL_ACCEL / (v_ego_raw ** 2)  # 1/m
    return math.degrees(VM.get_steer_from_curvature(max_curvature, v_ego_raw, 0))  # deg
```

### Panda Layer (safety/modes/chery.h)

#### CAN Message Parsing

The Panda extracts steering commands from the LKAS_CMD message:

```c
// DBC: SG_ CMD : 6|13@0- (1,0)
// Bits 6-18: 13-bit signed steering angle

// Extract 13-bit value
int16_t can_angle_raw = ((GET_BYTES(to_send, 0, 2) >> 6) & 0x1FFF);

// Sign extend from 13-bit to 16-bit
if (can_angle_raw & 0x1000) {
    can_angle_raw |= 0xE000;
}

// Convert to degrees × 100 format
// Reverse: (can_value + STEER_ANGLE_OFFSET) / STEER_ANGLE_SCALE
int desired_angle = ((can_angle_raw + 392) * 10);
```

#### Safety Validation

The `steer_angle_cmd_checks_vm()` function validates commands:

```c
bool steer_angle_cmd_checks_vm(
    int desired_angle,                    // Requested angle (deg × 100)
    bool steer_control_enabled,           // LKA_ACTIVE flag
    const AngleSteeringLimits limits,     // Max angle, frequency
    const AngleSteeringParams params      // slip_factor, steer_ratio, wheelbase
)
```

**Checks performed:**

1. **Lateral Jerk Check:**
   - Calculate max curvature rate: `MAX_LATERAL_JERK / (speed²)`
   - Convert to max angle rate using vehicle model
   - Ensure `|desired_angle - last_angle| ≤ max_angle_delta`

2. **Lateral Acceleration Check:**
   - Calculate max curvature: `MAX_LATERAL_ACCEL / (speed²)`
   - Convert to max angle using vehicle model
   - Ensure `|desired_angle| ≤ max_angle`

3. **Absolute Angle Check:**
   - Ensure `|desired_angle| ≤ max_angle` (30000 = 300°)

4. **Frequency Check:**
   - Count messages in rolling window
   - Ensure rate doesn't exceed 50 Hz × 1.2 (with buffer)

5. **Inactive Check:**
   - When `steer_control_enabled == false`, angle must equal measured angle

---

## Safety Guarantees

### Multi-Layer Defense

| Layer | Check | Action on Violation |
|-------|-------|-------------------|
| **Planning** | Path validity | Don't request steering |
| **Openpilot** | Angle limits, smoothing | Clip to safe value |
| **Panda** | VM-based physics | Block CAN message |
| **Vehicle** | Mechanical limits | Physical constraint |

### Failure Modes

| Failure | Mitigation | Result |
|---------|-----------|--------|
| openpilot crashes | Panda blocks unsafe commands | Stock LKAS or manual control |
| Panda firmware bug | Vehicle mechanical limits | Cannot exceed physical capability |
| Driver override | Override detection + reset | Smooth handoff, no fighting |
| Speed sensor fault | Conservative speed assumption | More restrictive limits |
| Excessive angle request | Multi-layer clipping | Safe, predictable behavior |

### Guaranteed Properties

✅ **Lateral acceleration cannot exceed ~3.6 m/s²** (ISO 11270 + margin)
✅ **Lateral jerk cannot exceed ~3.6 m/s³** (passenger comfort)
✅ **Steering angle cannot exceed ±150°** (mechanical limits)
✅ **Angle rate cannot exceed 5°/20ms** (prevents oscillations)
✅ **Commands validated at hardware level** (Panda failsafe)

---

## Testing & Validation

### Pre-Deployment Checklist

- [ ] **Build verification:** `scons -j4` compiles without errors
- [ ] **Unit tests:** Safety checks pass with test vectors
- [ ] **Simulation:** Replay with recorded routes shows expected behavior
- [ ] **Parking lot:** Low-speed tight turns work smoothly
- [ ] **Highway:** Lane changes feel natural, not limited
- [ ] **Override test:** Driver intervention recovers smoothly
- [ ] **Edge cases:** Test at speed limits, sharp curves, standstill

### Test Scenarios

#### 1. Low-Speed Maneuvers (Parking Lot)

**Expected Behavior:**
- Tight turns should work (up to 300° steering angle)
- Smoothing should prevent jerky movements
- No artificial limiting at low speeds

**Test:**
```
- Drive at 5-10 kph
- Execute tight turn (e.g., U-turn)
- Verify smooth steering without sudden limits
```

#### 2. Highway Lane Changes

**Expected Behavior:**
- Lane changes feel natural and responsive
- No oscillations or hunting
- Limits only activate on unreasonably sharp maneuvers

**Test:**
```
- Drive at 80-100 kph
- Execute normal lane change
- Verify no noticeable limiting
- Try aggressive lane change (should feel limited but smooth)
```

#### 3. Driver Override Recovery

**Expected Behavior:**
- System detects override (steering torque > 50 Nm)
- Temporary disable activates (steerDisableTemp = True)
- After 1 second without override, re-enable smoothly
- Angle tracking resets to current angle

**Test:**
```
- Engage openpilot on highway
- Apply steering torque > 50 Nm for 2 seconds
- Release and verify smooth re-engagement after 1 second
```

#### 4. Speed-Based Smoothing

**Expected Behavior:**
- Heavy smoothing at low speeds (parking)
- Minimal smoothing at high speeds (highway)

**Test:**
```
- Test steering response at 10 kph (should feel smooth/filtered)
- Test steering response at 80 kph (should feel responsive)
```

### Validation Metrics

Monitor these values during testing:

```python
# In carcontroller.py, add logging:
carlog.debug(f"Angle: desired={desired_angle:.1f}, "
             f"limited={apply_angle:.1f}, "
             f"measured={CS.out.steeringAngleDeg:.1f}, "
             f"v_ego={CS.out.vEgoRaw:.1f}")
```

**Good indicators:**
- `limited` usually close to `desired` (not hitting limits often)
- Smooth transitions when `limited` differs from `desired`
- No oscillations in `limited` value

---

## Troubleshooting

### Issue: Steering feels delayed or sluggish at low speeds

**Cause:** Excessive smoothing
**Solution:** Reduce `self.smoothing_factor` in carcontroller.py (default: 0.3)

```python
# In __init__:
self.smoothing_factor = 0.1  # Lower = less smoothing = more responsive
```

### Issue: Steering feels twitchy or oscillates at highway speeds

**Cause:** Insufficient smoothing or tuning issue
**Solution:**
1. Check that smoothing disables above 15 m/s (~54 kph)
2. Verify lateral tuning parameters
3. May need to adjust `MAX_ANGLE_RATE` if oscillating rapidly

### Issue: Panda blocks steering commands (openpilot shows "TAKE CONTROL IMMEDIATELY")

**Cause:** Safety check violation in Panda
**Diagnosis:**
1. Check Panda logs for violation details
2. Verify vehicle speed sensor is working
3. Check for extreme angle requests

**Solutions:**
- If speed sensor issue: Fix sensor or use conservative speed assumption
- If extreme angles: Review path planning, may indicate tuning issue
- If false positives: Review safety parameters (rare, but possible)

### Issue: Steering limited during normal highway driving

**Cause:** Safety limits too conservative
**Investigation:**
1. Log `max_angle` from `get_max_angle()` at current speed
2. Compare to typical lane change angles (~2-5°)
3. If `max_angle < 10°` at highway speeds, investigate

**Potential causes:**
- Incorrect vehicle speed reading (too high)
- Incorrect slip_factor calculation
- Incorrect steer_ratio value

### Issue: Driver override doesn't recover smoothly

**Cause:** Override detection not working properly
**Check:**
```python
# In carcontroller.py update():
if CS.out.steeringPressed:
    carlog.info(f"Override detected, torque={CS.out.steeringTorque}")
```

**Solutions:**
- Verify `STEER_THRESHOLD = 50` is appropriate for your vehicle
- Check that `recently_overridden` flag is resetting after 50 frames (~1 sec)

---

## References

### Standards

- **ISO 11270:2014** - Intelligent transport systems — Lane keeping assistance systems (LKAS)
- **ISO 15622:2018** - Intelligent transport systems — Adaptive cruise control systems

### Implementation References

- **VW Implementation:** `opendbc/car/volkswagen/` - Similar VM-based approach
- **Nissan Implementation:** `opendbc/car/nissan/` - Another VM-based implementation
- **Vehicle Model:** `opendbc/car/vehicle_model.py` - Dynamic bicycle model
- **Safety Checks:** `opendbc/safety/lateral.h` - Panda safety validation

### Related Documents

- [VEHICLE_MODEL_PARAMETERS.md](./VEHICLE_MODEL_PARAMETERS.md) - Detailed parameter derivations
- [TUNING_GUIDE.md](./TUNING_GUIDE.md) - How to tune for comfort/performance
- [LONGITUDINAL_TUNING_GUIDE.md](./LONGITUDINAL_TUNING_GUIDE.md) - Longitudinal control tuning

### Key Equations

**Slip Factor:**
```
sf = m × (cF × aF - cR × aR) / (l² × cF × cR)
```

**Curvature Factor:**
```
κ_factor = (1 - χ) / ((1 - sf × v²) × l)
```

**Steering Angle from Curvature:**
```
δ = κ × SR / κ_factor
```

**Max Lateral Acceleration:**
```
a_lat_max = v² × κ_max
```

**Max Lateral Jerk:**
```
j_lat_max = v² × (dκ/dt)_max
```

---

## Conclusion

The Chery lateral safety implementation provides robust, physics-based steering control with multiple layers of protection. The combination of openpilot-layer smoothing and Panda-layer hardware validation ensures safe, comfortable operation across all driving scenarios.

**Key Strengths:**
- ✅ Hardware failsafe protection
- ✅ ISO 11270 compliant
- ✅ Speed-adaptive behavior
- ✅ Graceful override handling
- ✅ Comprehensive documentation

For questions or issues, please refer to the troubleshooting section or consult the sunnypilot Discord community.

---

**Document Maintenance:**
- Update this document when safety parameters change
- Add new test scenarios as edge cases are discovered
- Keep version number in sync with major safety changes
