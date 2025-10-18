# Chery Lateral Control Tuning Guide

**Document Version:** 1.0
**Last Updated:** 2025-10-18
**Vehicle:** Chery Omoda E5

---

## Table of Contents

1. [Overview](#overview)
2. [Before You Start](#before-you-start)
3. [Tuning Parameters](#tuning-parameters)
4. [Step-by-Step Tuning](#step-by-step-tuning)
5. [Common Issues & Solutions](#common-issues--solutions)
6. [Advanced Tuning](#advanced-tuning)
7. [Safety Considerations](#safety-considerations)

---

## Overview

This guide helps you tune the lateral (steering) control for optimal comfort and performance. The Chery implementation uses advanced VM-based angle limiting with speed-dependent smoothing.

### What Can Be Tuned

✅ **Steering smoothing** (comfort vs responsiveness)
✅ **Override sensitivity** (how quickly system responds to driver input)
✅ **Temporary disable behavior** (when driver intervenes)
❌ **Safety limits** (fixed by ISO 11270 standards)

### What You'll Need

- Test vehicle (Chery Omoda E5)
- Safe test route (highway + residential streets)
- Passenger to take notes (optional but recommended)
- Laptop with SSH access to device (for live parameter changes)

---

## Before You Start

### Baseline Verification

Before tuning, verify your baseline is correct:

#### 1. Check Safety Implementation

```bash
# SSH into your device
cd /data/openpilot/opendbc_repo
grep -A 5 "CHERY_STEERING_PARAMS" opendbc/safety/modes/chery.h
```

Should show:
```c
.slip_factor = -0.000503295541,
.steer_ratio = 17.5,
.wheelbase = 2.63,
```

#### 2. Test Basic Functionality

- [ ] Openpilot engages successfully
- [ ] Steering responds to lane lines
- [ ] No constant "TAKE CONTROL" warnings
- [ ] Vehicle tracks center of lane reasonably well

If any of these fail, **fix baseline issues before tuning**.

### Document Current Behavior

Before making changes, document current behavior:

**At 50 kph (residential):**
- Steering smoothness: [Too jerky / Just right / Too sluggish]
- Lane tracking: [Overshoots / Centered / Undershoots]
- Responsiveness: [Too aggressive / Just right / Too slow]

**At 100 kph (highway):**
- Lane changes: [Too sharp / Smooth / Too gradual]
- Curve handling: [Cuts inside / Good / Cuts outside]
- Wind gusts: [Overreacts / Dampened well / Underreacts]

---

## Tuning Parameters

### Primary Tuning Parameters

Located in `opendbc/car/chery/carcontroller.py`:

#### 1. Smoothing Factor

```python
# Line ~123
self.smoothing_factor = 0.3
```

**Effect:** Adds extra smoothing on top of speed-based smoothing
- **Range:** 0.0 - 1.0
- **Default:** 0.3
- **Lower values** (0.0-0.2): More responsive, less smooth
- **Higher values** (0.4-1.0): More smooth, less responsive

**When to adjust:**
- **Increase** if steering feels twitchy or jerky at low speeds
- **Decrease** if steering feels delayed or sluggish

#### 2. Override Threshold

```python
# Line ~165
self.steering_pressed_counter = self.steering_pressed_counter + 1 if abs(CS.out.steeringTorque) >= 50 else 0
```

**Effect:** How much torque triggers override detection
- **Range:** 30-100 Nm
- **Default:** 50 Nm
- **Lower values:** More sensitive to driver input
- **Higher values:** Less sensitive to driver input

**When to adjust:**
- **Decrease** if you want system to disengage more easily
- **Increase** if system disengages too easily (hands on wheel causing disengagement)

#### 3. Temporary Disable Duration

```python
# Line ~167-168
if self.steering_pressed_counter * DT_CTRL > 1:
    self.steerDisableTemp = True
```

**Effect:** How long override must be held before temporary disable
- **Range:** 0.5 - 2.0 seconds
- **Default:** 1.0 second
- **Formula:** `threshold_seconds = value / DT_CTRL`

**When to adjust:**
- **Decrease** if you want quicker disengagement
- **Increase** if brief wheel touches cause disengagement

#### 4. Re-enable Delay

```python
# Line ~172
if self.steering_unpressed_counter * DT_CTRL > 1:
    self.steerDisableTemp = False
```

**Effect:** How long after override before re-enabling
- **Range:** 0.5 - 3.0 seconds
- **Default:** 1.0 second

**When to adjust:**
- **Decrease** for faster re-engagement
- **Increase** if re-engagement feels abrupt

### Speed-Based Smoothing Matrix

Located in `opendbc/car/chery/values.py`:

```python
# Lines ~59-60
SMOOTHING_ANGLE_VEGO_MATRIX = [0, 8.5, 11, 13.8, 22.22]  # m/s
SMOOTHING_ANGLE_ALPHA_MATRIX = [0.05, 0.1, 0.3, 0.6, 1]  # smoothing
```

**Effect:** How much smoothing at different speeds
- Lower alpha = more smoothing (more filtering)
- Higher alpha = less smoothing (more responsive)

**Default behavior:**
- 0 m/s: 95% smoothing (very filtered)
- 30 kph: 90% smoothing
- 40 kph: 70% smoothing
- 50 kph: 40% smoothing
- 80+ kph: 0% smoothing (full responsiveness)

---

## Step-by-Step Tuning

### Phase 1: Highway Tuning (High Speed Behavior)

**Goal:** Ensure highway driving feels natural and safe

#### Test 1: Lane Changes (100 kph)

1. Find straight highway section with light traffic
2. Engage openpilot in center lane
3. Initiate lane change with turn signal
4. Observe behavior:

**Too aggressive:**
```python
# Increase high-speed smoothing
SMOOTHING_ANGLE_ALPHA_MATRIX = [0.05, 0.1, 0.3, 0.5, 0.8]  # Was: [... 1]
```

**Too sluggish:**
```python
# Already at maximum responsiveness (1.0)
# Check if path planning is the issue (separate from smoothing)
```

#### Test 2: Highway Curves

1. Find gentle highway curve (not sharp ramp)
2. Observe tracking through curve
3. Check for oscillations or hunting

**Oscillates/hunts:**
```python
# Add smoothing at high speeds
SMOOTHING_ANGLE_ALPHA_MATRIX = [0.05, 0.1, 0.3, 0.6, 0.9]  # Was: [... 1]
```

**Cuts inside curve:**
- Likely path planning issue, not smoothing
- Check lateral tuning in interface.py

#### Test 3: Wind Gusts

1. Drive on windy day or near passing trucks
2. Observe reaction to disturbances

**Overreacts:**
```python
# Increase smoothing factor
self.smoothing_factor = 0.4  # Was: 0.3
```

**Underreacts:**
```python
# Decrease smoothing factor
self.smoothing_factor = 0.2  # Was: 0.3
```

### Phase 2: Urban Tuning (Low Speed Behavior)

**Goal:** Smooth, comfortable driving in city traffic

#### Test 4: Residential Streets (50 kph)

1. Drive on residential street with gentle curves
2. Observe steering smoothness

**Too jerky:**
```python
# Increase low-speed smoothing
SMOOTHING_ANGLE_ALPHA_MATRIX = [0.03, 0.08, 0.25, 0.6, 1]  # Lower values
```

**Too laggy:**
```python
# Decrease low-speed smoothing
SMOOTHING_ANGLE_ALPHA_MATRIX = [0.08, 0.15, 0.35, 0.6, 1]  # Higher values
```

#### Test 5: Tight Curves (30 kph)

1. Drive through roundabout or tight curve
2. Verify system can handle without limiting

**Hits angle limits prematurely:**
- Check logs for `angle_limit_counter` increments
- Verify vehicle speed sensor is accurate
- May need to adjust MAX_ANGLE_RATE (advanced tuning)

### Phase 3: Override Behavior

**Goal:** Graceful handoff when driver intervenes

#### Test 6: Driver Override

1. Engage openpilot on straight road
2. Apply gentle steering torque (~60 Nm)
3. Observe system response

**Disengages too easily:**
```python
# Increase override threshold
if abs(CS.out.steeringTorque) >= 70 else 0  # Was: 50
```

**Fights against driver:**
```python
# Decrease override threshold
if abs(CS.out.steeringTorque) >= 40 else 0  # Was: 50
```

#### Test 7: Re-engagement

1. Trigger override (steer gently)
2. Hold for 2 seconds
3. Release and observe re-engagement

**Re-engages too quickly:**
```python
# Increase re-enable delay
if self.steering_unpressed_counter * DT_CTRL > 1.5:  # Was: 1.0
```

**Takes too long:**
```python
# Decrease re-enable delay
if self.steering_unpressed_counter * DT_CTRL > 0.7:  # Was: 1.0
```

---

## Common Issues & Solutions

### Issue 1: Steering Feels Sluggish at All Speeds

**Diagnosis:**
```python
# Add logging to carcontroller.py
carlog.debug(f"Smoothing: desired={desired_angle:.1f}, "
             f"smoothed={apply_angle:.1f}, "
             f"delta={abs(desired_angle - apply_angle):.1f}")
```

**If delta is consistently large (>5°):**

**Solution 1:** Reduce global smoothing
```python
self.smoothing_factor = 0.1  # Was: 0.3
```

**Solution 2:** Reduce speed-based smoothing
```python
SMOOTHING_ANGLE_ALPHA_MATRIX = [0.1, 0.2, 0.4, 0.7, 1]  # Higher values
```

### Issue 2: Jerky Steering at Low Speeds

**Diagnosis:**
- Feels like discrete steering steps
- More noticeable in parking lots

**Solution:** Increase low-speed smoothing
```python
SMOOTHING_ANGLE_ALPHA_MATRIX = [0.02, 0.05, 0.2, 0.6, 1]  # Lower at low speeds
```

### Issue 3: System Disengages Constantly

**Diagnosis:**
```python
# Add logging
carlog.info(f"Override: torque={CS.out.steeringTorque:.1f}, "
            f"pressed_counter={self.steering_pressed_counter}")
```

**If torque is low (<50) but counter increments:**

**Solution:** Increase threshold
```python
if abs(CS.out.steeringTorque) >= 70 else 0  # More tolerant
```

### Issue 4: Oscillates in Gentle Curves

**Diagnosis:**
- Steering weaves left-right in highway curves
- More noticeable at higher speeds

**Solution 1:** Add high-speed smoothing
```python
SMOOTHING_ANGLE_ALPHA_MATRIX = [0.05, 0.1, 0.3, 0.6, 0.85]  # Was: 1.0
```

**Solution 2:** Increase smoothing factor
```python
self.smoothing_factor = 0.4  # Was: 0.3
```

### Issue 5: Cuts Inside/Outside of Curves

**This is NOT a smoothing issue!**

This indicates lateral control tuning problem in `interface.py`:

```python
# Check lateral tuning parameters
ret.lateralTuning.init('pid')
ret.lateralTuning.pid.kpV = [...]
ret.lateralTuning.pid.kiV = [...]
```

Consult sunnypilot Discord for lateral tuning guidance.

---

## Advanced Tuning

### Custom Smoothing Profile

Create speed-specific smoothing for your driving style:

```python
# Aggressive (sports driving)
SMOOTHING_ANGLE_VEGO_MATRIX = [0, 5, 10, 15, 20]
SMOOTHING_ANGLE_ALPHA_MATRIX = [0.1, 0.3, 0.6, 0.9, 1.0]

# Comfort (relaxed cruising)
SMOOTHING_ANGLE_VEGO_MATRIX = [0, 10, 15, 20, 25]
SMOOTHING_ANGLE_ALPHA_MATRIX = [0.03, 0.1, 0.3, 0.6, 0.9]

# Balanced (default)
SMOOTHING_ANGLE_VEGO_MATRIX = [0, 8.5, 11, 13.8, 22.22]
SMOOTHING_ANGLE_ALPHA_MATRIX = [0.05, 0.1, 0.3, 0.6, 1.0]
```

### Conditional Smoothing

Apply different smoothing based on conditions:

```python
# In carcontroller.py update()
# More smoothing on curves
if abs(actuators.steeringAngleDeg) > 10:  # In curve
    adjusted_alpha *= 0.7  # 30% more smoothing
```

### Override Window Tuning

Fine-tune the override detection window:

```python
# Shorter window (more responsive to overrides)
OVERRIDE_FRAMES = 30  # 0.6 seconds at 50 Hz
if self.steering_pressed_counter > OVERRIDE_FRAMES:
    self.steerDisableTemp = True

# Longer window (ignores brief touches)
OVERRIDE_FRAMES = 75  # 1.5 seconds
```

---

## Safety Considerations

### DO NOT Modify

❌ **MAX_LATERAL_ACCEL** (3.6 m/s²) - ISO 11270 safety limit
❌ **MAX_LATERAL_JERK** (3.6 m/s³) - Passenger comfort limit
❌ **MAX_ANGLE_RATE** (5°/20ms) - Prevents oscillations and faults
❌ **STEER_ANGLE_MAX** (300°) - Vehicle mechanical limit

These are safety-critical and should never be increased.

### Safe to Modify

✅ **smoothing_factor** (0.0 - 1.0) - Only affects comfort
✅ **SMOOTHING_ANGLE_*_MATRIX** - Only affects responsiveness
✅ **Override thresholds** - Only affects engagement behavior
✅ **Temporary disable timing** - Only affects re-engagement

### Testing Safety

When testing changes:

1. **Start conservative:** Make small changes (±10%)
2. **Test incrementally:** One parameter at a time
3. **Use safe locations:** Empty parking lots first, then low-traffic roads
4. **Have backup:** Always be ready to take over manually
5. **Document everything:** Keep notes on what you changed and results

### Rollback Plan

Before making changes:

```bash
# Backup current configuration
cd /data/openpilot/opendbc_repo
git diff opendbc/car/chery/ > /data/my_tuning_backup.patch

# To restore:
git checkout opendbc/car/chery/carcontroller.py
git checkout opendbc/car/chery/values.py
```

---

## Recommended Starting Points

### Conservative (Comfortable, Smooth)

```python
# carcontroller.py
self.smoothing_factor = 0.4

# values.py
SMOOTHING_ANGLE_VEGO_MATRIX = [0, 10, 15, 20, 25]
SMOOTHING_ANGLE_ALPHA_MATRIX = [0.03, 0.08, 0.25, 0.6, 0.95]
```

### Balanced (Default)

```python
# carcontroller.py
self.smoothing_factor = 0.3

# values.py
SMOOTHING_ANGLE_VEGO_MATRIX = [0, 8.5, 11, 13.8, 22.22]
SMOOTHING_ANGLE_ALPHA_MATRIX = [0.05, 0.1, 0.3, 0.6, 1.0]
```

### Sporty (Responsive, Minimal Smoothing)

```python
# carcontroller.py
self.smoothing_factor = 0.1

# values.py
SMOOTHING_ANGLE_VEGO_MATRIX = [0, 5, 10, 15, 20]
SMOOTHING_ANGLE_ALPHA_MATRIX = [0.08, 0.2, 0.5, 0.8, 1.0]
```

---

## Tuning Worksheet

Use this worksheet to track your tuning sessions:

```
Date: ___________
Weather: ___________
Test Route: ___________

BEFORE:
- Smoothing Factor: ______
- Override Threshold: ______
- Highway Feel (1-10): ______
- City Feel (1-10): ______
- Issues: ___________________

CHANGES MADE:
1. ___________________
2. ___________________
3. ___________________

AFTER:
- Highway Feel (1-10): ______
- City Feel (1-10): ______
- Improvement: ___________________
- New Issues: ___________________

KEEP CHANGES? [ ] Yes [ ] No [ ] Need more testing
```

---

## Conclusion

Lateral tuning is an iterative process. Start with small changes, test thoroughly, and document your results. The default parameters work well for most users, but fine-tuning can optimize for your personal driving style and preferences.

**Remember:**
- Safety limits are non-negotiable
- Test in safe conditions
- Make incremental changes
- Always be ready to take manual control

For additional support, consult the [LATERAL_SAFETY_IMPLEMENTATION.md](./LATERAL_SAFETY_IMPLEMENTATION.md) or ask in the sunnypilot Discord community.

---

**Version History:**
- v1.0 (2025-10-18): Initial tuning guide for Chery Omoda E5
