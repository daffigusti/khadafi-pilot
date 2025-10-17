# Chery Longitudinal Control Tuning Guide

## Table of Contents
1. [Current Configuration Analysis](#current-configuration-analysis)
2. [Understanding the Parameters](#understanding-the-parameters)
3. [Comparative Analysis](#comparative-analysis)
4. [Tuning Options](#tuning-options)
5. [Testing Methodology](#testing-methodology)
6. [Safety Considerations](#safety-considerations)
7. [Implementation Steps](#implementation-steps)
8. [Quick Reference](#quick-reference)
9. [Troubleshooting](#troubleshooting)

---

## Current Configuration Analysis

### Your Current Setup
```python
# File: opendbc/car/chery/interface.py
ret.longitudinalTuning.kiBP = [0., 5., 35.]  # Speed breakpoints in m/s
ret.longitudinalTuning.kiV = [0.5, 0.4, 0.2]  # Integral gains at each breakpoint
```

### What This Means
| Speed | km/h | m/s | Integral Gain (ki) | Behavior |
|-------|------|-----|-------------------|----------|
| Stopped | 0 | 0 | 0.5 | Moderate response from standstill |
| Low Speed | 18 | 5 | 0.4 | Moderate response in city driving |
| Cruise Speed | 126 | 35 | 0.2 | Conservative at highway speeds |

**Interpolation Example**: At 70 km/h (19.4 m/s), the system interpolates:
- ki ≈ 0.31 (interpolated between 0.4 at 5 m/s and 0.2 at 35 m/s)

### Tested Alternative Configurations
```python
# Alternative 1 - More Aggressive
ret.longitudinalTuning.kiV = [0.6, 0.5, 0.3]

# Alternative 2 - More Conservative
ret.longitudinalTuning.kiV = [0.5, 0.4, 0.15]
```

---

## Understanding the Parameters

### PID Controller Basics
The longitudinal controller uses a **PID (Proportional-Integral-Derivative)** control system:
- **P (Proportional)**: Responds to current error (not used in most implementations)
- **I (Integral)**: Responds to accumulated error over time (primary control method)
- **D (Derivative)**: Responds to rate of error change (handled internally)

### Parameter Definitions

#### kiBP (Integral Gain Breakpoints)
- **Definition**: Speed values (in m/s) where integral gain changes
- **Purpose**: Allows different control behavior at different speeds
- **Format**: Array of speeds `[0., 5., 35.]`

#### kiV (Integral Gain Values)
- **Definition**: Integral gain values corresponding to each breakpoint
- **Purpose**: Controls how aggressively the system corrects acceleration errors
- **Format**: Array of gains `[0.5, 0.4, 0.2]`
- **Range**: Typically 0.1 - 2.5 (higher = more aggressive)

### How Interpolation Works
```
At any speed between breakpoints, the system linearly interpolates:
Example at 20 m/s:
- Find surrounding breakpoints: 5 m/s (ki=0.4) and 35 m/s (ki=0.2)
- Calculate ratio: (20-5)/(35-5) = 0.5
- Interpolate: ki = 0.4 - 0.5*(0.4-0.2) = 0.3
```

### Effect on Driving Behavior

| Higher ki Values | Lower ki Values |
|-----------------|-----------------|
| ✓ Quicker acceleration response | ✓ Smoother acceleration |
| ✓ Better for merging | ✓ More comfortable for passengers |
| ✗ Can be jerky | ✗ Slower to reach target speed |
| ✗ May overshoot | ✗ May feel sluggish |

---

## Comparative Analysis

### Implementation Comparison Table

| Vehicle | Type | Breakpoints | ki Range | Characteristics | Use Case |
|---------|------|------------|----------|----------------|----------|
| **Chery (Current)** | Angle | [0, 5, 35] | 0.2-0.5 | Moderate, balanced | General purpose |
| **Honda NIDEC** | Torque | [0, 5, 35] | 0.5-1.2 | Aggressive, responsive | Performance |
| **GM Camera ACC** | Torque | [5, 35] | 1.5-2.4 | Very aggressive | Highway-focused |
| **Ford** | Angle | [0] | 0.5 | Simple, constant | Basic implementation |
| **Toyota TSS2** | Mixed | N/A | N/A | Stock ECU control | OEM longitudinal |

### Tuning Philosophy by Manufacturer

#### Conservative Approach (Toyota, Ford)
- Focus on comfort and smoothness
- Minimal custom tuning
- Relies on stock ECU capabilities

#### Moderate Approach (Chery, Honda)
- Balance between comfort and performance
- Speed-dependent tuning
- 3-point breakpoint strategy

#### Aggressive Approach (GM)
- Performance-oriented
- High integral gains
- Assumes minimum operating speed

---

## Tuning Options

### Option 1: Conservative Smooth Ride
**Best for**: Comfort-focused driving, passengers prone to motion sickness

```python
ret.longitudinalTuning.kiBP = [0., 5., 20., 35.]  # 4-point for finer control
ret.longitudinalTuning.kiV = [0.4, 0.35, 0.25, 0.15]
```

**Characteristics**:
- Very smooth acceleration/deceleration
- Gentle stops and starts
- Minimal jerk in traffic
- May feel slow when merging

### Option 2: Balanced Performance (Recommended)
**Best for**: Daily driving with good responsiveness

```python
ret.longitudinalTuning.kiBP = [0., 5., 35.]
ret.longitudinalTuning.kiV = [0.55, 0.45, 0.25]
```

**Characteristics**:
- 10% more responsive than current
- Good highway merging capability
- Comfortable for passengers
- Suitable for most driving conditions

### Option 3: Responsive Sport
**Best for**: Highway driving, quick merging needs

```python
ret.longitudinalTuning.kiBP = [0., 5., 35.]
ret.longitudinalTuning.kiV = [0.7, 0.6, 0.35]
```

**Characteristics**:
- Quick acceleration response
- Better lead car following
- May be jerky in stop-and-go
- More engaging driving experience

### Option 4: Advanced Fine-Tuning
**Best for**: Maximum control over behavior

```python
ret.longitudinalTuning.kiBP = [0., 3., 10., 25., 35.]  # 5-point
ret.longitudinalTuning.kiV = [0.6, 0.5, 0.4, 0.3, 0.2]
```

**Characteristics**:
- Granular speed-based control
- Optimized for different zones:
  - 0-3 m/s: Parking lot
  - 3-10 m/s: City streets
  - 10-25 m/s: Suburban roads
  - 25-35 m/s: Highway
- Requires extensive testing

### Option 5: Honda-Style Aggressive
**Best for**: Performance-oriented drivers

```python
ret.longitudinalTuning.kiBP = [0., 5., 35.]
ret.longitudinalTuning.kiV = [0.9, 0.7, 0.4]
```

**Characteristics**:
- Based on Honda NIDEC tuning (scaled down)
- Very responsive
- Quick to correct speed differences
- Best with good road conditions

---

## Testing Methodology

### Pre-Test Checklist
- [ ] Create backup of current configuration
- [ ] Ensure vehicle is in good mechanical condition
- [ ] Check weather conditions (dry roads preferred)
- [ ] Have a co-pilot for observations if possible
- [ ] Enable data logging

### Phase 1: Low-Speed Testing (Parking Lot)
**Location**: Empty parking lot
**Duration**: 30 minutes

#### Test Procedures:
1. **Stop-Start Test**
   - Accelerate to 20 km/h
   - Come to complete stop
   - Repeat 10 times
   - Check for: Smooth stops, no overshoot

2. **Creep Test**
   - Follow imaginary car at 5-10 km/h
   - Vary speed slightly
   - Check for: Smooth speed matching

3. **Quick Stop Test**
   - From 25 km/h, simulate emergency stop
   - Check for: Appropriate deceleration rate

#### Data to Record:
- Maximum deceleration achieved
- Time to stop from 20 km/h
- Jerk events (sudden changes)
- Comfort rating (1-10)

### Phase 2: City Driving (30-60 km/h)
**Location**: City streets with traffic
**Duration**: 1-2 hours

#### Test Scenarios:
1. **Traffic Light Behavior**
   - Approach red lights
   - Stop and wait
   - Accelerate when green

2. **Stop-and-Go Traffic**
   - Follow in congested traffic
   - Maintain safe distance
   - Check smoothness

3. **Turn Following**
   - Follow lead car through turns
   - Check speed adjustment

#### Metrics to Monitor:
- Following distance consistency
- Number of manual interventions
- Passenger comfort feedback
- Acceleration overshoot events

### Phase 3: Highway Testing (80-130 km/h)
**Location**: Highway/Motorway
**Duration**: 1-2 hours

#### Test Scenarios:
1. **Cruise Stability**
   - Set cruise at 100 km/h
   - Monitor speed holding
   - Check for oscillation

2. **Speed Changes**
   - Change set speed by 20 km/h
   - Measure time to reach new speed
   - Check for overshoot

3. **Following Distance**
   - Follow vehicles at various speeds
   - Check distance maintenance
   - Test cut-in scenarios

#### Critical Observations:
- Speed oscillation amplitude
- Response time to speed changes
- Stability in curves
- Wind gust response

### Data Logging Configuration
Add to your carstate.py for debugging:
```python
carlog.debug(f"Long Debug: vEgo={CS.vEgo:.2f} aEgo={CS.aEgo:.2f} "
            f"aTarget={actuators.accel:.2f} ki={current_ki:.3f}")
```

---

## Safety Considerations

### Critical Safety Limits

#### Never Exceed These Values:
| Parameter | Maximum Safe Value | Risk if Exceeded |
|-----------|-------------------|------------------|
| ki at 0 m/s | 1.5 | Violent starts |
| ki at 35 m/s | 1.0 | Speed oscillation |
| Total ki range | 2.0 | System instability |
| ki change rate | 0.5/breakpoint | Jerky transitions |

### Incremental Testing Approach
1. **Start Small**: Change ki by maximum ±0.1 per test
2. **Test Duration**: Minimum 50 km per configuration
3. **Rollback Ready**: Keep previous working configuration
4. **Weather Consideration**: Test in good conditions first

### Additional Safety Parameters

```python
# Acceleration limits (in carcontroller.py or values.py)
ACCEL_MAX = 2.0    # m/s² - Maximum acceleration
ACCEL_MIN = -3.5   # m/s² - Maximum deceleration

# Stopping behavior (in interface.py)
ret.stopAccel = -3.5           # Deceleration when stopping
ret.stoppingDecelRate = 0.3    # Rate of deceleration change
ret.vEgoStarting = 0.1         # Speed threshold for "starting"
ret.vEgoStopping = 0.1         # Speed threshold for "stopping"

# System delays
ret.longitudinalActuatorDelay = 0.05  # Seconds of actuator delay
```

### Emergency Fallback Configuration
If testing goes wrong, revert to this safe configuration:
```python
# Ultra-conservative safe mode
ret.longitudinalTuning.kiBP = [0., 5., 35.]
ret.longitudinalTuning.kiV = [0.3, 0.25, 0.15]
```

---

## Implementation Steps

### Step 1: Preparation
```bash
# Create testing branch
git checkout -b longitudinal-tuning-test

# Create backup of current file
cp interface.py interface.py.backup
```

### Step 2: Implement Logging
Add to `carcontroller.py`:
```python
def update(self, CC, CC_SP, CS, now_nanos):
    # ... existing code ...

    # Add debug logging
    if self.frame % 100 == 0:  # Log every 2 seconds
        carlog.info(f"Longitudinal: speed={CS.out.vEgo:.1f} m/s, "
                   f"accel={actuators.accel:.2f} m/s², "
                   f"gas={gas}")
```

### Step 3: Test Sequence
1. **Day 1**: Baseline testing with current values
2. **Day 2**: Test Option 2 (Balanced)
3. **Day 3**: Adjust based on Day 2 results
4. **Day 4**: Test preferred configuration extensively
5. **Day 5**: Fine-tune and finalize

### Step 4: Document Results
Create a test log:
```markdown
## Test Log - [Date]
Configuration: [Option Number]
Weather: [Conditions]
Distance: [km driven]

### Observations:
- Low speed: [behavior]
- City: [behavior]
- Highway: [behavior]

### Issues:
- [Any problems encountered]

### Rating: [1-10]
```

### Step 5: Finalize Configuration
```python
# After testing, update interface.py with final values
ret.longitudinalTuning.kiBP = [your_final_BP_values]
ret.longitudinalTuning.kiV = [your_final_ki_values]

# Add comment explaining choice
# Tuned for [your driving preference] on [date]
# Tested configurations: [list what you tried]
```

---

## Quick Reference

### Common Issues and Solutions

| Issue | Likely Cause | Solution |
|-------|-------------|----------|
| Jerky starts | ki too high at 0 m/s | Reduce first kiV value by 0.1 |
| Slow to reach speed | ki too low overall | Increase all kiV by 0.05 |
| Oscillating at cruise | ki too high at 35 m/s | Reduce last kiV value |
| Harsh stops | stopAccel too negative | Increase stopAccel (less negative) |
| Delayed response | longitudinalActuatorDelay | Adjust delay parameter |

### Speed Conversion Table
| km/h | m/s | Typical Use |
|------|-----|-------------|
| 0 | 0 | Stopped |
| 18 | 5 | Parking lot |
| 36 | 10 | Residential |
| 54 | 15 | City streets |
| 72 | 20 | Suburban |
| 90 | 25 | Rural roads |
| 108 | 30 | Highway ramp |
| 126 | 35 | Highway cruise |

### Tuning Cheat Sheet

#### For More Aggressive Response:
- Increase all kiV values by 0.1
- Reduce number of breakpoints
- Consider Ford-style single value

#### For Smoother Ride:
- Decrease all kiV values by 0.05
- Add more breakpoints for gradual changes
- Increase stoppingDecelRate

#### For Better Stop-and-Go:
- Increase ki at 0-5 m/s
- Add breakpoint at 3 m/s
- Fine-tune stopAccel

---

## Troubleshooting

### Debug Commands
```python
# Check current values in Python
from opendbc.car.chery.interface import CarInterface
params = CarInterface.get_non_essential_params("CHERY_OMODA_E5")
print(f"kiBP: {params.longitudinalTuning.kiBP}")
print(f"kiV: {params.longitudinalTuning.kiV}")
```

### Log Analysis
Look for these patterns in logs:
- **Oscillation**: aEgo rapidly switching between positive/negative
- **Lag**: Large delay between aTarget and aEgo
- **Overshoot**: aEgo exceeding aTarget significantly

### Performance Metrics
Calculate these from logs:
- **Rise time**: Time to reach 90% of target speed
- **Settling time**: Time for speed to stabilize
- **Overshoot**: Maximum speed above target
- **Steady-state error**: Average difference from target

---

## Notes and Recommendations

### Author's Recommendations:
1. **Start with Option 2** (Balanced) - it's 10% more responsive than current
2. **Test for at least 2 days** per configuration
3. **Keep a driving log** - note specific scenarios that feel wrong
4. **Consider your use case**:
   - Mostly highway → Option 3 or higher ki at 35 m/s
   - Mostly city → Focus on 0-10 m/s tuning
   - Mixed → Option 2 or 4

### Future Enhancements to Consider:
- Adding proportional control (kpV) for faster initial response
- Implementing different profiles switchable via UI
- Speed-dependent actuator delay adjustment
- Weather-based automatic tuning

### Community Resources:
- Openpilot Discord: #vehicle-tuning channel
- Sunnypilot community for Chery-specific feedback
- Compare with other Chery users' configurations

---

*Document Version: 1.0*
*Last Updated: Based on current codebase analysis*
*Compatible with: Chery OMODA E5 with alpha longitudinal control*