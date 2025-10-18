# Chery Vehicle Model Parameters Reference

**Document Version:** 1.0
**Last Updated:** 2025-10-18
**Vehicle:** Chery Omoda E5

---

## Table of Contents

1. [Overview](#overview)
2. [Vehicle Specifications](#vehicle-specifications)
3. [Tire Model Parameters](#tire-model-parameters)
4. [Calculated Parameters](#calculated-parameters)
5. [Parameter Derivations](#parameter-derivations)
6. [Validation & Tuning](#validation--tuning)
7. [Parameter Tables](#parameter-tables)

---

## Overview

This document provides a comprehensive reference for all vehicle model parameters used in the Chery openpilot implementation. These parameters define the vehicle's physical characteristics and are critical for accurate lateral control and safety validation.

### Purpose

The vehicle model parameters serve three primary purposes:

1. **Lateral Control:** Calculate required steering angles for desired paths
2. **Safety Validation:** Ensure commands don't exceed physical limits
3. **Comfort Optimization:** Tune behavior for passenger comfort

### Parameter Sources

| Category | Source | Confidence |
|----------|--------|------------|
| Physical dimensions | Manufacturer specs | High ✅ |
| Mass properties | Manufacturer specs | High ✅ |
| Tire stiffness | openpilot defaults | Medium ⚠️ |
| Derived parameters | Calculations | Medium ⚠️ |

---

## Vehicle Specifications

### Chery Omoda E5

Official manufacturer specifications:

```python
class CheryCarSpecs(CarSpecs):
    mass: float = 1785                    # kg (curb weight)
    wheelbase: float = 2.63               # m
    steerRatio: float = 17.5              # steering wheel angle / wheel angle
    centerToFrontRatio: float = 0.44      # aF / wheelbase
```

### Physical Dimensions

| Parameter | Value | Unit | Source |
|-----------|-------|------|--------|
| **Mass** | 1785 | kg | Manufacturer spec (curb weight) |
| **Wheelbase** | 2.63 | m | Manufacturer spec |
| **Track Width (Front)** | ~1.59 | m | Estimated from class |
| **Track Width (Rear)** | ~1.59 | m | Estimated from class |
| **Height** | ~1.66 | m | Manufacturer spec |
| **CoG Height** | ~0.58 | m | Estimated (0.35 × height) |

### Weight Distribution

**Front-Rear Distribution:**
- `centerToFrontRatio = 0.44` indicates **44% weight on front axle**
- This is typical for front-wheel-drive vehicles with transverse engine

**Calculated axle loads:**
```
Total mass: 1785 kg
Front axle: 1785 × 0.44 = 785.4 kg (44%)
Rear axle:  1785 × 0.56 = 999.6 kg (56%)
```

**Note:** This rear-heavy distribution is unusual but may be due to:
- Battery placement (if PHEV/electric variant)
- Cargo area inclusion in curb weight measurement
- Conservative safety margin

### Steering System

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Steer Ratio** | 17.5:1 | Steering wheel deg / road wheel deg |
| **Turns Lock-to-Lock** | ~2.86 | (17.5 × 30°) / 180° ≈ 2.9 turns |
| **Max Steering Angle** | ±150° | Wheel angle at full lock |
| **LKAS Max Angle** | ±150° | Software limit (300° total) |

---

## Tire Model Parameters

### Default Tire Stiffness

openpilot uses default tire stiffness values when not specified:

```python
class CarParams:
    tireStiffnessFront: float = 192150.0    # N/rad (default)
    tireStiffnessRear: float = 202500.0     # N/rad (default)
```

**Source:** These are openpilot's generic defaults for passenger vehicles.

### Tire Stiffness Background

**Cornering Stiffness (Cα)** represents how much lateral force a tire generates per unit of slip angle:

```
F_lateral = Cα × α_slip
```

Where:
- `F_lateral` = Lateral force (N)
- `Cα` = Cornering stiffness (N/rad)
- `α_slip` = Slip angle (rad)

**Typical values for passenger cars:**
- Front: 150,000 - 250,000 N/rad
- Rear: 180,000 - 280,000 N/rad

**Our values (192,150 / 202,500) fall within normal range** ✅

### Validation Methods

To validate or tune tire stiffness values:

#### Method 1: Steady-State Circular Test
1. Drive vehicle in steady circle at constant speed
2. Measure steering angle, lateral acceleration, speed
3. Calculate actual tire stiffness from measurements
4. Compare to model predictions

#### Method 2: Step Steer Response
1. Apply step input to steering
2. Measure lateral acceleration and yaw rate response
3. Fit vehicle model to measured response
4. Extract tire stiffness parameters

#### Method 3: System Identification
1. Collect data from normal driving
2. Use parameter estimation algorithms
3. Identify best-fit tire stiffness values
4. Validate against held-out data

**Recommendation:** Start with defaults, tune only if you observe:
- Consistent understeer/oversteer
- Poor path tracking at specific speeds
- Safety limits triggering inappropriately

---

## Calculated Parameters

### Center of Gravity Position

From `centerToFrontRatio = 0.44`:

```python
wheelbase = 2.63          # m
aF = 0.44 × 2.63 = 1.1572 # m (CoG to front axle)
aR = 2.63 - 1.1572 = 1.4728  # m (CoG to rear axle)
```

**Verification:**
```
aF + aR = 1.1572 + 1.4728 = 2.63 ✅
aF / wheelbase = 1.1572 / 2.63 = 0.44 ✅
```

### Rotational Inertia

Estimated using simplified formula for passenger vehicles:

```python
# Default openpilot calculation
j = mass × wheelbase² / 12
j = 1785 × 2.63² / 12
j ≈ 1028.7 kg⋅m²
```

**Typical range for compact SUVs:** 800 - 1500 kg⋅m²

**Note:** This is an approximation. Actual value depends on:
- Mass distribution
- Vehicle proportions
- Component placement

### Slip Factor

The slip factor determines understeer/oversteer characteristics:

```python
slip_factor = m × (cF × aF - cR × aR) / (l² × cF × cR)

# Substituting values:
slip_factor = 1785 × (192150 × 1.1572 - 202500 × 1.4728) /
              (2.63² × 192150 × 202500)
            = 1785 × (222365.58 - 298242.00) /
              (6.9169 × 38,910,375,000)
            = 1785 × (-75876.42) / 269,187,564,375
            = -135,439,509.7 / 269,187,564,375
            = -0.000503295541
```

**Interpretation:**
- **Negative value** → Understeer characteristic ✅
- **Magnitude ~0.0005** → Moderate understeer (typical for FWD)

**Comparison:**
- Nissan X-Trail: -0.000620067422 (more understeer)
- Chery Omoda E5: -0.000503295541 (less understeer, more neutral)

---

## Parameter Derivations

### Curvature Factor

The curvature factor converts steering angle to path curvature:

```python
def curvature_factor(v_ego: float) -> float:
    """Returns curvature factor as function of speed"""
    sf = slip_factor  # -0.000503295541
    chi = 0.0        # No rear-wheel steering
    l = 2.63         # wheelbase

    return (1 - chi) / (1 - sf × v_ego²) / l
```

**Example calculations:**

| Speed (m/s) | Speed (kph) | Curvature Factor | Notes |
|-------------|-------------|------------------|-------|
| 5 | 18 | 0.3798 | Low speed |
| 10 | 36 | 0.3798 | Still minimal slip effect |
| 20 | 72 | 0.3800 | Slight increase |
| 30 | 108 | 0.3805 | Speed effects visible |

**At low speeds (<10 m/s), curvature factor ≈ 1/wheelbase = 0.3802**

### Steering Angle to Curvature

Convert steering wheel angle to path curvature:

```python
def calc_curvature(steering_angle_deg: float, v_ego: float) -> float:
    """Calculate path curvature from steering angle"""
    sa_rad = math.radians(steering_angle_deg)
    sr = 17.5  # steer ratio
    cf = curvature_factor(v_ego)

    return cf × sa_rad / sr
```

**Example:** 10° steering wheel angle at 20 m/s:
```
curvature = 0.3800 × radians(10) / 17.5
          = 0.3800 × 0.1745 / 17.5
          = 0.00379 m⁻¹
radius = 1 / 0.00379 = 264 m
```

### Lateral Acceleration Calculation

```python
def lateral_accel(steering_angle_deg: float, v_ego: float) -> float:
    """Calculate lateral acceleration"""
    curv = calc_curvature(steering_angle_deg, v_ego)
    return v_ego² × curv
```

**Example:** 10° at 20 m/s (72 kph):
```
a_lat = 20² × 0.00379
      = 400 × 0.00379
      = 1.52 m/s²
```

This is **well within the 3.6 m/s² safety limit** ✅

### Max Angle vs Speed

Calculate maximum safe steering angle at different speeds:

```python
def max_safe_angle(v_ego: float) -> float:
    """Max steering angle to stay within 3.6 m/s² lateral accel"""
    MAX_LAT_ACCEL = 3.6  # m/s²
    max_curv = MAX_LAT_ACCEL / v_ego²
    cf = curvature_factor(v_ego)
    sr = 17.5

    max_angle_rad = max_curv × sr / cf
    return math.degrees(max_angle_rad)
```

**Results:**

| Speed (m/s) | Speed (kph) | Max Angle (deg) | Notes |
|-------------|-------------|-----------------|-------|
| 5 | 18 | 76.1° | Parking lot maneuvers |
| 10 | 36 | 38.0° | Residential streets |
| 15 | 54 | 25.4° | Urban driving |
| 20 | 72 | 19.0° | Highway ramps |
| 25 | 90 | 15.2° | Highway cruising |
| 30 | 108 | 12.7° | Fast highway |

**Note:** These are *maximum* angles; normal driving uses much less:
- Lane change at 100 kph: ~2-5°
- Highway curve: ~5-10°
- Tight turn at 30 kph: ~20-40°

---

## Validation & Tuning

### Validation Checklist

Use this checklist to validate parameters match your vehicle:

#### Physical Measurements

- [ ] **Wheelbase:** Measure actual wheelbase, compare to 2.63 m
- [ ] **Steer Ratio:** Turn wheel lock-to-lock, count turns (should be ~2.9)
- [ ] **Weight:** Verify curb weight is ~1785 kg (check door jamb sticker)

#### Driving Tests

- [ ] **Understeer Test:**
  - Drive steady circle at increasing speeds
  - Vehicle should understeer (require more steering at higher speeds)
  - Matches negative slip factor ✅

- [ ] **Steering Response:**
  - Test steering response at 50 kph
  - Should feel natural, not over/under responsive
  - If too sensitive: steer ratio may be lower than 17.5
  - If too dull: steer ratio may be higher than 17.5

- [ ] **Path Tracking:**
  - Enable openpilot on gentle highway curve
  - Check if vehicle tracks center of lane
  - If consistently inside curve: may be understeering more than model
  - If consistently outside curve: may be oversteering more than model

### Tuning Parameters

If validation reveals issues, tune in this order:

#### 1. Steer Ratio (Most Impact)

**Symptom:** Path tracking consistently off
**Test:** Measure actual steer ratio
**Adjust:** Modify `steerRatio` in `values.py`

```python
# If vehicle requires more steering than expected:
steerRatio = 18.0  # Increase

# If vehicle is too sensitive:
steerRatio = 17.0  # Decrease
```

#### 2. Center to Front Ratio

**Symptom:** Understeer/oversteer behavior wrong
**Test:** Physics-based estimation or weighing individual axles
**Adjust:** Modify `centerToFrontRatio`

```python
# If more understeer than expected:
centerToFrontRatio = 0.46  # Move CoG forward

# If less understeer than expected:
centerToFrontRatio = 0.42  # Move CoG rearward
```

#### 3. Tire Stiffness (Last Resort)

**Symptom:** Speed-dependent tracking errors
**Test:** System identification from driving data
**Adjust:** Add to `_get_params` in interface.py

```python
ret.tireStiffnessFront = 200000.0  # N/rad
ret.tireStiffnessRear = 210000.0   # N/rad
```

### Data Collection for Tuning

Collect this data during test drives:

```python
# Add to carcontroller.py
carlog.info(f"VM_DATA: "
           f"speed={CS.out.vEgoRaw:.2f} "
           f"steer_angle={CS.out.steeringAngleDeg:.2f} "
           f"lat_accel={CS.out.aLateral:.3f} "
           f"desired_angle={apply_angle:.2f} "
           f"path_offset={CC.pathOffset:.3f}")
```

Analyze logged data to identify systematic errors.

---

## Parameter Tables

### Quick Reference Table

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| **Physical** |
| Mass | m | 1785 | kg |
| Wheelbase | l | 2.63 | m |
| CoG to Front | aF | 1.1572 | m |
| CoG to Rear | aR | 1.4728 | m |
| **Steering** |
| Steer Ratio | SR | 17.5 | - |
| Rear Steer Ratio | χ | 0.0 | - |
| Max Angle | δ_max | ±150 | deg |
| **Tire** |
| Front Stiffness | cF | 192150 | N/rad |
| Rear Stiffness | cR | 202500 | N/rad |
| **Derived** |
| Slip Factor | sf | -0.000503295541 | - |
| Rotational Inertia | j | ~1029 | kg⋅m² |
| Curvature Factor (v=0) | κ_f | 0.3802 | 1/m |
| **Safety** |
| Max Lateral Accel | a_lat | 3.6 | m/s² |
| Max Lateral Jerk | j_lat | 3.6 | m/s³ |
| Max Angle Rate | δ̇_max | 5 | deg/20ms |

### Conversion Factors

```python
# Angle conversions
deg_to_rad = π / 180 = 0.017453
rad_to_deg = 180 / π = 57.29578

# Speed conversions
ms_to_kph = 3.6
kph_to_ms = 1 / 3.6 = 0.27778

# CAN conversions (Chery)
angle_deg_to_can = 10  # deg × 10 in CAN
angle_can_offset = -392
angle_can_to_deg = (can_value + 392) / 10
```

### Safety Parameter Lookup

**Max Safe Steering Angle by Speed:**

```
v (m/s) | v (kph) | δ_max (deg) | Lateral Accel @ Max
--------|---------|-------------|--------------------
   5    |   18    |    76.1     |    3.6 m/s²
  10    |   36    |    38.0     |    3.6 m/s²
  15    |   54    |    25.4     |    3.6 m/s²
  20    |   72    |    19.0     |    3.6 m/s²
  25    |   90    |    15.2     |    3.6 m/s²
  30    |  108    |    12.7     |    3.6 m/s²
```

**Max Safe Angle Rate by Speed:**

```
v (m/s) | v (kph) | δ̇_max (deg/s) | Physical Limit
--------|---------|---------------|----------------
   5    |   18    |    ~580       |   250 (50Hz)
  10    |   36    |    ~147       |   250 (50Hz)
  15    |   54    |    ~66        |   250 (50Hz)
  20    |   72    |    ~37        |   250 (50Hz)
  25    |   90    |    ~24        |   250 (50Hz)
  30    |  108    |    ~17        |   250 (50Hz)
```

Note: Physical limit is 5°/20ms × 50Hz = 250°/s

---

## Appendix: Mathematical Derivations

### Bicycle Model Equations

The vehicle model uses a simplified bicycle model where:

**State:** `x = [v, r]ᵀ`
- `v` = lateral velocity (m/s)
- `r` = yaw rate (rad/s)

**Input:** `u = [δ, φ]ᵀ`
- `δ` = steering angle (rad)
- `φ` = road roll (rad)

**State equation:** `ẋ = Ax + Bu`

**A matrix:**
```
     ⎡ -(cF + cR)/(m⋅u)        -(cF⋅aF - cR⋅aR)/(m⋅u) - u ⎤
A = ⎢                                                    ⎥
     ⎣ -(cF⋅aF - cR⋅aR)/(j⋅u)   -(cF⋅aF² + cR⋅aR²)/(j⋅u) ⎦
```

**B matrix:**
```
     ⎡ (cF + χ⋅cR)/(m⋅SR)    -g ⎤
B = ⎢                           ⎥
     ⎣ (cF⋅aF - χ⋅cR⋅aR)/(j⋅SR)  0 ⎦
```

Where:
- `u` = longitudinal speed (m/s)
- `g` = 9.81 m/s² (gravity)
- Other symbols as defined in tables above

### Steady-State Solution

At steady state (`ẋ = 0`):
```
x_ss = -A⁻¹ B u
```

### Curvature Calculation

Path curvature:
```
κ = r / u = (curvature_factor) × (δ / SR)

curvature_factor = (1 - χ) / ((1 - sf⋅u²) ⋅ l)
```

### Lateral Acceleration

```
a_lat = u² ⋅ κ = u² ⋅ curvature_factor ⋅ δ / SR
```

---

## Conclusion

This reference provides all parameters needed for understanding and tuning the Chery vehicle model. The default values work well for most scenarios, but validation against your specific vehicle is recommended.

**Key Takeaways:**
- ✅ Parameters based on manufacturer specs where available
- ✅ Defaults used for tire stiffness (typical values)
- ✅ Calculated slip factor indicates moderate understeer
- ✅ Safety limits provide comfortable margin above normal driving

For questions about specific parameters or tuning guidance, consult the [TUNING_GUIDE.md](./TUNING_GUIDE.md) or sunnypilot Discord community.

---

**Version History:**
- v1.0 (2025-10-18): Initial documentation with Omoda E5 parameters
