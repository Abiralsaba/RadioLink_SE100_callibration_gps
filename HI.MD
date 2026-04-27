# 🔴 Two Critical Issues — What They Cause & What Changes After Fixing

---

## Issue 1: Compass I2C Failure → Silent Wrong Heading

### What is the problem?

When the compass (HMC5883L) fails to respond over I2C — due to a loose wire, vibration, electrical noise, or a sensor hang — the function `get_antenna_heading()` returns **0.0°** with absolutely no warning.

```cpp
// Current code (hardware_interface.cpp, line 334)
double get_antenna_heading() {
  float headingDegrees = 0.0;         // ← Pre-set to 0.0

  Wire.requestFrom(COMPASS_ADDR, 6);
  if (Wire.available() == 6) {
    // ... compute real heading ...
  }
  // If Wire.available() != 6 → falls through silently
  // headingDegrees is still 0.0
  // No error message printed
  // No flag set

  return (double)headingDegrees;      // ← Returns 0.0 as if heading IS North
}
```

### What does this cause in the field?

The control loop computes: `error = bearing - heading`

If heading is **silently stuck at 0.0** (pretending to be North), then:

```
error = bearing - 0.0 = bearing itself
servo_angle = 90° + bearing
```

**Real-world scenario:**

| Actual Situation | What Happens | What You See |
|-----------------|--------------|--------------|
| Rover is at bearing 150° (southeast) | `servo = 90 + 150 = 240°` → clamped to **180°** | Servo **slams** to maximum end-stop |
| Rover is at bearing 200° | `error = normalize(200 - 0) = -160°` → `servo = 90 + (-160) = -70°` → clamped to **0°** | Servo **slams** to opposite end-stop |
| Antenna is physically facing East (heading = 90°) | System thinks heading = 0° (North) | **90° tracking error** — antenna points completely wrong direction |
| I2C recovers after 5 seconds | Heading jumps from 0° to real value | Servo **jerks violently** to correct position |

**The worst part:** You won't see any error in the Serial Monitor. The system looks like it's working. You'll just see the antenna pointing the wrong way and wonder why.

### What happens after fixing?

After the fix, when I2C fails:

1. **The heading returns the last known good value** instead of 0.0 — the antenna holds its current position instead of snapping to a wrong angle
2. **An error message prints** to Serial Monitor: `[COMPASS] I2C read failed — using last heading: 45.2°` — you immediately know what's wrong
3. **The servo stays calm** — no violent jerks, no slamming into end-stops
4. When I2C recovers, the heading smoothly updates — no sudden jump because the last-known value was already close to the real one

**Before fix:**
```
Compass wire vibrates loose for 2 seconds
→ heading = 0° (silent)
→ servo slams to end-stop (180° or 0°)
→ compass reconnects
→ servo jerks back violently
→ you have no idea what happened
```

**After fix:**
```
Compass wire vibrates loose for 2 seconds
→ heading = last good value (e.g. 45°)
→ servo stays where it is (error ≈ 0)
→ Serial prints: "[COMPASS] I2C read failed"
→ compass reconnects
→ heading resumes smoothly
→ you know exactly what happened
```

---

## Issue 2: Pre-GPS Bearing Computation on (0,0) → (0,0)

### What is the problem?

On boot, GPS takes **30–60 seconds** to acquire satellites. During that time:

```
ant_lat = 0.0, ant_lon = 0.0    (no GPS fix yet)
rov_lat = 0.0, rov_lon = 0.0    (no telemetry yet)
```

But the control loop doesn't check this — it **immediately starts computing bearing and driving the servo:**

```cpp
// Current code (antenna.ino, line 24)
double bearing = compute_bearing(my_lat, my_lon, rover_lat, rover_lon);
//                                 0.0     0.0      0.0        0.0

double distance = compute_distance(my_lat, my_lon, rover_lat, rover_lon);
//                                   0.0     0.0      0.0        0.0  → distance = 0.0
```

`compute_bearing(0, 0, 0, 0)` → `atan2(0, 0)` → returns **0.0** on ESP32

Then:
```
error = normalize(0.0 - heading)
error = -heading
servo_angle = 90 + (-heading) = 90 - heading
```

### What does this cause in the field?

**For the first 30–60 seconds after every boot**, the antenna servo is driven by the **negative of the compass heading**, which is meaningless.

| Compass Heading | Servo Angle | What You See |
|----------------|-------------|--------------|
| 0° (North) | 90 - 0 = **90°** | Servo at center — looks OK by accident |
| 45° (NE) | 90 - 45 = **45°** | Servo at 45° — wrong, but not obvious |
| 90° (East) | 90 - 90 = **0°** | Servo at 0° — **slammed to one end** |
| 180° (South) | 90 - 180 = **-90°** → clamped to **0°** | Servo **slammed to end-stop** |
| 270° (West) | 90 - (-90) = **180°** | Servo **slammed to opposite end-stop** |

**With your new soft deadband (0.3×):** The dynamic deadband at `distance = 0.0m` is `min_db = 1.0°`. If heading < 1°, error is scaled to 0.3× — slightly dampened. If heading > 1° (almost always), **full raw error drives the servo.**

**Real-world scenario at competition:**

```
1. You power on the antenna system
2. GPS module starts searching for satellites
3. Compass reads heading = 130° (facing southeast)
4. servo_angle = 90 - 130 = -40° → clamped to 0°
5. Servo SLAMS to 0° immediately after boot
6. Antenna physically hits the mechanical end-stop
7. For 45 seconds, the servo stays pinned at 0°
8. GPS acquires — bearing jumps to real value
9. Servo jerks from 0° to the correct tracking angle
10. Last_Wish might save the wrong position during this phase
```

### What happens after fixing?

After the fix, during the pre-GPS phase:

1. **The servo stays at 90° (home position)** — the system recognizes that coordinates are invalid and skips the bearing computation entirely
2. **Serial Monitor shows:** `"Waiting for GPS"` and `"Waiting for Telemetry"` — clear status
3. **No servo movement until BOTH GPS and telemetry have valid data**
4. Once both acquire, tracking begins smoothly from the 90° center position
5. **No mechanical stress** — the servo never hits end-stops on boot
6. **No false Last_Wish saves** — the wrong angle is never written

**Before fix:**
```
Power on → sweep to 90° (correct)
→ 0.3 seconds later, GPS has no fix
→ servo driven by -heading = -130° → clamped to 0°
→ servo SLAMS from 90° to 0° ← mechanical stress
→ stays at 0° for 45 seconds ← looks broken
→ GPS acquires → servo JERKS to correct angle ← violent motion
```

**After fix:**
```
Power on → sweep to 90° (correct)
→ GPS has no fix, telemetry has no data
→ servo stays at 90° ← calm, professional
→ Serial: "Waiting for GPS... Waiting for Telemetry..."
→ GPS acquires at T+45s, telemetry arrives at T+3s
→ Both valid → tracking begins smoothly from 90° ← gentle start
```

---

## Summary

```
┌────────────────────────┬──────────────────────────┬──────────────────────────┐
│                        │     BEFORE FIX           │     AFTER FIX            │
├────────────────────────┼──────────────────────────┼──────────────────────────┤
│ Compass disconnects    │ Heading = 0° (silent)    │ Heading = last good val  │
│                        │ Servo slams to end-stop  │ Servo holds position     │
│                        │ No error message         │ Error printed to Serial  │
│                        │ Violent jerk on recovery │ Smooth recovery          │
├────────────────────────┼──────────────────────────┼──────────────────────────┤
│ Boot (no GPS yet)      │ Servo driven by -heading │ Servo stays at 90°       │
│                        │ Slams to 0° or 180°      │ Waits calmly for GPS     │
│                        │ 30-60s of wrong position │ Starts tracking smoothly │
│                        │ Mechanical end-stop hit  │ No mechanical stress     │
│                        │ False NVS saves possible │ No false saves           │
└────────────────────────┴──────────────────────────┴──────────────────────────┘
```
