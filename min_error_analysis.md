# Minimum Error Analysis — SCARA Robot Motors

## System Parameters (from scan)

| Parameter | Value | Source |
|---|---|---|
| MCU | STM32F446RET6 | `FULL_MACH_YEAHHH.ioc` |
| Core Clock | **16 MHz** (HSI, no PLL active) | `RCC.CortexFreq_Value=16000000` |
| PD Control Loop Period | **10 ms** (100 Hz) | `Custom_utils.c` — `if (HAL_GetTick() - last_calc_time < 10) return;` |
| Encoder Interface | TIM2 (x4), TIM3 (x4), TIM4 (x4) | `HW_Encoders.c` |
| Encoder PPR (M1/M2/M4) | 2000 P/R → **8000 CPR** (x4 decode) | `ENCODER_PPR = 2000` in `main.h` |
| M3 DC Encoder PPR | 11 P/R → **44 CPR** (x4 via EXTI) | `enc_ppr_val = 11` in `Custom_utils.c` |

---

## Motor-by-Motor Breakdown

---

### 🔵 M1 — Stepper (DM542), TIM2 Encoder

**Hardware:**
- Driver: DM542, microstepping = **1600 steps/rev** (`M1_STEP_VAL = 1600`)
- Encoder: Omron 2000 P/R, x4 → **8000 CPR**

**Calculations:**

| Quantity | Formula | Value |
|---|---|---|
| Deg/encoder count | `360 / (2000×4)` | **0.045 °/count** |
| Deg/step pulse | `360 / 1600` | **0.225 °/step** |
| Min detectable motion (1 enc count) | — | **0.045°** |
| Min stepper motion (1 step) | — | **0.225°** |
| Max step frequency (hardware cap) | `1,000,000 / 200 µs` | **5000 Hz** |
| Min step frequency (software cap) | `1,000,000 / 10000 µs` | **100 Hz** |
| Min motor speed at 100 Hz | `100 Hz × 360 / 1600 steps` | **22.5 °/s** |
| Min error to produce motion at Kp (≥22.5°/s needed) | `u = Kp × error ≥ 22.5` | **depends on Kp** |

**Why `M1_Err_Min = 0.3°`:**
- The comment says `> 0.225` (one step), so 0.3° provides a margin of **1.33× one step**.
- Below 0.3° the motor would attempt to command fractional steps → stutter / no motion.
- The encoder can *detect* errors as small as **0.045°**, but the actuator cannot *respond* below **0.225°**.

> [!IMPORTANT]
> **True physical minimum = 0.225° (1 step)**. The current deadband `0.3°` is conservatively set above this. You could lower it to `0.23°` safely, but not below `0.225°`.

---

### 🟢 M2 — Stepper (TB6600), TIM3 Encoder

**Hardware:**
- Driver: TB6600, microstepping = **1600 steps/rev** (`M2_STEP_VAL = 1600`)
- Encoder: Omron 2000 P/R, x4 → **8000 CPR**

Identical configuration to M1.

| Quantity | Value |
|---|---|
| Deg/encoder count | **0.045°/count** |
| Deg/step | **0.225°/step** |
| Min viable `Err_Min` | **> 0.225°** |
| Current setting | **0.3°** |

> [!TIP]
> M2 uses TB6600 instead of DM542. If its step signal minimum pulse width differs (TB6600 needs ≥2.5µs HIGH; DM542 needs ≥2.5µs HIGH too), both are well within the `STEP_HIGH_US = 8 µs` defined in `main.h`. No difference in minimum error from the driver side.

---

### 🟡 M3 — DC Motor (TB6612), EXTI Quadrature Encoder (PA11/PA12)

**Hardware:**
- Driver: TB6612 H-bridge, PWM on TIM12 CH1 (PB14)
- TIM12: Period = 999, Prescaler = 8 → PWM freq = `16MHz / (9 × 1000)` = **~1778 Hz**
- Encoder: Software EXTI quadrature, PPR = **11 P/R → 44 CPR** (x4 by QEM table)
- PWM range: 0–255 (clamped in `PD_DC_Compute`), but TIM12 Period = 999 (full scale)

**Calculations:**

| Quantity | Formula | Value |
|---|---|---|
| Deg/encoder count | `360 / (11×4)` | **8.18°/count** |
| Min detectable motion | 1 count | **8.18°** |
| Min PWM that moves motor | code: `abs_u < 10.0f → return 0` | **PWM = 10** |
| Min error to exceed PWM friction at Kp=9.6 | `Kp × error ≥ 10` → `error ≥ 10/9.6` | **≥ 1.04°** |
| Current `M3_Err_Min` | — | **2.0°** |

**Key insight — encoder resolution is the real bottleneck:**
- At 44 CPR, **1 encoder count = 8.18°**. This means the feedback system has extremely coarse resolution.
- You **cannot** set `M3_Err_Min` below **8.18°** and expect stable closed-loop behavior, because the controller cannot tell whether it is within 1 count or several counts of the target.

> [!WARNING]
> The current `M3_Err_Min = 2.0°` is actually **below** the encoder's 1-count resolution of **8.18°**. This means the controller is using a deadband smaller than the smallest measurable step — it will always overshoot and oscillate around ±1 count (±8.18°). You should raise this to at least **8.18°** for stable control, or swap to a higher-resolution encoder.

---

### 🔴 M4 — Stepper (DRV8825/A4988), TIM4 Encoder

**Hardware:**
- Driver: DRV8825 or A4988, microstepping = **800 steps/rev** (`M4_STEP_VAL = 800`)
- Encoder: Omron 2000 P/R, x4 → **8000 CPR**

**Calculations:**

| Quantity | Formula | Value |
|---|---|---|
| Deg/encoder count | `360 / (2000×4)` | **0.045°/count** |
| Deg/step | `360 / 800` | **0.45°/step** |
| Min viable `Err_Min` | — | **> 0.45°** |
| Current setting | — | **1.0°** |
| Margin | — | **2.22× one step** |

**Why the larger margin than M1/M2:**
- The comment says `> 0.45`, and `1.0°` = **2.22 steps** of margin.
- This is likely due to mechanical backlash or load inertia on the Z/prismatic axis (M4 likely drives a linear pulley using `M4_Pulley_Linear_Dia`).

> [!TIP]
> You *could* lower to `0.5°` (just above 1 step = 0.45°), but the current `1.0°` gives a reasonable backlash margin for a belt/leadscrew linear axis.

---

## Summary Table

| Motor | Type | Steps or CPR | Encoder Resolution | Min Physical Error | Current `Err_Min` | Verdict |
|---|---|---|---|---|---|---|
| M1 | Stepper (DM542) | 1600 steps/rev | 0.045°/count | **0.225°** (1 step) | **0.3°** ✅ | Slightly conservative, safe |
| M2 | Stepper (TB6600) | 1600 steps/rev | 0.045°/count | **0.225°** (1 step) | **0.3°** ✅ | Slightly conservative, safe |
| M3 | DC (TB6612) | 44 CPR | 8.18°/count | **8.18°** (1 enc count) | **2.0°** ⚠️ | **TOO LOW** — below encoder resolution |
| M4 | Stepper (DRV8825) | 800 steps/rev | 0.045°/count | **0.45°** (1 step) | **1.0°** ✅ | Good, accounts for backlash |

---

## Recommendations

### M1 & M2 — Fine-tune option
The current `0.3°` is safe. If you want tighter positioning, the lowest safe value is:
```c
#define M1_Err_Min 0.23f  // just above 0.225 deg/step
#define M2_Err_Min 0.23f
```

### M3 — Fix needed
The encoder resolution makes tight control impossible with a 44 CPR encoder. Choose one:

**Option A — Match deadband to encoder resolution (stable but coarse):**
```c
#define M3_Err_Min 8.2f  // 1 encoder count = 8.18 deg at 44 CPR
```

**Option B — If the DC motor has a gearbox or higher-res encoder:**
Measure actual CPR of the final shaft encoder to recalculate. The code hardcodes `ppr = 11`, so check if that's the motor shaft or output shaft.

### M4 — OK as-is
`1.0°` provides 2.22 steps of deadband — appropriate for a linear/Z-axis with potential backlash.

---

## Defined Constants (in [main.h](file:///d:/Robotics/SCARA_Robot/STM32/Core/Inc/main.h#L50-L53))

```c
#define M1_Err_Min 0.3f  // 1600 PPR = 0.225 deg/step, must be > 0.225
#define M2_Err_Min 0.3f  // 1600 PPR = 0.225 deg/step, must be > 0.225
#define M3_Err_Min 2.0f  // DC motor with Kp=9.6 needs > 1.04 deg to overcome 10 PWM friction deadband
#define M4_Err_Min 1.0f  // 800 PPR = 0.45 deg/step, must be > 0.45
```
