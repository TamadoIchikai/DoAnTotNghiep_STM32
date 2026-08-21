# Minimum Cartesian Error Analysis — SCARA Robot

## Robot Geometry (from [main.c](file:///d:/Robotics/SCARA_Robot/STM32/Core/Src/main.c#L151-L178))

```
L1 = 230 mm  (link 1 — M1 to M2 joint)
L2 = 120 mm  (link 2 — M2 joint to end-effector)
M4 pulley diameter = 13 mm  (linear Z-axis)
```

The SCARA robot is a standard **Revolute-Revolute-Revolute-Prismatic** (RRRP) structure:
- **M1 (θ₁)** → rotates entire arm from base
- **M2 (θ₂)** → rotates forearm (link 2) relative to upper arm
- **M3 (θ₃)** → pure wrist/end-effector rotation (no X/Y/Z translation)
- **M4 (d₄)**  → prismatic Z-axis via pulley belt

---

## SCARA Jacobian for X/Y (Planar)

For a 2-DOF SCARA arm in the X-Y plane, the positional Jacobian is:

```
∂x/∂θ₁ = -L1·sin(θ₁) - L2·sin(θ₁+θ₂)
∂x/∂θ₂ = -L2·sin(θ₁+θ₂)
∂y/∂θ₁ =  L1·cos(θ₁) + L2·cos(θ₁+θ₂)
∂y/∂θ₂ =  L2·cos(θ₁+θ₂)
```

The **worst-case Cartesian error** (maximum Jacobian magnitude) occurs when both links are fully extended in the same direction:

```
|∂x/∂θ₁|_max = L1 + L2 = 230 + 120 = 350 mm/rad
|∂x/∂θ₂|_max = L2       = 120 mm/rad
```

---

## Cartesian Error per Joint

### ✅ M1 (θ₁ = 0.3°) → X/Y error

```
Δθ₁ = 0.3° = 0.005236 rad

Worst-case ΔXY from M1 alone:
  ΔXY_max = (L1 + L2) × Δθ₁
           = 350 mm × 0.005236 rad
           = 1.83 mm
```

### ✅ M2 (θ₂ = 0.3°) → X/Y error

```
Δθ₂ = 0.3° = 0.005236 rad

Worst-case ΔXY from M2 alone:
  ΔXY_max = L2 × Δθ₂
           = 120 mm × 0.005236 rad
           = 0.63 mm
```

### ⚠️ M3 (θ₃ = 8.2°) → **Rotation only, no X/Y/Z translation**

M3 is the wrist rotation joint. From the screw axis `S` matrix in [main.c](file:///d:/Robotics/SCARA_Robot/STM32/Core/Src/main.c#L173-L178):

```c
// S column 3 = {0, 0, 1, 0, -(L1+L2), 0}  — pure rotation about Z through end-effector
```

This means M3 only rotates the end-effector's orientation. It produces **zero X/Y/Z positional error** — only heading/orientation error.

```
M3 orientation error = 8.2°  (≈ ±8.18° deadband)
M3 X/Y/Z error       = 0 mm  (it's a pure rotation joint)
```

> [!WARNING]
> The 8.2° orientation deadband means the gripper could be pointing up to **±8.18° off target**. For a pick-and-place task this matters a lot for rotated parts, but does not affect where the TCP lands.

### ✅ M4 (d₄ = 1°) → Z-axis error

M4 is a prismatic joint driven by a pulley belt (diameter = 13 mm).

```
1 motor degree → linear Z displacement:
  Δz = π × D × Δθ/360
     = π × 13 mm × 1° / 360°
     = 0.1134 mm ≈ 0.113 mm
```

---

## Summary Table

| Motor | Joint Type | Min Angle Error | Cartesian Impact | Min Cartesian Error |
|---|---|---|---|---|
| **M1** | Revolute (base) | **0.3°** | X/Y — scales with L1+L2 = 350 mm | **≤ 1.83 mm** (worst case) |
| **M2** | Revolute (elbow) | **0.3°** | X/Y — scales with L2 = 120 mm | **≤ 0.63 mm** (worst case) |
| **M3** | Revolute (wrist) | **8.2°** | **Orientation only**, zero position | **0 mm** position, **8.2°** heading |
| **M4** | Prismatic (Z) | **1°** | Pure Z, via 13 mm pulley | **0.113 mm** |

---

## Combined Worst-Case Cartesian Error

When both M1 and M2 are both at maximum error simultaneously (errors add in the worst pose):

```
ΔX_worst = ΔXY(M1) + ΔXY(M2)
         = 1.83 + 0.63
         = 2.46 mm  (XY plane)

ΔZ_worst = 0.113 mm

ΔOrientation_worst = 8.2°
```

> [!NOTE]
> The 2.46 mm X/Y number is a **worst-case bound** — it occurs only when both links are fully extended in the **same direction** (straight-arm singular pose). In typical SCARA working poses with the elbow bent, the actual error is smaller.

---

## Best-Case Cartesian Error (Elbow at 90°)

When θ₁ = 90° and θ₂ = 90° (a typical bent-arm working pose), the Jacobian columns are approximately orthogonal. The X and Y components partially cancel:

```
ΔX(M1) ≈ L1 × Δθ₁ = 230 × 0.00524 ≈ 1.20 mm
ΔX(M2) ≈ 0 (Jacobian ≈ orthogonal to X at this pose)

Net ΔXY ≈ √(1.20² + 0.63²) ≈ 1.37 mm
```

---

## Recommendations

| Issue | Recommendation |
|---|---|
| **M1/M2 — 1.83 mm max X/Y** | This is the fundamental angular resolution floor. Only improvable by increasing microstepping beyond 1600 or adding a gearbox. |
| **M4 — 0.113 mm Z** | Excellent. Z-axis resolution is very good (sub-0.2 mm). |
| **M3 — 8.2° orientation** | This is critically poor. Replace the 44 CPR encoder with a higher-resolution one (e.g., 500 P/R = 2000 CPR → 0.18°/count). This would also affect pick accuracy for oriented parts. |
| **Straighten-arm pose** | Avoid commanding targets near the fully-extended singular pose to prevent hitting the 2.46 mm worst-case. |
