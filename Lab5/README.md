# Lab 5 – Discrete-time PI control for set-point control

| Part | File | |
|---|---|---|
| Pre-lab | `ELEC4632_Lab5_Prelab.md` | Answers to (a)–(c) |
| 1 – System identification | `ELEC4632_Lab5_SysID.m` | Lab 2 procedure on `SysIdenData_StudentVersion.mat`; plots the Lab 1/2 figures and saves the model, offsets and pump limits to `Lab5_Model.mat` |
| 2 – PI design | `Lab5_PI_Design.slx` | Simulink model: PI controller → saturation → identified G(z), with the lab's reference signal |
| 3 – Real-time | `WaterTankSysControlPID2026.slx` | Real-time model from Moodle (lab PC only) |
| 3d – Plot | `ELEC4632_Lab5_RealTime_Plot.m` | Plots the recorded run against `Lab5_PI_Design.slx` (Fig. 4 style) |

## Results with the student data

- Model: G(z) = (−0.00653 z + 0.05094)/(z² − 1.3772 z + 0.3919), Ts = 0.75 s.
- u_offset = 2.000 V, y_offset = 2.353 V. Pump limits Vmax = 3.0 V, Vmin = 1.0 V (Lab 2 Eq. 1 with u_offset = 2.0 V), so −1 V ≤ u(k) ≤ 1 V offset-free.
- PI gains: **Kp = 1.30, Ki = 0.045**. Overshoot 0.5 % (spec < 2 %), 2 % settling time 22.5 s per step, pump voltage 1.13–2.94 V.

Trial-and-error tuning in Simulink, starting from the notes' Kp = 0.5, Ki = 0.01 (`figures/P2_Simulink_TrialAndError.png`):

| Kp | Ki | Overshoot | Result |
|---|---|---|---|
| 0.50 | 0.010 | 0 % | Too slow – first level never settles |
| 0.70 | 0.020 | 0 % | Settling 289 s total |
| 0.90 | 0.030 | 0.09 % | Settling 147 s total |
| 1.10 | 0.040 | 1.01 % | Settling 102 s total |
| **1.30** | **0.045** | **0.50 %** | **Chosen – settling 90 s total** |
| 1.30 | 0.055 | 3.05 % | Ki too big – fails 2 % spec |
| 1.45 | 0.050 | 0.56 % | Kp too big – pump would need 3.05 V (> Vmax) |

## How to run

1. Run `ELEC4632_Lab5_SysID.m` (on the day, set `DATA_FILE` to your `SysIdenData_x.mat` and type in your Vmin).
2. Open `Lab5_PI_Design.slx` and press Run. It uses Kp = 1.30, Ki = 0.045 unless `Kp` / `Ki` are already set in the workspace — set them there to try other gains.
3. Real-time run (lab PC): with Kp, Ki, u_offset and y_offset in the workspace, open `WaterTankSysControlPID2026.slx`, set Saturation TANK#1 to your Vmin/Vmax, set the stop time to 525 s, and run it.
4. Copy the saved `PIDControlData*.mat` into this folder and run `ELEC4632_Lab5_RealTime_Plot.m`.

## Things that differ from the lab notes

- The 2026 real-time model logs the scope as `ScopeData1` (not `PIDLogData`) and saves it as `PIDControlData0.mat` in the current folder. The plot script accepts either.
- Its stop time is 2000 s, not 525 s.
- Its default Saturation TANK#1 limits are 2.0–3.0 V. With u_offset = 2.0 V the −0.5 V level would be unreachable (it needs about 1.83 V), so set your rig's real Vmin.

## Bonus: why the real run saturates at the start

The simulation starts from zero initial conditions (tank already at y_offset). The real tank starts nearly empty, so in the first period the error is about 0 − (0 − 2.35) = 2.35 V and Kp·e asks for about 5 V. The pump sits at Vmax while the tank fills. Meanwhile the integrator keeps adding up that large error (the PID block has no anti-windup), so when the level reaches the set-point the integral term is too big and the level overshoots before it unwinds. The tank is also far from the operating point the model was identified at, so that part of the run won't match the simulation. Pre-filling the tank, turning on anti-windup (clamping), or setting the integrator's initial condition would fix it.
