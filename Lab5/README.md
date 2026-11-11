# Lab 5 – Discrete-time PI control for set-point control

Data: `SysIdenData_StudentVersion.mat` (same student data set as Lab 1).
Real-time model from Moodle: `WaterTankSysControlPID2026.slx`.

Run in this order:

| Script | What it does |
|---|---|
| `ELEC4632_Lab5_Exercise.m` | Part 1 system identification (Lab 2 method) and Part 2 PI design: gain search, Ziegler–Nichols and `pidtune` comparison, chosen-gain simulation, bonus start-up analysis. Writes `Lab5_Design.mat` and `figures/`. |
| `ELEC4632_Lab5_Prelab_Check.m` | Numerically checks the pre-lab answers in `ELEC4632_Lab5_Prelab.md`. |
| `ELEC4632_Lab5_Simulink_Design.m` | Part 2 in Simulink: builds `Lab5_PI_Design.slx`, repeats the trial-and-error tuning in Simulink, simulates the chosen gains and checks them against the MATLAB loop. Needs Simulink. |
| `ELEC4632_Lab5_RealTime_Setup.m` | Part 3a/3b, on the lab PC: loads the gains and offsets into the workspace, sets Saturation TANK#1 to Vmin/Vmax and the run time to 525 s. |
| `ELEC4632_Lab5_RealTime_Plot.m` | Part 3d, after the run: plots the recorded `PIDControlData*.mat` against the simulation (Fig. 4 style). |

## Results with the student data

- Model: G(z) = (−0.00653 z + 0.05094)/(z² − 1.3772 z + 0.3919), Ts = 0.75 s, poles 0.975 and 0.402, DC gain 3.03.
- Offsets: u_offset = 2.000 V, y_offset = 2.353 V. Pump limits Vmax = 3.0 V, Vmin = 1.0 V (from Lab 2 Eq. 1 with u_offset = 2.0 V), so −1 V ≤ u(k) ≤ 1 V offset-free.
- Chosen PI gains: **Kp = 1.30, Ki = 0.045**. Overshoot 0.5 % (spec < 2 %), 2 % settling time 22.5 s per step, control input 1.13–2.94 V (inside 1.0–3.0 V), gain margin 19.1 dB, phase margin 76°.
- The fastest pair that still meets the spec is Kp = 1.35, Ki = 0.050, but it uses 98 % of the input range and sits closer to the overshoot limit, so the slightly slower pair was kept for margin on the real rig.

## Things that differ from the lab notes

- The 2026 real-time model logs the scope as `ScopeData1` (the notes say `PIDLogData`) and saves it as `PIDControlData0.mat` in the current folder. The plot script accepts either name.
- Its stop time is 2000 s, not 525 s; the setup script sets it to 525 s.
- The default Saturation TANK#1 limits are 2.0–3.0 V. With u_offset = 2.0 V that would make the −0.5 V level unreachable (it needs about 1.83 V), so set your rig's real Vmin.
- Vmin/Vmax and the offsets above are for the student data set. Re-run `ELEC4632_Lab5_Exercise.m` with your own `SysIdenData_x.mat` on the day.
