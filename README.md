# ELEC4632 - Control Systems Design
UNSW School of Electrical Engineering & Telecommunications

MATLAB work for the ELEC4632 water-tank labs (system identification and control design).

## Lab1 - Linear least squares & system identification
- `ELEC4632_Lab1_Prelab.m` - MATLAB refresher (sinusoidal data plotting)
- `ELEC4632_Lab1_Exercise.m` - second-order model identification and verification
- `ELEC4632_Lab1_Postlab.m` - first- vs second-order model comparison (MSE)
- `SysIdenData_StudentVersion.mat` - pre-collected water-tank data

## Lab2 - Identifying a linear model for the water-tank system
- `ELEC4632_Lab2_Exercise.m` - rig settings (u_offset, input range, stair period), data cut-off, second-order model identification and verification
- `figures/` - raw, cut-off and offset-free data, model verification

## Lab5 - Discrete-time PI control for set-point control
- `ELEC4632_Lab5_Prelab.md` - pre-lab derivations
- `ELEC4632_Lab5_SysID.m` - system identification (Lab 2 procedure)
- `Lab5_PI_Design.slx` - PI design and simulation in Simulink (Kp = 1.30, Ki = 0.045)
- `ELEC4632_Lab5_RealTime_Plot.m` - plot the real-time run against the simulation

See `Lab5/README.md` for results and run order.
