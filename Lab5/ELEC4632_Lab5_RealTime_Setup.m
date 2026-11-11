%% ELEC4632 Lab 5 - Part 3a/3b: set up the real-time PI model (run on the LAB PC)
% Needs Simulink and Simulink Desktop Real-Time, which the lab PCs have.
%
%  1. If you repeat system identification on the day (Part 1), set DATA_FILE
%     (and Vmin if you measured it) in ELEC4632_Lab5_Exercise.m and run it
%     first, so Lab5_Design.mat holds today's model, offsets and gains.
%  2. Run this script. It
%       - puts Kp, Ki, u_offset and y_offset in the base workspace (the
%         Discrete PID Controller block reads P = Kp, I = Ki, and the
%         Constant blocks read u_offset / y_offset from there),
%       - sets the Saturation TANK#1 limits to your Vmin / Vmax,
%       - sets the run time to 5*140*0.75 = 525 s.
%  3. Build / Connect To Target / Run (External mode) as in Lab 4.
%  4. When the run stops, the model's StopFcn saves the scope data
%     (variable ScopeData1) as PIDControlData0.mat in the current folder.
%     Then run ELEC4632_Lab5_RealTime_Plot.m.

clear; clc;

MODEL      = 'WaterTankSysControlPID2026';
RUN_TIME   = 5*140*0.75;      % [s] = 525 s, as in the lab notes
SAVE_MODEL = false;           % true = save the settings into the .slx

D = load('Lab5_Design.mat');
Kp       = D.Kp;
Ki       = D.Ki;
u_offset = D.u_offset;
y_offset = D.y_offset;
Vmax     = D.Vmax;
Vmin     = D.Vmin;

open_system(MODEL);

sat1 = [MODEL '/Saturation' newline 'TANK#1'];
set_param(sat1, 'UpperLimit', num2str(Vmax), 'LowerLimit', num2str(Vmin));
set_param(MODEL, 'StopTime', num2str(RUN_TIME));

pidBlk = [MODEL '/Discrete PID Controller'];
fprintf('Real-time model %s is set up:\n', MODEL);
fprintf('  PID block:      P = %s, I = %s, D = %s, Ts = %s s, integrator = %s\n', ...
        get_param(pidBlk, 'P'), get_param(pidBlk, 'I'), get_param(pidBlk, 'D'), ...
        get_param(pidBlk, 'SampleTime'), get_param(pidBlk, 'IntegratorMethod'));
fprintf('  Workspace:      Kp = %.3f, Ki = %.4f, u_offset = %.4f V, y_offset = %.4f V\n', ...
        Kp, Ki, u_offset, y_offset);
fprintf('  Saturation #1:  %s V .. %s V\n', get_param(sat1, 'LowerLimit'), get_param(sat1, 'UpperLimit'));
fprintf('  Stop time:      %s s\n', get_param(MODEL, 'StopTime'));
fprintf('Data will be saved to %s when the run stops.\n', fullfile(pwd, 'PIDControlData0.mat'));

if SAVE_MODEL
    save_system(MODEL); %#ok<UNRCH>
    fprintf('Model saved.\n');
end
