%% ELEC4632 Lab 5 - Part 3d: compare the real-time PI run with the simulation
% Run after the real-time test. Loads the recorded data, re-simulates the
% identified model with the same PI gains and the recorded reference, and
% plots both like Fig. 4 of the lab notes.
%
% The notes call the logged structure PIDLogData; the 2026 model's scope
% logs it as ScopeData1. Either name works here.

clear; clc; close all;

%% ===================== USER SETTINGS =====================================
DATA_FILE = '';     % '' = newest PIDControlData*.mat here or in Documents/MATLAB
SAVE_FIGS = true;
FIG_DIR   = fullfile(pwd, 'figures');
%% =========================================================================

if isempty(DATA_FILE)
    files = [dir('PIDControlData*.mat'); dir(fullfile(userpath, 'PIDControlData*.mat'))];
    if isempty(files)
        error(['No PIDControlData*.mat found. Copy the file saved after the real-time ' ...
               'run into this folder or set DATA_FILE.']);
    end
    [~, newest] = max([files.datenum]);
    DATA_FILE = fullfile(files(newest).folder, files(newest).name);
end
S = load(DATA_FILE);
if isfield(S, 'PIDLogData')
    PIDLogData = S.PIDLogData;
else
    fn = fieldnames(S);
    PIDLogData = S.(fn{1});
end
fprintf('Real-time data: %s\n', DATA_FILE);

% ---- extract the data as in the lab notes (Part 3d)
treal = PIDLogData.time;
yref  = PIDLogData.signals(1).values(:,1);
yreal = PIDLogData.signals(1).values(:,2);
ureal = PIDLogData.signals(2).values;
ynr   = [];
if size(PIDLogData.signals(1).values, 2) >= 3
    ynr = PIDLogData.signals(1).values(:,3);    % noise-reduced output (the fed-back one)
end

% ---- simulate the design with the reference that was actually applied
D  = load('Lab5_Design.mat');
h  = D.Ts;
pm = struct('a1', D.a1, 'a2', D.a2, 'b1', D.b1, 'b2', D.b2);
r  = yref - D.y_offset;                          % offset-free reference
[ys, ~, uss] = simPI(pm, D.Kp, D.Ki, h, r, D.umin, D.umax);
ysim = ys  + D.y_offset;
usim = uss + D.u_offset;
fprintf('Kp = %.3f, Ki = %.4f, u_offset = %.4f V, y_offset = %.4f V\n', ...
        D.Kp, D.Ki, D.u_offset, D.y_offset);

% ---- compare after the first period (the start-up transient is not modelled)
nLevel = D.N_LEVEL;
after  = treal >= nLevel*h;
yComp  = yreal;  if ~isempty(ynr), yComp = ynr; end
fprintf('After the first %.0f s:  output MSE = %.5f V^2,  control input MSE = %.5f V^2\n', ...
        nLevel*h, mean((yComp(after) - ysim(after)).^2), mean((ureal(after) - usim(after)).^2));

% overshoot of the real output at each reference change
chg = find(abs(diff(yref)) > 1e-6) + 1;
for q = 1:numel(chg)
    i0 = chg(q);  i1 = min(numel(yref), i0 + nLevel - 1);
    if q < numel(chg), i1 = min(i1, chg(q+1) - 1); end
    d  = yref(i0) - yref(i0-1);
    os = max(0, max(sign(d)*(yComp(i0:i1) - yref(i0))))/abs(d)*100;
    fprintf('  step %+.2f V at t = %5.1f s: real overshoot = %.2f %%\n', d, treal(i0), os);
end

% ---- Fig. 4 style plot
fig = figure('Name','Real-time vs simulation','Color','w');
subplot(2,1,1);
plot(treal, yref, 'g'); hold on;
plot(treal, ysim, 'b');
plot(treal, yreal, 'r');
if ~isempty(ynr), plot(treal, ynr, 'Color', [0.6 0 0]); end
hold off;
title({'PI Control Results', 'Output Signal'});
xlabel({'Time (sec)', '(a)'}); ylabel('Water Level (V)'); grid on; ylim([0 5]);
leg = {'Reference Output', 'Simulated Output', 'Actual Output'};
if ~isempty(ynr), leg{end+1} = 'Actual Output (noise-reduced)'; end
legend(leg, 'Location', 'northeast');

subplot(2,1,2);
plot(treal, usim, 'b'); hold on;
plot(treal, ureal, 'r');
plot(treal([1 end]), [D.Vmax D.Vmax], 'k-.', treal([1 end]), [D.Vmin D.Vmin], 'k-.');
hold off;
text(treal(round(end*0.25)), D.Vmax - 0.12, sprintf('V_{max} = %.1f', D.Vmax));
text(treal(round(end*0.25)), D.Vmin + 0.12, sprintf('V_{min} = %.1f', D.Vmin));
title('Control Input Signal'); xlabel({'Time (sec)', '(b)'}); ylabel('Pump Voltage (V)');
legend('Simulated Control Input', 'Actual Control Input', 'Location', 'northeast');
grid on; ylim([D.Vmin - 0.3, D.Vmax + 0.3]);

if SAVE_FIGS
    if ~exist(FIG_DIR, 'dir'), mkdir(FIG_DIR); end
    exportgraphics(fig, fullfile(FIG_DIR, 'P3_RealTime_vs_Simulation.png'), 'Resolution', 150);
    fprintf('Figure saved to %s\n', fullfile(FIG_DIR, 'P3_RealTime_vs_Simulation.png'));
end

%% ---- local function (same loop as ELEC4632_Lab5_Exercise.m)
function [y, u, us] = simPI(p, Kp, Ki, h, r, umin, umax)
N  = numel(r);
Y  = zeros(N+2, 1);
US = zeros(N+2, 1);
u  = zeros(N, 1);
I  = 0;
for k = 1:N
    K = k + 2;
    Y(K)  = -p.a1*Y(K-1) - p.a2*Y(K-2) + p.b1*US(K-1) + p.b2*US(K-2);
    e     = r(k) - Y(K);
    u(k)  = Kp*e + I;
    US(K) = min(max(u(k), umin), umax);
    I     = I + Ki*h*e;
end
y  = Y(3:end);
us = US(3:end);
end
