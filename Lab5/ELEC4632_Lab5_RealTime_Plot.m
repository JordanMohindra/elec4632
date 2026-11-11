%% ELEC4632 Lab 5 - Part 3d: plot the real-time PI run against the simulation
% Run after the real-time test. Extracts the recorded data as in the lab
% notes, simulates Lab5_PI_Design.slx with the same Kp / Ki for the same
% length of time, and plots both like Fig. 4 of the notes.
%
% Uses the Kp, Ki and offsets in the workspace (the ones the real-time model
% ran with). The 2026 real-time model logs the scope as ScopeData1; the notes
% call it PIDLogData. Either name works.

close all;

DATA_FILE = '';     % '' = newest PIDControlData*.mat here or in Documents/MATLAB
SAVE_FIG  = true;

%% load the recorded run
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

treal = PIDLogData.time;
yref  = PIDLogData.signals(1).values(:,1);
yreal = PIDLogData.signals(1).values(:,2);
ureal = PIDLogData.signals(2).values;

%% simulate the design model for the same length of time
if ~exist('a1', 'var') || ~exist('u_offset', 'var'), load('Lab5_Model.mat'); end
if ~exist('Kp', 'var') || ~exist('Ki', 'var')
    error('Set Kp and Ki in the workspace to the gains used in the real-time run.');
end
out  = sim('Lab5_PI_Design', 'StopTime', num2str(treal(end)));
ysim = interp1(out.tout, out.y_sl,  treal, 'previous', 'extrap') + y_offset;
usim = interp1(out.tout, out.us_sl, treal, 'previous', 'extrap') + u_offset;

%% Fig. 4 style plot
fig = figure('Name', 'Real-time vs simulation', 'Color', 'w');
subplot(2,1,1);
plot(treal, yref, 'g'); hold on;
plot(treal, ysim, 'b');
plot(treal, yreal, 'r'); hold off;
title({'PI Control Results', 'Output Signal'});
xlabel({'Time (sec)', '(a)'}); ylabel('Water Level (V)'); grid on; ylim([0 5]);
legend('Reference Output', 'Simulated Output', 'Actual Output', 'Location', 'northeast');

subplot(2,1,2);
plot(treal, usim, 'b'); hold on;
plot(treal, ureal, 'r');
plot(treal([1 end]), [Vmax Vmax], 'k-.', treal([1 end]), [Vmin Vmin], 'k-.'); hold off;
text(treal(round(end*0.25)), Vmax - 0.12, sprintf('V_{max} = %.1f', Vmax));
text(treal(round(end*0.25)), Vmin + 0.12, sprintf('V_{min} = %.1f', Vmin));
title('Control Input Signal'); xlabel({'Time (sec)', '(b)'}); ylabel('Pump Voltage (V)');
legend('Simulated Control Input', 'Actual Control Input', 'Location', 'northeast');
grid on; ylim([Vmin - 0.3, Vmax + 0.3]);

if SAVE_FIG
    if ~exist('figures', 'dir'), mkdir('figures'); end
    exportgraphics(fig, fullfile('figures', 'P3_RealTime_vs_Simulation.png'), 'Resolution', 150);
    fprintf('Figure saved to %s\n', fullfile(pwd, 'figures', 'P3_RealTime_vs_Simulation.png'));
end
