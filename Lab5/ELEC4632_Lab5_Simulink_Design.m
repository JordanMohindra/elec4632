%% ELEC4632 Lab 5 - Part 2 in Simulink: build, tune and simulate the PI loop
% Builds Lab5_PI_Design.slx (Fig. 2 style, transfer-function plant):
%
%   Repeating Sequence Stair -> Rate Transition -> (+)-> Discrete PID (PI)
%        -> Saturation [umin, umax] -> G(z) = (b1 z + b2)/(z^2 + a1 z + a2) --+
%                                       (-) <-----------------------------------+
%
% The blocks use the same settings as the real-time model
% WaterTankSysControlPID2026.slx: PID block parallel form, Forward Euler,
% Ts = 0.75 s; reference [0.7 0.2 -0.5 0] every 140*0.75 s through a Rate
% Transition (which holds 0 for the first 105 s). Zero initial conditions.
%
% Then it repeats the trial-and-error tuning from the notes in Simulink
% (start at Kp = 0.5, Ki = 0.01 and step Kp by 0.05s / Ki by 0.005s),
% simulates the chosen gains, and checks the Simulink result against the
% MATLAB loop in ELEC4632_Lab5_Exercise.m.
%
% Needs Simulink. Run ELEC4632_Lab5_Exercise.m first (makes Lab5_Design.mat).

clear; clc; close all;

MDL       = 'Lab5_PI_Design';
SAVE_FIGS = true;
FIG_DIR   = fullfile(pwd, 'figures');

% trial-and-error path (Kp, Ki): notes start -> ... -> chosen, plus the two
% "one step too far" cases that break the spec
TRIALS = [0.50 0.010;
          0.70 0.020;
          0.90 0.030;
          1.10 0.040;
          1.30 0.045;     % chosen
          1.30 0.055;     % Ki too big   -> overshoot > 2 %
          1.45 0.050];    % Kp too big   -> control input leaves its limits

D = load('Lab5_Design.mat');
Ts = D.Ts;  N_LEVEL = D.N_LEVEL;  RUN_TIME = D.N_PERIODS*N_LEVEL*Ts;

%% ---------------- build the model ---------------------------------------
if bdIsLoaded(MDL), close_system(MDL, 0); end
if exist([MDL '.slx'], 'file'), delete([MDL '.slx']); end
new_system(MDL);
open_system(MDL);

add_block('simulink/Sources/Repeating Sequence Stair', [MDL '/Reference'], ...
    'OutValues', mat2str(D.REF_LEVELS(:)), 'tsamp', sprintf('%d*Ts', N_LEVEL), ...
    'Position', [30 88 80 132]);
add_block('simulink/Signal Attributes/Rate Transition', [MDL '/Rate Transition'], ...
    'OutPortSampleTime', 'Ts', 'Position', [120 90 165 130]);
add_block('simulink/Math Operations/Sum', [MDL '/Sum'], ...
    'Inputs', '+-', 'IconShape', 'round', 'Position', [215 100 235 120]);
add_block('simulink/Discrete/Discrete PID Controller', [MDL '/PI Controller'], ...
    'Controller', 'PI', 'P', 'Kp', 'I', 'Ki', 'SampleTime', 'Ts', ...
    'IntegratorMethod', 'Forward Euler', 'Position', [280 88 360 132]);
add_block('simulink/Discontinuities/Saturation', [MDL '/Saturation'], ...
    'UpperLimit', 'umax', 'LowerLimit', 'umin', 'Position', [410 90 450 130]);
add_block('simulink/Discrete/Discrete Transfer Fcn', [MDL '/Water Tank G(z)'], ...
    'Numerator', '[b1 b2]', 'Denominator', '[1 a1 a2]', 'SampleTime', 'Ts', ...
    'Position', [500 85 620 135]);
add_block('simulink/Signal Routing/Mux', [MDL '/Mux'], 'Inputs', '2', ...
    'Position', [690 30 695 80]);
add_block('simulink/Sinks/Scope', [MDL '/Scope Output'], 'Position', [730 40 760 70]);
add_block('simulink/Sinks/Scope', [MDL '/Scope Input'], 'Position', [500 180 530 210]);
names = {'r_sl', 'u_sl', 'us_sl', 'y_sl'};
pos   = [200 240; 400 240; 500 290; 700 140];
for q = 1:4
    add_block('simulink/Sinks/To Workspace', [MDL '/' names{q}], 'VariableName', names{q}, ...
        'SaveFormat', 'Array', 'Position', [pos(q,1) pos(q,2) pos(q,1)+60 pos(q,2)+25]);
end

add_line(MDL, 'Reference/1',        'Rate Transition/1', 'autorouting', 'on');
add_line(MDL, 'Rate Transition/1',  'Sum/1',             'autorouting', 'on');
add_line(MDL, 'Sum/1',              'PI Controller/1',   'autorouting', 'on');
add_line(MDL, 'PI Controller/1',    'Saturation/1',      'autorouting', 'on');
add_line(MDL, 'Saturation/1',       'Water Tank G(z)/1', 'autorouting', 'on');
add_line(MDL, 'Water Tank G(z)/1',  'Sum/2',             'autorouting', 'on');
add_line(MDL, 'Rate Transition/1',  'Mux/1',             'autorouting', 'on');
add_line(MDL, 'Water Tank G(z)/1',  'Mux/2',             'autorouting', 'on');
add_line(MDL, 'Mux/1',              'Scope Output/1',    'autorouting', 'on');
add_line(MDL, 'Saturation/1',       'Scope Input/1',     'autorouting', 'on');
add_line(MDL, 'Rate Transition/1',  'r_sl/1',            'autorouting', 'on');
add_line(MDL, 'PI Controller/1',    'u_sl/1',            'autorouting', 'on');
add_line(MDL, 'Saturation/1',       'us_sl/1',           'autorouting', 'on');
add_line(MDL, 'Water Tank G(z)/1',  'y_sl/1',            'autorouting', 'on');

set_param(MDL, 'Solver', 'FixedStepDiscrete', 'FixedStep', 'Ts', ...
               'StopTime', num2str(RUN_TIME), 'SaveTime', 'on', 'ReturnWorkspaceOutputs', 'on');

% parameters live in the model workspace, so the .slx works on its own
hws = get_param(MDL, 'ModelWorkspace');
vars = {'Ts', Ts; 'Kp', D.Kp; 'Ki', D.Ki; 'umin', D.umin; 'umax', D.umax; ...
        'a1', D.a1; 'a2', D.a2; 'b1', D.b1; 'b2', D.b2};
for q = 1:size(vars,1), assignin(hws, vars{q,1}, vars{q,2}); end

save_system(MDL);
fprintf('Built and saved %s.slx\n', MDL);

%% ---------------- trial-and-error tuning in Simulink --------------------
nT = size(TRIALS, 1);
in(1:nT) = Simulink.SimulationInput(MDL);
for c = 1:nT
    in(c) = in(c).setVariable('Kp', TRIALS(c,1), 'Workspace', MDL);
    in(c) = in(c).setVariable('Ki', TRIALS(c,2), 'Workspace', MDL);
end
out = sim(in, 'ShowProgress', 'off');

nS = D.N_PERIODS*N_LEVEL;              % compare the same 700 samples as MATLAB
fprintf('\nTrial-and-error tuning (Simulink results)\n');
fprintf('%6s %7s %9s %12s %18s  %s\n', 'Kp', 'Ki', 'max OS%', 'sum ts (s)', 'u range (V)', 'meets spec?');
Ytr = zeros(nS, nT);
for c = 1:nT
    r  = out(c).r_sl(1:nS);   y = out(c).y_sl(1:nS);   u = out(c).u_sl(1:nS);
    m  = stepMetrics(r, y, N_LEVEL, Ts);
    ok = max(m(:,2)) < 2 && max(u) <= D.umax && min(u) >= D.umin && ~any(isnan(m(:,4)));
    tsTxt = sprintf('%.1f', sum(m(:,4)));
    if any(isnan(m(:,4))), tsTxt = 'not settled'; end
    fprintf('%6.2f %7.3f %9.2f %12s   [%6.3f, %6.3f]  %s\n', TRIALS(c,1), TRIALS(c,2), ...
            max(m(:,2)), tsTxt, min(u), max(u), string(ok));
    Ytr(:,c) = y;
end

%% ---------------- chosen gains ------------------------------------------
outC = sim(MDL);                        % model workspace holds the chosen Kp, Ki
t_sl  = outC.tout(1:nS);
r_sl  = outC.r_sl(1:nS);
y_sl  = outC.y_sl(1:nS);
u_sl  = outC.u_sl(1:nS);
us_sl = outC.us_sl(1:nS);
mC = stepMetrics(r_sl, y_sl, N_LEVEL, Ts);
fprintf('\nChosen gains in Simulink: Kp = %.3f, Ki = %.4f\n', D.Kp, D.Ki);
fprintf('  max overshoot = %.2f %%, 2%% settling time per step = %s s\n', max(mC(:,2)), mat2str(mC(:,4).', 4));
fprintf('  control input %.3f .. %.3f V offset-free (limits %.2f .. %.2f)\n', ...
        min(u_sl), max(u_sl), D.umin, D.umax);
fprintf('  Simulink vs MATLAB loop: max |dy| = %.2e, max |du| = %.2e, max |dr| = %.2e\n', ...
        max(abs(y_sl - D.y_sim)), max(abs(us_sl - D.us_sim)), max(abs(r_sl - D.r)));

%% ---------------- figures -----------------------------------------------
figTr = figure('Name','Simulink trial and error','Color','w');
stairs(t_sl, r_sl, 'k', 'LineWidth', 1); hold on;
cols = parula(nT + 1);
lg = {'Reference'};
for c = 1:nT
    plot(t_sl, Ytr(:,c), 'Color', cols(c,:), 'LineWidth', 1 + 1.5*(c == 5));
    lg{end+1} = sprintf('K_p = %.2f, K_i = %.3f', TRIALS(c,1), TRIALS(c,2)); %#ok<SAGROW>
end
hold off; grid on;
xlabel('Time (sec)'); ylabel('Offset-free Water Level (V)');
title('Trial-and-Error PI Tuning in Simulink');
legend(lg, 'Location', 'northeast');

figSl = figure('Name','Simulink PI design (Fig. 4 style)','Color','w');
subplot(2,1,1);
stairs(t_sl, r_sl + D.y_offset, 'g'); hold on; plot(t_sl, y_sl + D.y_offset, 'b'); hold off;
title({'PI Control Results (Simulink)', 'Output Signal'});
xlabel({'Time (sec)', '(a)'}); ylabel('Water Level (V)');
legend('Reference Output', 'Simulated Output'); grid on; ylim([0 5]);
subplot(2,1,2);
stairs(t_sl, us_sl + D.u_offset, 'b'); hold on;
plot(t_sl([1 end]), [D.Vmax D.Vmax], 'k-.', t_sl([1 end]), [D.Vmin D.Vmin], 'k-.'); hold off;
text(t_sl(round(end*0.25)), D.Vmax - 0.12, sprintf('V_{max} = %.1f', D.Vmax));
text(t_sl(round(end*0.25)), D.Vmin + 0.12, sprintf('V_{min} = %.1f', D.Vmin));
title('Control Input Signal'); xlabel({'Time (sec)', '(b)'}); ylabel('Pump Voltage (V)');
legend('Simulated Control Input'); grid on; ylim([D.Vmin - 0.3, D.Vmax + 0.3]);

save('Lab5_Simulink_Results.mat', 't_sl', 'r_sl', 'y_sl', 'u_sl', 'us_sl', 'TRIALS', 'Ytr');

if SAVE_FIGS
    if ~exist(FIG_DIR, 'dir'), mkdir(FIG_DIR); end
    exportgraphics(figTr, fullfile(FIG_DIR, 'P2_Simulink_TrialAndError.png'), 'Resolution', 150);
    exportgraphics(figSl, fullfile(FIG_DIR, 'P2_Simulink_PIDesign_Fig4Style.png'), 'Resolution', 150);
    print(['-s' MDL], '-dpng', '-r150', fullfile(FIG_DIR, 'P2_Simulink_BlockDiagram.png'));
    fprintf('Figures saved to %s\n', FIG_DIR);
end

%% ---------------- local function ----------------------------------------
function m = stepMetrics(r, y, nLevel, h)
% [step size, overshoot %, 10-90% rise time, 2% settling time, final error]
m = [];
for s = 2:numel(r)/nLevel
    idx = (s-1)*nLevel + (1:nLevel)';
    rn  = r(idx(1));   rp = r(idx(1)-1);   d = rn - rp;
    if abs(d) < 1e-12, continue; end
    yy   = y(idx);
    os   = max(0, max(sign(d)*(yy - rn)))/abs(d)*100;
    frac = (yy - rp)/d;
    i10  = find(frac >= 0.1, 1);   i90 = find(frac >= 0.9, 1);
    if isempty(i10) || isempty(i90), tr = NaN; else, tr = (i90 - i10)*h; end
    out = find(abs(yy - rn) > 0.02*abs(d), 1, 'last');
    if isempty(out), ts = 0; elseif out == nLevel, ts = NaN; else, ts = out*h; end
    m = [m; d os tr ts rn - yy(end)]; %#ok<AGROW>
end
end
