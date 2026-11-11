%% ELEC4632 Lab 5 - Discrete-time PI control for set-point control
% Lab Exercise parts 1 and 2. Part 3 is done on the rig: see
% ELEC4632_Lab5_RealTime_Setup.m (before the run) and
% ELEC4632_Lab5_RealTime_Plot.m (after the run).
%
%   Part 1: system identification, same procedure as Lab 2
%   Part 2: PI design and simulation
%
% The loop simulated here is the same discrete loop that the Simulink
% models run (WaterTankSysControlPID2026.slx and Lab5_PI_Design.slx):
%   Discrete PID Controller block (parallel form, Forward Euler,
%   Ts = 0.75 s, D = 0)  ->  + u_offset  ->  Saturation [Vmin, Vmax]
%   ->  water tank, with (noise-reduced output - y_offset) fed back.
% The reference goes through a Rate Transition block in the real-time
% model, which holds 0 for the first 105 s, so the simulated reference is
% {0, 0.7, 0.2, -0.5, 0} for 5 x 105 s = 525 s.
%
% Saves everything the other Lab 5 scripts need to Lab5_Design.mat.

clear; clc; close all;

%% ===================== USER SETTINGS =====================================
DATA_FILE = 'SysIdenData_StudentVersion.mat';   % <-- your SysIdenData_x.mat

% Pump voltage limits of YOUR rig (Saturation TANK#1 block).
% Lab 2 Eq. (1): u_offset = (Vmax + Vmin)/2 with Vmax = 3.0 V fixed by the
% lab. This data set has u_offset = 2.0 V, so Vmin = 2*u_offset - Vmax = 1.0 V.
% If you measured Vmin on the day, type it in instead of [].
Vmax = 3.0;            % [V]
Vmin = [];             % [V]  [] = derive from u_offset with Lab 2 Eq. (1)

% Offset-free reference levels and timing (lab notes, part 2a)
REF_LEVELS = [0.7 0.2 -0.5 0];
N_LEVEL    = 140;      % samples per level -> 140*0.75 = 105 s
N_PERIODS  = 5;        % 5*105 = 525 s, same length as the real-time run

OS_MAX  = 2;                  % [%] overshoot spec
Kp_grid = 0.05:0.05:2.00;     % step sizes suggested in the notes
Ki_grid = 0.005:0.005:0.30;

% Chosen design (reasoning is printed at the end of Part 2).
% Set both to [] to use the fastest feasible pair from the grid search.
Kp_design = 1.30;
Ki_design = 0.045;

SAVE_FIGS = true;
FIG_DIR   = fullfile(pwd, 'figures');
%% =========================================================================

%% ======================= PART 1: SYSTEM IDENTIFICATION ===================
fprintf('==================== PART 1: SYSTEM IDENTIFICATION ====================\n');
S  = load(DATA_FILE);
fn = fieldnames(S);
LogData = S.(fn{1});

t_raw  = LogData.time(:);
y_meas = LogData.signals(1).values(:,1);   % measured output
y_nr   = LogData.signals(1).values(:,2);   % noise-reduced output (fed back in the loop)
u_raw  = LogData.signals(2).values(:);     % actual input
Ts     = mean(diff(t_raw));
N_raw  = numel(t_raw);
fprintf('Data: %s  (%d samples, %.1f s, Ts = %.2f s)\n', DATA_FILE, N_raw, t_raw(end), Ts);

% ---- input offset and pump limits
u_offset = u_raw(1);                       % first period = u_offset only
if isempty(Vmin), Vmin = 2*u_offset - Vmax; end
umin = Vmin - u_offset;
umax = Vmax - u_offset;
fprintf('u_offset = %.4f V   ->   Vmin = %.2f V, Vmax = %.2f V\n', u_offset, Vmin, Vmax);
fprintf('Offset-free control input limits: %.3f V <= u(k) <= %.3f V\n', umin, umax);

% ---- constant-input segments and their settled output levels
chg    = find(abs(diff(u_raw)) > 1e-6);
sStart = [1; chg+1];
sEnd   = [chg; N_raw];
nSeg   = numel(sStart);
isOff  = abs(u_raw(sStart) - u_offset) < 1e-6;
offSeg = find(isOff);
sSettle = sStart;
sLevel  = nan(nSeg,1);
for s = 1:nSeg
    idx   = (sStart(s):sEnd(s))';
    final = mean(y_nr(idx(max(1, round(0.75*numel(idx))):end)));
    out   = find(abs(y_nr(idx) - final) > 0.05, 1, 'last');
    if ~isempty(out), sSettle(s) = min(idx(end), idx(out) + 1); end
    sLevel(s) = mean(y_nr(sSettle(s):sEnd(s)));
end

% ---- Lab 2 step 5a: decide what to cut off
seqLen   = 3*median(sEnd(2:end-1) - sStart(2:end-1) + 1);
offLater = [];
for s = offSeg(2:end)'
    if sEnd(s) - sSettle(s) > 10, offLater(end+1) = sLevel(s); end %#ok<SAGROW>
end
kCut = 1;  cutTxt = 'nothing (no transient, offsets consistent)';
if ~isempty(offLater) && abs(sLevel(offSeg(1)) - mean(offLater)) > 0.05 && ...
        numel(offSeg) >= 2 && (N_raw - sSettle(offSeg(2)) + 1) >= seqLen
    kCut = sSettle(offSeg(2));   cutTxt = 'first sequence';
elseif abs(y_nr(1) - sLevel(offSeg(1))) > 0.05
    kCut = sSettle(offSeg(1));   cutTxt = 'initial transient';
end
fprintf('Cut off: %s -> keep samples %d..%d\n', cutTxt, kCut, N_raw);

keep   = (kCut:N_raw)';
t      = t_raw(keep) - t_raw(kCut);
y_act  = y_nr(keep);
y_actm = y_meas(keep);
u_act  = u_raw(keep);

% ---- output offset: mean of the settled output over every u_offset segment
offIdx = [];
for s = offSeg'
    a = max(sSettle(s), kCut);  b = sEnd(s);
    if b >= a, offIdx = [offIdx; (a:b)' - kCut + 1]; end %#ok<AGROW>
end
y_offset = mean(y_act(offIdx));
fprintf('y_offset = %.4f V (mean over %d settled samples at u = u_offset)\n', ...
        y_offset, numel(offIdx));

u = u_act - u_offset;
y = y_act - y_offset;

% ---- least squares, second-order model (Lab 1 Eq. 4-8)
N     = numel(u);
Nhalf = floor(N/2);
k     = (10:Nhalf)';
Phi   = [y(k-1) y(k-2) u(k-1) u(k-2)];
theta = (Phi'*Phi) \ (Phi'*y(k));
a1 = -theta(1);  a2 = -theta(2);  b1 = theta(3);  b2 = theta(4);

G_tf = tf([b1 b2], [1 a1 a2], Ts);
Gm = [0 1; -a2 -a1];   Hm = [0; 1];   Cm = [b2 b1];   Dm = 0;
G_ss = ss(Gm, Hm, Cm, Dm, Ts);

fprintf('\nIdentified model:  a1 = %.6f  a2 = %.6f  b1 = %.6f  b2 = %.6f\n', a1, a2, b1, b2);
G_tf %#ok<NOPTS>
G_ss %#ok<NOPTS>
fprintf('Poles: %s    DC gain: %.4f V/V\n', mat2str(pole(G_tf).', 5), dcgain(G_tf));

% ---- verification
u_2nd = u(Nhalf+1:end);   y_2nd = y(Nhalf+1:end);
t_2nd = (0:numel(u_2nd)-1)' * Ts;
y_sim_2nd  = lsim(G_tf, u_2nd, t_2nd);
t_full     = (0:N-1)' * Ts;
y_sim_full = lsim(G_tf, u, t_full);
fprintf('MSE (entire) = %.6f V^2\n', mean((y - y_sim_full).^2));

% ---- Part 1 figures (the ones asked for in Labs 1 and 2)
figRaw = figure('Name','Raw data','Color','w');
subplot(2,1,1);
plot(t_raw, y_nr, 'b'); hold on; plot(t_raw, y_meas, 'r'); hold off;
title('Actual Output Signal'); xlabel('Time (sec)'); ylabel('Water Level (V)');
legend('Noise-Reduced Output', 'Measured Output'); grid on;
subplot(2,1,2);
plot(t_raw, u_raw, 'b');
title('Actual Input Signal'); xlabel('Time (sec)'); ylabel('Pump Voltage (V)');
legend('Actual Input'); grid on;

figOff = figure('Name','Offset-free data','Color','w');
subplot(2,1,1);
plot(t, y, 'r'); title('Actual Offset-Free Output Signal');
xlabel('Time (sec)'); ylabel('Water Level (V)'); legend('Actual Output'); grid on;
subplot(2,1,2);
plot(t, u, 'Color', [0.3 0.6 0.9]); title('Actual Offset-Free Input Signal');
xlabel('Time (sec)'); ylabel('Pump Voltage (V)'); legend('Actual Input'); grid on;

figVer = figure('Name','Model verification','Color','w');
subplot(2,1,1);
plot(t_2nd, y_2nd, 'r'); hold on; plot(t_2nd, y_sim_2nd, 'b--'); hold off;
title('Offset-Free Model Verification (2nd Half)');
xlabel('Time (sec)'); ylabel('Water Level (V)');
legend('Actual Output', 'Simulated Output'); grid on;
subplot(2,1,2);
plot(t_full, y, 'r'); hold on; plot(t_full, y_sim_full, 'b--'); hold off;
title('Offset-Free Model Verification (Entire)');
xlabel('Time (sec)'); ylabel('Water Level (V)');
legend('Actual Output', 'Simulated Output'); grid on;

%% ======================= PART 2: PI CONTROL DESIGN =======================
fprintf('\n==================== PART 2: PI CONTROL DESIGN ====================\n');
h  = Ts;
pm = struct('a1', a1, 'a2', a2, 'b1', b1, 'b2', b2);

% reference as seen by the controller (Rate Transition holds 0 first)
lev   = [0, repmat(REF_LEVELS, 1, ceil(N_PERIODS/numel(REF_LEVELS)))];
lev   = lev(1:N_PERIODS);
r     = kron(lev(:), ones(N_LEVEL,1));
t_sim = (0:numel(r)-1)' * h;
fprintf('Reference levels per 105 s period: %s\n', mat2str(lev));

% ---- grid search (trial and error, done systematically)
nKp = numel(Kp_grid);  nKi = numel(Ki_grid);
OSmap = nan(nKp,nKi);  TSmap = OSmap;  UHI = OSmap;  ULO = OSmap;
for i = 1:nKp
    for j = 1:nKi
        [yy, uu] = simPI(pm, Kp_grid(i), Ki_grid(j), h, r, umin, umax);
        m = stepMetrics(r, yy, N_LEVEL, h);
        OSmap(i,j) = max(m(:,2));
        TSmap(i,j) = sum(m(:,4));           % NaN if a level never settles
        UHI(i,j)   = max(uu);
        ULO(i,j)   = min(uu);
    end
end
feasible = OSmap < OS_MAX & UHI <= umax & ULO >= umin & ~isnan(TSmap);
cost = TSmap + 1e-3*OSmap;   cost(~feasible) = inf;
[~, ix] = min(cost(:));   [bi, bj] = ind2sub(size(cost), ix);
Kp_fast = Kp_grid(bi);   Ki_fast = Ki_grid(bj);
fprintf('Grid: %d x %d pairs, %d feasible (OS < %g%%, u inside limits, every level settles)\n', ...
        nKp, nKi, nnz(feasible), OS_MAX);
fprintf('Fastest feasible pair: Kp = %.2f, Ki = %.3f\n', Kp_fast, Ki_fast);

if isempty(Kp_design) || isempty(Ki_design)
    Kp = Kp_fast;  Ki = Ki_fast;
else
    Kp = Kp_design;  Ki = Ki_design;
end

% ---- Ziegler-Nichols step-response method (Appendix, Table A1)
t_st = (0:600)' * h;
y_st = step(G_tf, t_st);
[R, iR] = max(diff(y_st)/h);                 % steepest slope
L_zn  = t_st(iR) - y_st(iR)/R;               % tangent meets the time axis
a_zn  = R*L_zn;
Kp_zn = 0.9/a_zn;   Ti_zn = 3*L_zn;   Ki_zn = Kp_zn/Ti_zn;
fprintf('Ziegler-Nichols: R = %.4f V/s, L = %.2f s, a = RL = %.4f -> Kp = %.3f, Ti = %.2f s, Ki = %.4f\n', ...
        R, L_zn, a_zn, Kp_zn, Ti_zn, Ki_zn);
fprintf(['   Note: Z-N assumes a process with a real dead time. This tank has almost\n' ...
         '   none (L is only about one sample), so a = RL is tiny and the rule gives\n' ...
         '   gains that are far too aggressive here - see the table below.\n']);

% ---- MATLAB automatic tuning (same engine as the PID block's tuner)
C_pt = pidtune(G_tf, 'PI');
fprintf('pidtune: Kp = %.4f, Ki = %.4f (IFormula = %s)\n', C_pt.Kp, C_pt.Ki, C_pt.IFormula);
fprintf(['   Note: pidtune balances speed against robustness with a ~60 deg phase-margin\n' ...
         '   target; it does not know about the 2%% overshoot spec or the input limits.\n']);

% ---- comparison table
cand = {'Notes starting point',          0.5,     0.01;
        'Notes Fig. 4 example',          0.68,    0.03;
        'Ziegler-Nichols step response', Kp_zn,   Ki_zn;
        'pidtune (auto)',                C_pt.Kp, C_pt.Ki;
        'Fastest feasible (grid)',       Kp_fast, Ki_fast;
        'CHOSEN DESIGN',                 Kp,      Ki};
fprintf('\n%-31s %7s %7s %9s %12s %18s  %s\n', 'Gains', 'Kp', 'Ki', 'max OS%', 'sum ts (s)', 'u range (V)', 'meets spec?');
candY = cell(size(cand,1),1);
for c = 1:size(cand,1)
    [yy, uu] = simPI(pm, cand{c,2}, cand{c,3}, h, r, umin, umax);
    m  = stepMetrics(r, yy, N_LEVEL, h);
    ok = max(m(:,2)) < OS_MAX && max(uu) <= umax && min(uu) >= umin && ~any(isnan(m(:,4)));
    tsTxt = sprintf('%.1f', sum(m(:,4)));
    if any(isnan(m(:,4))), tsTxt = 'not settled'; end
    fprintf('%-31s %7.3f %7.4f %9.2f %12s   [%6.3f, %6.3f]  %s\n', cand{c,1}, cand{c,2}, cand{c,3}, ...
            max(m(:,2)), tsTxt, min(uu), max(uu), string(ok));
    candY{c} = yy;
end

% ---- chosen design
[y_sim, u_sim, us_sim] = simPI(pm, Kp, Ki, h, r, umin, umax);
mC = stepMetrics(r, y_sim, N_LEVEL, h);
fprintf('\nChosen PI controller: Kp = %.3f, Ki = %.4f\n', Kp, Ki);
fprintf('%10s %8s %10s %10s %12s\n', 'step (V)', 'OS (%)', 'tr (s)', 'ts 2% (s)', 'ess (V)');
for q = 1:size(mC,1)
    fprintf('%+10.2f %8.2f %10.2f %10.2f %12.2e\n', mC(q,1), mC(q,2), mC(q,3), mC(q,4), mC(q,5));
end
fprintf('Control input: %.3f .. %.3f V offset-free  (limits %.2f .. %.2f)\n', ...
        min(u_sim), max(u_sim), umin, umax);
fprintf('             = %.3f .. %.3f V absolute     (Vmin = %.2f, Vmax = %.2f)\n', ...
        min(u_sim) + u_offset, max(u_sim) + u_offset, Vmin, Vmax);

% ---- link to the pre-lab: closed-loop transfer functions (pre-lab c)
C_pi = pid(Kp, Ki, 0, 0, h, 'IFormula', 'ForwardEuler');   % = Kp + Ki*h/(z-1)
Gcl  = feedback(C_pi*G_tf, 1);       % Y(z)/Yref(z)
Gu   = feedback(C_pi, G_tf);         % U(z)/Yref(z)
fprintf('\nClosed loop Y/Yref (pre-lab c):\n');
display(tf(Gcl));
fprintf('Closed-loop poles: %s\n', mat2str(pole(Gcl).', 4));
fprintf('Gcl(1) = %.6f (= 1 -> zero steady-state error)\n', dcgain(Gcl));
[GMg, PMg] = margin(C_pi*G_tf);
fprintf('Gain margin = %.2f (%.1f dB), phase margin = %.1f deg\n', GMg, 20*log10(GMg), PMg);
if all(u_sim <= umax & u_sim >= umin)
    y_lin = lsim(Gcl, r, t_sim);   u_lin = lsim(Gu, r, t_sim);
    fprintf('Saturation never active, so the loop simulation must equal lsim(Gcl, r):\n');
    fprintf('   max |y_loop - y_Gcl| = %.2e,  max |u_loop - u_Gu| = %.2e\n', ...
            max(abs(y_sim - y_lin)), max(abs(u_sim - u_lin)));
end

% ---- Part 2 figures
% (1) design map
figMap = figure('Name','PI gain map','Color','w');
TSplot = TSmap;  TSplot(~feasible) = NaN;
hI = imagesc(Ki_grid, Kp_grid, TSplot);  set(hI, 'AlphaData', ~isnan(TSplot));
set(gca, 'YDir', 'normal', 'Color', [0.92 0.92 0.92]); hold on;
cb = colorbar; cb.Label.String = 'Sum of 2% settling times (s)';
[~, hC] = contour(Ki_grid, Kp_grid, OSmap, [OS_MAX OS_MAX], 'r', 'LineWidth', 1.2);
hF = plot(Ki_fast, Kp_fast, 'kd', 'MarkerFaceColor', 'y', 'MarkerSize', 8);
hD = plot(Ki, Kp, 'kp', 'MarkerFaceColor', 'g', 'MarkerSize', 12);
hS = plot(0.01, 0.5, 'ks', 'MarkerFaceColor', 'w');
hold off;
xlabel('K_i'); ylabel('K_p');
title('Feasible PI gains (grey = fails spec, red line = 2% overshoot)');
legend([hC hF hD hS], 'OS = 2%', 'Fastest feasible', 'Chosen', 'Notes start', 'Location', 'northwest');

% (2) candidate comparison
figCmp = figure('Name','Gain comparison','Color','w');
stairs(t_sim, r, 'k', 'LineWidth', 1); hold on;
cols = lines(size(cand,1));
for c = 1:size(cand,1)
    plot(t_sim, candY{c}, 'Color', cols(c,:), 'LineWidth', 1 + (c == size(cand,1)));
end
hold off; grid on;
xlabel('Time (sec)'); ylabel('Offset-free Water Level (V)');
title('Offset-Free Closed-Loop Response for Different PI Gains');
legend([{'Reference'}; cand(:,1)], 'Location', 'northeast');

% (3) chosen design, offset-free (what the Simulink design model shows)
figDes = figure('Name','PI design (offset-free)','Color','w');
subplot(2,1,1);
stairs(t_sim, r, 'g', 'LineWidth', 1); hold on; plot(t_sim, y_sim, 'b'); hold off;
title(sprintf('Offset-Free PI Control (K_p = %.2f, K_i = %.3f)', Kp, Ki));
xlabel('Time (sec)'); ylabel('Water Level (V)');
legend('Reference Output', 'Simulated Output'); grid on;
subplot(2,1,2);
stairs(t_sim, us_sim, 'b'); hold on;
plot(t_sim([1 end]), [umax umax], 'k-.', t_sim([1 end]), [umin umin], 'k-.'); hold off;
title('Offset-Free Control Input'); xlabel('Time (sec)'); ylabel('Pump Voltage (V)');
legend('Simulated Control Input', 'u_{max} / u_{min}'); grid on;
ylim([umin umax] + [-0.2 0.2]);

% (4) chosen design in absolute volts, laid out like Fig. 4 of the notes
figAbs = figure('Name','PI design (absolute, Fig. 4 style)','Color','w');
subplot(2,1,1);
stairs(t_sim, r + y_offset, 'g'); hold on; plot(t_sim, y_sim + y_offset, 'b'); hold off;
title({'PI Control Results (Simulation)', 'Output Signal'});
xlabel({'Time (sec)', '(a)'}); ylabel('Water Level (V)');
legend('Reference Output', 'Simulated Output'); grid on; ylim([0 5]);
subplot(2,1,2);
stairs(t_sim, us_sim + u_offset, 'b'); hold on;
plot(t_sim([1 end]), [Vmax Vmax], 'k-.', t_sim([1 end]), [Vmin Vmin], 'k-.'); hold off;
text(t_sim(round(end*0.25)), Vmax - 0.12, sprintf('V_{max} = %.1f', Vmax));
text(t_sim(round(end*0.25)), Vmin + 0.12, sprintf('V_{min} = %.1f', Vmin));
title('Control Input Signal'); xlabel({'Time (sec)', '(b)'}); ylabel('Pump Voltage (V)');
legend('Simulated Control Input'); grid on; ylim([Vmin - 0.3, Vmax + 0.3]);

fprintf(['\n--- Choice of gains ---\n' ...
 'Increasing Kp speeds up the response, but on the -0.7 V step (0.2 -> -0.5) the\n' ...
 'controller output jumps by about -0.7*Kp, so Kp is capped by the input limits\n' ...
 '(roughly Kp <= 1.4 here). Ki removes the steady-state error and sets how hard the\n' ...
 'integrator pushes through each level; too much Ki gives overshoot. The grid search\n' ...
 'shows the fastest pair that meets the spec (Kp = %.2f, Ki = %.3f) sits right on the\n' ...
 'input limit and close to 2%% overshoot. The chosen pair (Kp = %.2f, Ki = %.3f) is\n' ...
 'only slightly slower but leaves margin: overshoot %.2f%% (spec < 2%%) and peak |u| =\n' ...
 '%.2f V of the %.2f V available. That margin matters because the real tank is\n' ...
 'nonlinear and noisy, so it will not match the identified model exactly.\n'], ...
 Kp_fast, Ki_fast, Kp, Ki, max(mC(:,2)), max(abs(u_sim)), min(abs([umin umax])));

%% ---- Bonus (part 3, optional): initial transient of the real-time run
y_empty = 0;                                  % sensor reads ~0 V with an empty tank
y0 = y_empty - y_offset;                      % offset-free starting level
u0 = y0*(1 + a1 + a2)/(b1 + b2);              % input that would hold that level
[y_b, u_b, us_b] = simPI(pm, Kp, Ki, h, r, umin, umax, y0, u0);
nSatB = find(us_b < umax - 1e-9, 1) - 1;
fprintf(['\n--- Bonus: initial transient of the real-time run ---\n' ...
 'The simulation assumes zero initial conditions, i.e. the tank already sits at\n' ...
 'y_offset. On the rig the tank starts (almost) empty, so in the first period the\n' ...
 'controller sees e = 0 - (%.1f - %.2f) = %.2f V. Kp*e alone asks for %.2f V\n' ...
 'absolute, far above Vmax = %.1f V, so the pump runs flat out at Vmax\n' ...
 '(saturated) while the tank fills. During that time the integrator keeps\n' ...
 'accumulating the large error (integrator wind-up: the Discrete PID block has no\n' ...
 'anti-windup), so once the level reaches the set-point the integral term is too\n' ...
 'big and the level overshoots before the integrator unwinds. The tank is also far\n' ...
 'from the operating point the linear model was identified at (outflow ~ sqrt of\n' ...
 'level), so this part of the run is not expected to match the simulation. Fixes:\n' ...
 'pre-fill the tank to y_offset before starting, turn on anti-windup (clamping) in\n' ...
 'the PID block, or set the integrator initial condition.\n' ...
 'Linear-model estimate of this case: input saturated for the first %.1f s.\n'], ...
 y_empty, y_offset, -y0, Kp*(-y0) + u_offset, Vmax, nSatB*h);

figBon = figure('Name','Bonus: empty-tank start','Color','w');
subplot(2,1,1);
stairs(t_sim, r + y_offset, 'g'); hold on;
plot(t_sim, y_sim + y_offset, 'b'); plot(t_sim, y_b + y_offset, 'm'); hold off;
title('Start from Steady State (Simulation) vs Start from an Empty Tank (Linear Model)');
xlabel('Time (sec)'); ylabel('Water Level (V)'); grid on; ylim([0 5]);
legend('Reference Output', 'Zero initial conditions', 'Empty tank at t = 0');
subplot(2,1,2);
stairs(t_sim, us_sim + u_offset, 'b'); hold on; stairs(t_sim, us_b + u_offset, 'm');
plot(t_sim([1 end]), [Vmax Vmax], 'k-.', t_sim([1 end]), [Vmin Vmin], 'k-.'); hold off;
xlabel('Time (sec)'); ylabel('Pump Voltage (V)'); grid on; ylim([Vmin - 0.3, Vmax + 0.3]);
legend('Zero initial conditions', 'Empty tank at t = 0');

%% ---- save for the Simulink / real-time scripts
save('Lab5_Design.mat', 'Kp', 'Ki', 'u_offset', 'y_offset', 'Vmax', 'Vmin', 'umin', 'umax', ...
     'a1', 'a2', 'b1', 'b2', 'Gm', 'Hm', 'Cm', 'Dm', 'Ts', 'G_tf', 'G_ss', 'Gcl', 'Gu', ...
     'REF_LEVELS', 'N_LEVEL', 'N_PERIODS', 'lev', 'r', 't_sim', 'y_sim', 'u_sim', 'us_sim');
fprintf('\nSaved design to %s\n', fullfile(pwd, 'Lab5_Design.mat'));

if SAVE_FIGS
    if ~exist(FIG_DIR, 'dir'), mkdir(FIG_DIR); end
    figs  = [figRaw figOff figVer figMap figCmp figDes figAbs figBon];
    names = {'P1_RawData', 'P1_OffsetFreeData', 'P1_ModelVerification', 'P2_GainMap', ...
             'P2_GainComparison', 'P2_PIDesign_OffsetFree', 'P2_PIDesign_Fig4Style', ...
             'Bonus_EmptyTankStart'};
    for i = 1:numel(figs)
        exportgraphics(figs(i), fullfile(FIG_DIR, [names{i} '.png']), 'Resolution', 150);
    end
    fprintf('Figures saved to %s\n', FIG_DIR);
end

%% ======================= local functions =================================
function [y, u, us] = simPI(p, Kp, Ki, h, r, umin, umax, y0, u0)
% Discrete PI loop, identical to the Simulink Discrete PID block
% (parallel, Forward Euler):  u(k) = Kp*e(k) + I(k),  I(k+1) = I(k) + Ki*h*e(k)
% followed by saturation and the plant
%   y(k) = -a1*y(k-1) - a2*y(k-2) + b1*us(k-1) + b2*us(k-2).
% y0/u0 = constant past output/input (default 0 = zero initial conditions).
if nargin < 8, y0 = 0; u0 = 0; end
N  = numel(r);
Y  = [y0; y0; zeros(N,1)];
US = [u0; u0; zeros(N,1)];
u  = zeros(N,1);
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

function m = stepMetrics(r, y, nLevel, h)
% One row per reference change: [step size, overshoot %, 10-90% rise time,
% 2% settling time (NaN if it never settles in the period), final error]
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
