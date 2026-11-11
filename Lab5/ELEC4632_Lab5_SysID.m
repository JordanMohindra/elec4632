%% ELEC4632 Lab 5 - Part 1: system identification (same procedure as Lab 2)
% Identifies the second-order model, works out the offsets and pump limits,
% plots the Lab 1/2 figures, and saves everything the Simulink model needs
% to Lab5_Model.mat (a1, a2, b1, b2, Ts, u_offset, y_offset, umin, umax).
% Run this before opening Lab5_PI_Design.slx.

clear; clc; close all;

%% ===================== USER SETTINGS =====================================
DATA_FILE = 'SysIdenData_StudentVersion.mat';   % <-- your SysIdenData_x.mat

% Pump voltage limits of YOUR rig (Saturation TANK#1 block).
% Lab 2 Eq. (1): u_offset = (Vmax + Vmin)/2 with Vmax = 3.0 V fixed by the
% lab, so Vmin = 2*u_offset - Vmax. Type in the measured Vmin if you have it.
Vmax = 3.0;            % [V]
Vmin = [];             % [V]  [] = derive from u_offset

SAVE_FIGS = true;
FIG_DIR   = fullfile(pwd, 'figures');
%% =========================================================================

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

%% offsets and pump limits
u_offset = u_raw(1);                       % first period = u_offset only
if isempty(Vmin), Vmin = 2*u_offset - Vmax; end
umin = Vmin - u_offset;
umax = Vmax - u_offset;

% constant-input segments and their settled output levels
chg    = find(abs(diff(u_raw)) > 1e-6);
sStart = [1; chg+1];
sEnd   = [chg; N_raw];
nSeg   = numel(sStart);
offSeg = find(abs(u_raw(sStart) - u_offset) < 1e-6);
sSettle = sStart;
sLevel  = nan(nSeg,1);
for s = 1:nSeg
    idx   = (sStart(s):sEnd(s))';
    final = mean(y_nr(idx(max(1, round(0.75*numel(idx))):end)));
    out   = find(abs(y_nr(idx) - final) > 0.05, 1, 'last');
    if ~isempty(out), sSettle(s) = min(idx(end), idx(out) + 1); end
    sLevel(s) = mean(y_nr(sSettle(s):sEnd(s)));
end

% Lab 2 step 5a: decide what to cut off
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
u_act  = u_raw(keep);

% output offset = mean of the settled output over every u_offset segment
offIdx = [];
for s = offSeg'
    a = max(sSettle(s), kCut);  b = sEnd(s);
    if b >= a, offIdx = [offIdx; (a:b)' - kCut + 1]; end %#ok<AGROW>
end
y_offset = mean(y_act(offIdx));

u = u_act - u_offset;
y = y_act - y_offset;

fprintf('u_offset = %.4f V,  y_offset = %.4f V\n', u_offset, y_offset);
fprintf('Pump limits Vmin = %.2f V, Vmax = %.2f V  ->  %.2f V <= u(k) <= %.2f V offset-free\n', ...
        Vmin, Vmax, umin, umax);

%% least squares, second-order model (Lab 1 Eq. 4-8)
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
display(G_tf);
display(G_ss);
fprintf('Poles: %s    DC gain: %.4f V/V\n', mat2str(pole(G_tf).', 5), dcgain(G_tf));

%% verification
u_2nd = u(Nhalf+1:end);   y_2nd = y(Nhalf+1:end);
t_2nd = (0:numel(u_2nd)-1)' * Ts;
y_sim_2nd  = lsim(G_tf, u_2nd, t_2nd);
t_full     = (0:N-1)' * Ts;
y_sim_full = lsim(G_tf, u, t_full);
fprintf('MSE (entire) = %.6f V^2\n', mean((y - y_sim_full).^2));

%% figures (the ones asked for in Labs 1 and 2)
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

%% save for Simulink
save('Lab5_Model.mat', 'a1', 'a2', 'b1', 'b2', 'Ts', 'u_offset', 'y_offset', ...
     'Vmax', 'Vmin', 'umin', 'umax', 'Gm', 'Hm', 'Cm', 'Dm');
fprintf('\nSaved %s - the variables are also in the workspace for Simulink.\n', ...
        fullfile(pwd, 'Lab5_Model.mat'));

if SAVE_FIGS
    if ~exist(FIG_DIR, 'dir'), mkdir(FIG_DIR); end
    exportgraphics(figRaw, fullfile(FIG_DIR, 'P1_RawData.png'), 'Resolution', 150);
    exportgraphics(figOff, fullfile(FIG_DIR, 'P1_OffsetFreeData.png'), 'Resolution', 150);
    exportgraphics(figVer, fullfile(FIG_DIR, 'P1_ModelVerification.png'), 'Resolution', 150);
    fprintf('Figures saved to %s\n', FIG_DIR);
end
