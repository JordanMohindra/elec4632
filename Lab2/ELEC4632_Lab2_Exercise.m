%% ELEC4632 Lab 2 - Identifying a linear model for a water-tank system
% Lab Exercise Steps 1-5.
%   Steps 1-4 are done on the rig in the lab; this script does the
%   calculations for them (Eq. 1, Eq. 2, Repeating Sequence Stair period).
%   Step 5 is the analysis: cut off the unusable part of the recorded data,
%   then identify and verify a second-order model the same way as Lab 1.
%
% >>> TO USE YOUR OWN LAB 2 DATA: change DATA_FILE below to your
% >>> SysIdenData_#.mat file and fill in Vmin / t_settle from the lab.

clear; clc; close all;

%% ===================== USER SETTINGS =====================================
% Data file. For now this points at the Lab 1 data so the script can be
% tested. Replace with e.g. fullfile(pwd,'SysIdenData_0.mat') later.
DATA_FILE = fullfile('..', 'Lab1', 'SysIdenData_StudentVersion.mat');

% Step 2 results (fill in from the table in the lab)
Vmax      = 3.0;     % [V]  fixed by the lab
Vmin      = 2.0;     % [V]  <-- REPLACE with the Vmin you found in Step 2b
Level_min = NaN;     % [ml] <-- water level at Vmin (80-110 ml)
Level_max = NaN;     % [ml] <-- water level at Vmax (240-280 ml)

% Step 3d: time the level took to settle after the u_offset -> 3 V step
t_settle  = [];      % [sec] <-- from Scope cursor measurements ([] = unknown)

% Step 5a: how to cut the raw data
%   'auto'           - decide automatically (recommended)
%   'transient'      - cut only the initial transient
%   'first_sequence' - cut the first sequence + transient of the 2nd one
%   'none'           - keep everything
CUT_MODE   = 'auto';
settle_tol = 0.05;   % [V] band used to decide a level has settled
offset_tol = 0.05;   % [V] max allowed difference between output offsets

% Draw the Fig. 5 style annotations (Transient / Output Offset / One Sequence)
ANNOTATE_RAW = true;

% Axis limits used in the lab notes figures (Fig. 5 and Fig. 6)
XLIM_RAW = [0 1400];   XLIM_CUT = [0 700];
YLIM_OUT = [0 5];      YLIM_IN  = [1.5 3.5];

SAVE_FIGS = true;
FIG_DIR   = fullfile(pwd, 'figures');
%% =========================================================================

Ts_nom = 0.75;   % model sampling time [sec]

%% ---------------- Steps 1-4: calculations for the rig -------------------
fprintf('=========== Steps 2-4: settings for the Simulink model ===========\n');
fprintf('Step 2 table:\n');
fprintf('  Vmin = %.2f V    Water Level min = %g ml\n', Vmin, Level_min);
fprintf('  Vmax = %.2f V    Water Level max = %g ml\n', Vmax, Level_max);

u_offset_eq1 = (Vmax + Vmin)/2;                     % Eq. (1)
umin = Vmin - u_offset_eq1;                         % Eq. (2)
umax = Vmax - u_offset_eq1;
fprintf('\nStep 3a: u_offset = (Vmax + Vmin)/2 = (%.2f + %.2f)/2 = %.3f V\n', ...
        Vmax, Vmin, u_offset_eq1);
fprintf('Step 3b: control input range  %.3f V <= u(k) <= %.3f V\n', umin, umax);

stair_default = [-0.3 0.2 0];                       % default block values
if all(stair_default >= umin - 1e-9 & stair_default <= umax + 1e-9)
    fprintf('         Default stair values [%s] are inside the range -> keep them.\n', ...
            num2str(stair_default));
else
    fprintf('         Default stair values [%s] are OUTSIDE the range -> change them\n', ...
            num2str(stair_default));
    fprintf('         (e.g. [%.2f %.2f 0]).\n', 0.9*umin, 0.9*umax);
end

N_default = 230;
if isempty(t_settle)
    N_period = N_default;
    fprintf('Step 3d: t_settle not entered -> using default period %d x 0.75 = %.1f s\n', ...
            N_period, N_period*Ts_nom);
else
    N_meas   = ceil(t_settle/Ts_nom);
    N_period = min(max(N_meas, N_default), 250);    % >230 use it, cap at 250
    fprintf('Step 3d: t_settle = %.1f s -> %.1f/0.75 = %d samples\n', t_settle, t_settle, N_meas);
    fprintf('         Use Sample Time = %d*0.75 = %.1f s\n', N_period, N_period*Ts_nom);
end

fprintf(['\nStep 4a (Why is the first period equal to u_offset?):\n' ...
 '  The Repeating Sequence Stair runs at the slow rate (230*0.75 s) and\n' ...
 '  feeds a Rate Transition block (slow -> fast, 0.75 s). With deterministic\n' ...
 '  data transfer, a slow-to-fast Rate Transition delays the signal by one\n' ...
 '  slow period and outputs its initial condition (0) during that first\n' ...
 '  period. So the offset-free input is 0 and only u_offset reaches the pump,\n' ...
 '  which gives the tank time to fill up to the output offset level.\n\n']);

%% ---------------- Step 5: load raw data -----------------------------------
S = load(DATA_FILE);
fn = fieldnames(S);
LogData = S.(fn{1});                               % LogData structure

t_raw  = LogData.time(:);
y_meas = LogData.signals(1).values(:,1);           % measured output
y_nr   = LogData.signals(1).values(:,2);           % noise-reduced output
u_raw  = LogData.signals(2).values(:);             % actual input
Ts     = mean(diff(t_raw));
N_raw  = numel(t_raw);
fprintf('=========== Step 5: system identification ===========\n');
fprintf('Data file: %s\n', DATA_FILE);
fprintf('Samples = %d, duration = %.1f s, Ts = %.4f s\n', N_raw, t_raw(end), Ts);

%% ---------------- Step 5a: find segments, offsets and the cut point ------
u_offset = u_raw(1);             % first period is the offset-only input

chg    = find(abs(diff(u_raw)) > 1e-6);
sStart = [1; chg+1];
sEnd   = [chg; N_raw];
sVal   = u_raw(sStart);
isOff  = abs(sVal - u_offset) < 1e-6;
offSeg = find(isOff);

% settled part of each segment: after the last sample outside the band
% around the segment's final level (mean of the last 25%)
nSeg = numel(sStart);
sSettle = sStart; sLevel = nan(nSeg,1);
for s = 1:nSeg
    idx   = (sStart(s):sEnd(s))';
    tail  = idx(max(1, round(0.75*numel(idx))):end);
    final = mean(y_nr(tail));
    out   = find(abs(y_nr(idx) - final) > settle_tol, 1, 'last');
    if isempty(out), sSettle(s) = sStart(s);
    else,            sSettle(s) = min(idx(end), idx(out) + 1);
    end
    sLevel(s) = mean(y_nr(sSettle(s):sEnd(s)));
end

period_samples = median(sEnd(2:end-1) - sStart(2:end-1) + 1);
if isnan(period_samples), period_samples = N_period; end
seq_samples = 3*period_samples;                  % 3 stair levels per sequence

first  = offSeg(1);
hasTransient = abs(y_nr(1) - sLevel(first)) > settle_tol;
off1   = sLevel(first);
offLater = sLevel(offSeg(2:end));
offLater = offLater(sEnd(offSeg(2:end)) - sSettle(offSeg(2:end)) > 10);   % enough settled data

mode = CUT_MODE;
if strcmp(mode, 'auto')
    if ~isempty(offLater) && abs(off1 - mean(offLater)) > offset_tol
        mode = 'first_sequence';
    elseif hasTransient
        mode = 'transient';
    else
        mode = 'none';
    end
end
% only drop the first sequence if at least one full sequence remains
if strcmp(mode, 'first_sequence')
    if numel(offSeg) < 2 || (N_raw - sSettle(offSeg(2)) + 1) < seq_samples
        warning(['Not enough data after the first sequence - cutting only the ' ...
                 'initial transient instead.']);
        mode = 'transient';
    end
end

switch mode
    case 'none',            kCut = 1;
    case 'transient',       kCut = sSettle(first);
    case 'first_sequence',  kCut = sSettle(offSeg(2));
    otherwise, error('Unknown CUT_MODE "%s"', CUT_MODE);
end
fprintf('\nStep 5a: 1st-sequence output offset = %.4f V', off1);
if ~isempty(offLater), fprintf(', later offsets = %s V', mat2str(offLater', 4)); end
fprintf('\n         Cut mode = %s -> keep samples %d..%d (t >= %.2f s)\n', ...
        mode, kCut, N_raw, t_raw(kCut));

% transient end (for the Fig. 5 annotation)
tTransEnd = t_raw(sSettle(first));

%% ---------------- Fig. 5: raw collected data -----------------------------
fig1 = figure('Name','Raw collected data','Color','w','Position',[100 80 560 600]);
ax1 = subplot(2,1,1);
plot(t_raw, y_nr, 'b'); hold on;
plot(t_raw, y_meas, 'r');
title('Actual Output Signal', 'FontWeight', 'bold');
xlabel({'Time (sec)', '(a)'}); ylabel('Water Level (V)');
legend('Noise-Reduced Output', 'Measured Output', 'Location', 'northeast');
xlim(XLIM_RAW); ylim(YLIM_OUT); grid on; box on;

ax2 = subplot(2,1,2);
hIn = plot(t_raw, u_raw, 'b', 'HandleVisibility', 'off'); hold on;
hLeg = plot(NaN, NaN, 'b');   % legend proxy (datatips recolour the legend icon)
title('Actual Input Signal', 'FontWeight', 'bold');
xlabel({'Time (sec)', '(b)'}); ylabel('Pump Voltage (V)');
legend(hLeg, 'Actual Input', 'Location', 'northeast');
xlim(XLIM_RAW); ylim(YLIM_IN); grid on; box on;

% data tips at the end of each u_offset segment (as in Fig. 5b)
for s = offSeg'
    if s < nSeg                       % only where the input then changes
        datatip(hIn, t_raw(sEnd(s)), u_raw(sEnd(s)), 'Location', 'northeast');
    end
end

if ANNOTATE_RAW
    y_off_all = mean(sLevel(offSeg));
    green = [0 0.65 0.3];  tBlue = [0.1 0.35 0.7];
    axes(ax1); hold on;
    plot(XLIM_RAW, [y_off_all y_off_all], '--', 'Color', green, 'LineWidth', 1.2, ...
         'HandleVisibility', 'off');
    drawnow;
    % Output Offset label + arrows to each offset plateau
    lblX = mean(XLIM_RAW)*0.8;  lblY = 0.12*diff(YLIM_OUT) + YLIM_OUT(1);
    text(lblX, lblY + 0.18, 'Output Offset', 'Color', green, 'EdgeColor', green, ...
         'BackgroundColor', 'w', 'HorizontalAlignment', 'center', 'FontSize', 8);
    for s = offSeg'
        xm = t_raw(round((sSettle(s) + sEnd(s))/2));
        dataArrow(ax1, [lblX xm], [lblY + 0.4, sLevel(s) - 0.05], green, '--');
    end
    % Transient
    if hasTransient
        dataArrow(ax1, [t_raw(1) tTransEnd], [0.5 0.5], tBlue, ':', 'doublearrow');
        plot([tTransEnd tTransEnd], [sLevel(first) YLIM_OUT(1)-0.6], '--', 'Color', tBlue, ...
             'Clipping', 'off', 'HandleVisibility', 'off');
        text(tTransEnd, YLIM_OUT(1)-0.6, 'Transient', 'Color', tBlue, 'EdgeColor', tBlue, ...
             'BackgroundColor', 'w', 'HorizontalAlignment', 'right', 'FontSize', 8, ...
             'Clipping', 'off');
    end
    % One Sequence (input)
    axes(ax2); hold on;
    tSeqEnd = t_raw(min(N_raw, sStart(first) + seq_samples - 1));
    if numel(offSeg) >= 2, tSeqEnd = t_raw(sStart(offSeg(2))); end
    yArr = YLIM_IN(1) + 0.2*diff(YLIM_IN);
    dataArrow(ax2, [t_raw(1) tSeqEnd], [yArr yArr], 'r', ':', 'doublearrow');
    plot([tSeqEnd tSeqEnd], [yArr u_offset], '-.', 'Color', 'r', 'HandleVisibility', 'off');
    text(mean([t_raw(1) tSeqEnd]), yArr - 0.12, 'One Sequence', 'Color', 'r', ...
         'EdgeColor', 'r', 'BackgroundColor', 'w', 'HorizontalAlignment', 'center', 'FontSize', 8);
end

% re-draw the input legend last (annotations otherwise turn its icon black)
legend(ax2, hLeg, 'Actual Input', 'Location', 'northeast');

%% ---------------- Cut the data -------------------------------------------
keep   = (kCut:N_raw)';
t      = t_raw(keep) - t_raw(kCut);        % time restarts at zero
y_act  = y_nr(keep);
y_actm = y_meas(keep);
u_act  = u_raw(keep);

%% ---------------- Fig. 6: cut-off data -----------------------------------
fig2 = figure('Name','Cut-off data','Color','w','Position',[680 80 560 600]);
subplot(2,1,1);
plot(t, y_act, 'b'); hold on;
plot(t, y_actm, 'r');
title('Actual Output Signal', 'FontWeight', 'bold');
xlabel({'Time (sec)', '(a)'}); ylabel('Water Level (V)');
legend('Noise-Reduced Output', 'Measured Output', 'Location', 'northeast');
xlim(XLIM_CUT); ylim(YLIM_OUT); grid on; box on;

subplot(2,1,2);
plot(t, u_act, 'b');
title('Actual Input Signal', 'FontWeight', 'bold');
xlabel({'Time (sec)', '(b)'}); ylabel('Pump Voltage (V)');
legend('Actual Input', 'Location', 'northeast');
xlim(XLIM_CUT); ylim(YLIM_IN); grid on; box on;

%% ================= Lab 1 procedure on the cut-off data ===================
%% 1c. Offset removal
u = u_act - u_offset;

% output offset = mean of the settled output over every u_offset segment
% that is inside the kept data (green levels in Fig. 5a)
offIdx = [];
for s = offSeg'
    a = max(sSettle(s), kCut);  b = sEnd(s);
    if b >= a, offIdx = [offIdx; (a:b)' - kCut + 1]; end %#ok<AGROW>
end
y_offset = mean(y_act(offIdx));
y = y_act - y_offset;

fprintf('\nu_offset = %.4f V\n', u_offset);
fprintf('y_offset = %.4f V (mean over %d settled samples at u = u_offset)\n', ...
        y_offset, numel(offIdx));

fig3 = figure('Name','Offset-free data','Color','w');
subplot(2,1,1);
plot(t, y, 'r');
title('Actual Offset-Free Output Signal');
xlabel('Time (sec)'); ylabel('Water Level (V)');
legend('Actual Output'); grid on;

subplot(2,1,2);
plot(t, u, 'Color', [0.3 0.6 0.9]);
title('Actual Offset-Free Input Signal');
xlabel('Time (sec)'); ylabel('Pump Voltage (V)');
legend('Actual Input'); grid on;

%% 2. Identify a second-order discrete-time model (least squares)
N      = length(u);
Nhalf  = floor(N/2);
kstart = 10;
k      = (kstart:Nhalf)';

Y   = y(k);
Phi = [y(k-1) y(k-2) u(k-1) u(k-2)];

theta_hat = (Phi'*Phi) \ (Phi'*Y);         % least squares solution
a1 = -theta_hat(1);
a2 = -theta_hat(2);
b1 =  theta_hat(3);
b2 =  theta_hat(4);

fprintf('\nIdentified second-order model parameters:\n');
fprintf(' a1 = %.6f\n a2 = %.6f\n b1 = %.6f\n b2 = %.6f\n\n', a1, a2, b1, b2);

sys_tf = tf([b1 b2], [1 a1 a2], Ts)
Gm = [0 1; -a2 -a1];
Hm = [0; 1];
Cm = [b2 b1];
Dm = 0;
sys_ss = ss(Gm, Hm, Cm, Dm, Ts)
fprintf('Poles: %s\n', mat2str(pole(sys_tf).', 5));
fprintf('DC gain: %.4f V/V\n', dcgain(sys_tf));

%% 3. Model verification and simulation
u_2nd = u(Nhalf+1:end);
y_2nd = y(Nhalf+1:end);
t_2nd = (0:numel(u_2nd)-1)' * Ts;
y_sim_2nd = lsim(sys_tf, u_2nd, t_2nd);

fig4 = figure('Name','Verification (2nd half)','Color','w');
plot(t_2nd, y_2nd, 'r'); hold on;
plot(t_2nd, y_sim_2nd, 'b--'); hold off;
title('Offset-Free Model Verification (2nd Half)');
xlabel('Time (sec)'); ylabel('Water Level (V)');
legend('Actual Output', 'Simulated Output'); grid on;

t_full = (0:numel(u)-1)' * Ts;
y_sim_full = lsim(sys_tf, u, t_full);

fig5 = figure('Name','Verification (entire)','Color','w');
plot(t_full, y, 'r'); hold on;
plot(t_full, y_sim_full, 'b--'); hold off;
title('Offset-Free Model Verification (Entire)');
xlabel('Time (sec)'); ylabel('Water Level (V)');
legend('Actual Output', 'Simulated Output'); grid on;

MSE_2nd  = mean((y_2nd - y_sim_2nd).^2);
MSE_full = mean((y - y_sim_full).^2);
fprintf('MSE (2nd half) = %.6f V^2,  MSE (entire) = %.6f V^2\n', MSE_2nd, MSE_full);

%% Save figures
if SAVE_FIGS
    if ~exist(FIG_DIR, 'dir'), mkdir(FIG_DIR); end
    figs  = [fig1 fig2 fig3 fig4 fig5];
    names = {'Fig5_RawData','Fig6_CutOffData','OffsetFreeData', ...
             'Verification_2ndHalf','Verification_Entire'};
    for i = 1:numel(figs)
        exportgraphics(figs(i), fullfile(FIG_DIR, [names{i} '.png']), 'Resolution', 150);
    end
    fprintf('Figures saved to %s\n', FIG_DIR);
end

%% ---------------- local function -----------------------------------------
function dataArrow(ax, x, y, col, ls, type)
% Draw an annotation arrow using data coordinates of axes ax.
if nargin < 6, type = 'arrow'; end
drawnow;
p  = ax.Position;  xl = ax.XLim;  yl = ax.YLim;
xf = p(1) + (x - xl(1))/diff(xl)*p(3);
yf = p(2) + (y - yl(1))/diff(yl)*p(4);
xf = min(max(xf, 0), 1);  yf = min(max(yf, 0), 1);
h = annotation(ax.Parent, type, xf, yf, 'Color', col, 'LineStyle', ls);
if strcmp(type, 'doublearrow')
    h.Head1Length = 5; h.Head2Length = 5; h.Head1Width = 5; h.Head2Width = 5;
else
    h.HeadLength = 6; h.HeadWidth = 6;
end
end
