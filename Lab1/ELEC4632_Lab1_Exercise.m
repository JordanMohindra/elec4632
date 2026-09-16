%% ELEC4632 Lab 1 - Lab Exercise (2 marks): System Identification
% Second-order discrete-time model via linear least squares

clear; clc; close all;

%% 1a. Load pre-collected data
load('SysIdenData_StudentVersion.mat');   % loads LogData
t      = LogData.time;
y_act  = LogData.signals(1).values(:,2);   % actual noise-reduced output
y_actm = LogData.signals(1).values(:,1);   % original measured output
u_act  = LogData.signals(2).values;        % actual input
Ts     = mean(diff(t));

fprintf('Sampling time Ts = %.4f sec\n', Ts);

%% 1b. Plot original data (Fig. 5)
figure;

subplot(2,1,1);
plot(t, y_actm, 'r'); hold on;
plot(t, y_act, 'b'); hold off;
title('Actual Output Signal');
xlabel('Time (sec)'); ylabel('Water Level (V)');
legend('Measured Output', 'Noise-Reduced Output');
grid on;

subplot(2,1,2);
plot(t, u_act, 'Color', [0.3 0.6 0.9]);
title('Actual Input Signal');
xlabel('Time (sec)'); ylabel('Pump Voltage (V)');
legend('Actual Input');
grid on;

%% 1c. Offset removal
u_offset = u_act(1);   % first value assumed to be input offset (per hint)
u = u_act - u_offset;

idx = 1;
while idx < length(u_act) && u_act(idx+1) == u_act(1)
    idx = idx + 1;
end
y_offset = mean(y_act(1:idx));   % average over first period, before first input change
y = y_act - y_offset;

fprintf('u_offset = %.4f V\n', u_offset);
fprintf('y_offset = %.4f V (averaged over first %d samples)\n', y_offset, idx);

figure;
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
kstart = 10;   % start a few samples in - avoids needing y(-1),y(0),u(-1),u(0)
k      = (kstart:Nhalf)';

Y   = y(k);
Phi = [y(k-1) y(k-2) u(k-1) u(k-2)];

theta_hat = inv(Phi'*Phi) * (Phi'*Y);   % least squares solution, Eq. (8)
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

%% 3. Model verification and simulation
u_2nd = u(Nhalf+1:end);
y_2nd = y(Nhalf+1:end);
t_2nd = (0:numel(u_2nd)-1)' * Ts;

y_sim_2nd = lsim(sys_tf, u_2nd, t_2nd);

figure;
plot(t_2nd, y_2nd, 'r'); hold on;
plot(t_2nd, y_sim_2nd, 'b--'); hold off;
title('Offset-Free Model Verification (2nd Half)');
xlabel('Time (sec)'); ylabel('Water Level (V)');
legend('Actual Output', 'Simulated Output'); grid on;

t_full = (0:numel(u)-1)' * Ts;
y_sim_full = lsim(sys_tf, u, t_full);

figure;
plot(t_full, y, 'r'); hold on;
plot(t_full, y_sim_full, 'b--'); hold off;
title('Offset-Free Model Verification (Entire)');
xlabel('Time (sec)'); ylabel('Water Level (V)');
legend('Actual Output', 'Simulated Output'); grid on;

fprintf('\n--- Answer to 3c: why the simulated output does not start at the same point as y ---\n');
fprintf('lsim initialises the model states at zero (x(0) = 0), i.e. it assumes the\n');
fprintf('tank was exactly at its offset-free equilibrium at the start of that data\n');
fprintf('window. The state of the real tank at that instant carries the history of\n');
fprintf('everything that happened before it (the first half of the run), which the\n');
fprintf('model has no knowledge of. So there is an initial transient while the model\n');
fprintf('state catches up to the real, unknown initial condition; after that the two\n');
fprintf('curves converge because the identified dynamics are close to the real ones.\n');
