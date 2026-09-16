%% ELEC4632 Lab 1 - Post-lab Exercise: Model Order Comparison
% Identify a first-order model and compare it against the second-order
% model from the Lab Exercise, using MSE as the numerical criterion.

clear; clc; close all;

%% Reload data and repeat offset removal (same procedure as Exercise 1)
load('SysIdenData_StudentVersion.mat');
t      = LogData.time;
y_act  = LogData.signals(1).values(:,2);
u_act  = LogData.signals(2).values;
Ts     = mean(diff(t));

u_offset = u_act(1);
u = u_act - u_offset;

idx = 1;
while idx < length(u_act) && u_act(idx+1) == u_act(1)
    idx = idx + 1;
end
y_offset = mean(y_act(1:idx));
y = y_act - y_offset;

N      = length(u);
Nhalf  = floor(N/2);
kstart = 10;

%% Step 1: Identify a first-order model (Eq. 9): y(k) = -a1*y(k-1) + b1*u(k-1)
k1 = (kstart:Nhalf)';
Y1 = y(k1);
Phi1 = [y(k1-1) u(k1-1)];
theta1_hat = inv(Phi1'*Phi1) * (Phi1'*Y1);
a1_1st = -theta1_hat(1);
b1_1st =  theta1_hat(2);

fprintf('First-order model: a1 = %.6f, b1 = %.6f\n', a1_1st, b1_1st);
sys_tf1 = tf([b1_1st], [1 a1_1st], Ts)

%% Re-identify the second-order model (same as Lab Exercise, Step 2)
k2 = (kstart:Nhalf)';
Y2 = y(k2);
Phi2 = [y(k2-1) y(k2-2) u(k2-1) u(k2-2)];
theta2_hat = inv(Phi2'*Phi2) * (Phi2'*Y2);
a1_2nd = -theta2_hat(1);
a2_2nd = -theta2_hat(2);
b1_2nd =  theta2_hat(3);
b2_2nd =  theta2_hat(4);

fprintf('Second-order model: a1 = %.6f, a2 = %.6f, b1 = %.6f, b2 = %.6f\n', a1_2nd, a2_2nd, b1_2nd, b2_2nd);
sys_tf2 = tf([b1_2nd b2_2nd], [1 a1_2nd a2_2nd], Ts)

%% Step 2: Simulate both models using the entire offset-free input, compare with y
t_full = (0:numel(u)-1)' * Ts;
y_sim1_full = lsim(sys_tf1, u, t_full);
y_sim2_full = lsim(sys_tf2, u, t_full);

MSE1 = mean((y - y_sim1_full).^2);
MSE2 = mean((y - y_sim2_full).^2);
fprintf('\nMSE1 (1st order) = %.8f\n', MSE1);
fprintf('MSE2 (2nd order) = %.8f\n\n', MSE2);

figure;
plot(t_full, y, 'r'); hold on;
plot(t_full, y_sim1_full, 'g:', 'LineWidth', 1.5);
plot(t_full, y_sim2_full, 'b--', 'LineWidth', 1.2);
hold off;
title('Comparison of Different Offset-Free Order Models');
xlabel('Time (sec)'); ylabel('Water Level (V)');
legend('Actual Output', '1st Order Model Response', '2nd Order Model Response', 'Location', 'best');
grid on;
yl = ylim;
xl = xlim;
text(xl(1) + 0.02*(xl(2)-xl(1)), yl(2) - 0.08*(yl(2)-yl(1)), sprintf('MSE1 = %.8f', MSE1));
text(xl(1) + 0.02*(xl(2)-xl(1)), yl(2) - 0.16*(yl(2)-yl(1)), sprintf('MSE2 = %.8f', MSE2));

%% Step 3: Which model would you choose for controller design, and why?
fprintf('--- Answer to Post-lab Step 3 ---\n');
if MSE2 < MSE1
    fprintf('The 2nd order model has the lower MSE (%.8f vs %.8f), so it tracks\n', MSE2, MSE1);
    fprintf('the real tank more accurately, particularly through the transients.\n');
else
    fprintf('The 1st order model has the lower (or equal) MSE (%.8f vs %.8f).\n', MSE1, MSE2);
end
fprintf('If the two MSE values are close, the 1st order model is usually the better\n');
fprintf('practical choice for controller design: fewer states/parameters means a\n');
fprintf('simpler observer and controller (e.g. easier pole placement, lower real-time\n');
fprintf('computation), and it is less sensitive to noise in the identification data.\n');
fprintf('The 2nd order model is worth the extra complexity only if its accuracy gain\n');
fprintf('is large enough to matter for the specific control requirements (e.g. tight\n');
fprintf('settling-time or overshoot specs) - otherwise the simpler model wins.\n');
