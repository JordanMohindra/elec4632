%% ELEC4632 Lab 1 - Pre-lab Exercise: MATLAB Refresher
% Reproduces the sinusoidal data plot shown in Fig. 1 of the Lab 1 notes.

clear; clc; close all;

%% Step 1: Time vector - column vector, 0 to 100 sec, 0.1 sec spacing
t = (0:0.1:100)';

%% Step 2: Two sinusoids (period = 100 sec) plus uniform noise in [-0.2, 0.2]
w = 2*pi/100;                                  % = 0.02*pi rad/sec, period 100 sec
y1 = sin(w*t) + (-0.2 + 0.4*rand(size(t)));    % sin(0.02*pi*t) + noise
y2 = cos(w*t) + (-0.2 + 0.4*rand(size(t)));    % cos(0.02*pi*t) + noise

%% Step 3: Cut off the first 200 samples, shift time to start at zero
t_new  = t(201:end) - t(201);
y1_new = y1(201:end);
y2_new = y2(201:end);

%% Step 4: Matrix concatenation into an N-by-3 matrix
data_new = [t_new y1_new y2_new];

%% Step 5 & 6: Plot original data (top) and cut-off data (bottom)
figure;

subplot(2,1,1);
plot(t, y1, 'r'); hold on;
plot(t, y2, 'b'); hold off;
xlim([0 140]);
ylim([-1.5 1.5]);
title('Original Data');
xlabel('Time (sec)');
ylabel('Data');
grid on;
legend('sin(0.02\pi t)', 'cos(0.02\pi t)');

subplot(2,1,2);
plot(t_new, y1_new, 'g'); hold on;
plot(t_new, y2_new, 'Color', [0.6 0.7 0.8]); hold off;
xlim([0 140]);
ylim([-1.5 1.5]);
title('Cut-off Data');
xlabel('Time (sec)');
ylabel('Data');
grid on;
legend('sin(0.02\pi t)', 'cos(0.02\pi t)');
