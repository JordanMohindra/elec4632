%% ELEC4632 Lab 5 - Pre-lab: numerical check of the derivations
% The derivations themselves are written up in ELEC4632_Lab5_Prelab.md.
% This script checks each answer numerically:
%   (a) ZOH discretisation of Kp + Ki/s  ->  Kp + Ki*h/(z-1)
%   (b) the difference equation reproduces G_DT(z)
%   (c) the closed-loop formulas for Y/Yref and U/Yref match feedback()
% Uses the identified model and chosen gains from Lab5_Design.mat if it
% exists (run ELEC4632_Lab5_Exercise.m first), otherwise the values below.

clear; clc;

if exist('Lab5_Design.mat', 'file')
    D  = load('Lab5_Design.mat');
    Kp = D.Kp;  Ki = D.Ki;  h = D.Ts;
    a1 = D.a1;  a2 = D.a2;  b1 = D.b1;  b2 = D.b2;
    fprintf('Using Lab5_Design.mat\n');
else
    Kp = 1.30;  Ki = 0.045;  h = 0.75;
    a1 = -1.377233;  a2 = 0.391909;  b1 = -0.006530;  b2 = 0.050939;
    fprintf('Lab5_Design.mat not found - using default values\n');
end
fprintf('Kp = %.4f, Ki = %.4f, h = %.2f s\n', Kp, Ki, h);
fprintf('G(z) = (%.6f z + %.6f)/(z^2 + %.6f z + %.6f)\n\n', b1, b2, a1, a2);

%% (a) ZOH discretisation
G_ct  = tf([Kp Ki], [1 0]);                    % Kp + Ki/s = (Kp s + Ki)/s
G_zoh = c2d(G_ct, h, 'zoh');                   % MATLAB's ZOH equivalent
G_eq1 = tf([Kp, Ki*h - Kp], [1 -1], h);        % Eq. (1): Kp + Ki*h/(z-1)
G_blk = tf(pid(Kp, Ki, 0, 0, h, 'IFormula', 'ForwardEuler'));  % Simulink PID block

[nz, dz] = tfdata(G_zoh, 'v');
[ne, de] = tfdata(G_eq1, 'v');
[nb, db] = tfdata(G_blk, 'v');
fprintf('(a) c2d(Kp + Ki/s, h, ''zoh''):  num = %s, den = %s\n', mat2str(nz, 6), mat2str(dz, 6));
fprintf('    Kp + Ki*h/(z-1):            num = %s, den = %s\n', mat2str(ne, 6), mat2str(de, 6));
fprintf('    Discrete PID block (FE):    num = %s, den = %s\n', mat2str(nb, 6), mat2str(db, 6));
fprintf('    max coefficient difference ZOH vs Eq. (1) = %.2e\n', ...
        max(abs([nz - ne, dz - de])));
fprintf(['    -> ZOH of the integrator 1/s is h/(z-1), which is the same as the\n' ...
         '       Forward Euler integrator used by the Simulink Discrete PID block.\n\n']);

%% (b) difference equation  u(k) = u(k-1) + Kp*e(k) + (Ki*h - Kp)*e(k-1)
rng(1);
e  = randn(200, 1);
u1 = zeros(size(e));                 % velocity form
uI = 0;  u2 = zeros(size(e));        % parallel form (what the PID block does)
for k = 1:numel(e)
    if k == 1, ukm1 = 0; ekm1 = 0; else, ukm1 = u1(k-1); ekm1 = e(k-1); end
    u1(k) = ukm1 + Kp*e(k) + (Ki*h - Kp)*ekm1;
    u2(k) = Kp*e(k) + uI;
    uI    = uI + Ki*h*e(k);
end
u_tf = filter(ne, de, e);            % response of G_DT(z) itself
fprintf('(b) max |difference equation - G_DT(z) response| = %.2e\n', max(abs(u1 - u_tf)));
fprintf('    max |parallel form           - G_DT(z) response| = %.2e\n\n', max(abs(u2 - u_tf)));

%% (c) closed-loop transfer functions
Nc = [Kp, Ki*h - Kp];   Dc = [1 -1];          % controller
Np = [b1 b2];           Dp = [1 a1 a2];       % process
Dcl  = conv(Dc, Dp) + [0 conv(Nc, Np)];       % (z-1)(z^2+a1 z+a2) + (Kp z + Ki h - Kp)(b1 z + b2)
Ny   = [0 conv(Nc, Np)];                      % Y/Yref numerator
Nu   = conv(Nc, Dp);                          % U/Yref numerator
Gcl_formula = tf(Ny, Dcl, h);
Gu_formula  = tf(Nu, Dcl, h);

c = Ki*h - Kp;
Dcl_hand = [1, a1 - 1 + Kp*b1, a2 - a1 + Kp*b2 + c*b1, c*b2 - a2];   % expanded by hand

G   = tf(Np, Dp, h);
Gcl = feedback(G_eq1*G, 1);
Gu  = feedback(G_eq1, G);

kk = (0:300)';
fprintf('(c) Y/Yref   numerator   = %s\n', mat2str(Ny, 6));
fprintf('    U/Yref   numerator   = %s\n', mat2str(Nu, 6));
fprintf('    common denominator   = %s\n', mat2str(Dcl, 6));
fprintf('    hand-expanded D(z)   = %s  (diff %.1e)\n', mat2str(Dcl_hand, 6), max(abs(Dcl - Dcl_hand)));
fprintf('    max |step(formula) - step(feedback)|  Y/Yref: %.2e   U/Yref: %.2e\n', ...
        max(abs(step(Gcl_formula, kk*h) - step(Gcl, kk*h))), ...
        max(abs(step(Gu_formula, kk*h) - step(Gu, kk*h))));
fprintf('    closed-loop poles    = %s (all inside the unit circle: %s)\n', ...
        mat2str(roots(Dcl).', 4), string(all(abs(roots(Dcl)) < 1)));
fprintf('    Gcl(1) = %.6f  (1 -> zero steady-state error to a step)\n', ...
        polyval(Ny, 1)/polyval(Dcl, 1));
fprintf('    U/Yref at z = 1 = %.4f = 1/G(1) = %.4f\n', ...
        polyval(Nu, 1)/polyval(Dcl, 1), 1/dcgain(G));
