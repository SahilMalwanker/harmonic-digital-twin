clc;
clear;

%% Define PT2 Controller Parameters
TR = 0.08;         % Rise time (s)
dR = 1;            % Damping ratio

a0 = 1 / TR^2;
a1 = 2 * dR / TR;

fprintf('a0 = %.4f\n', a0);
fprintf('a1 = %.4f\n', a1);

%% Motion State at Current Time
qd = 1.2;          % Desired angle (rad)
q = 1.1;           % Actual angle (rad)
q_dot = 1.8;       % Angular velocity (rad/s)

% Compute control error term
r0 = a0 * (qd - q) - a1 * q_dot;
fprintf('r0 = %.4f\n', r0);

%% Dynamic Parameters from Prior Model
M_star = 0.3377;   % From Task_4_1

% Evaluate b*(q, q_dot)
cos_q = cos(q);    % q is in radians already
b_star = 0.9245 * q_dot + 0.8881 * cos_q;

%% Final Control Input
Us = M_star * r0 + b_star;

fprintf('b* = %.4f\n', b_star);
fprintf('Us(t*) = %.4f\n', Us);
