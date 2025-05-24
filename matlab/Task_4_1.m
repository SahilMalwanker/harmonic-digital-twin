clc;
clear;

%% Parameter Setup for Model-Based Control
% Define system parameters and constants
u = 160;                     % Control input voltage (V)
JA = 3.37e-4;                % Armature inertia (kg·m^2)
KM_I = 0.8475;               % Motor constant (V/A)
FM = 0.0015;                 % Viscous friction (Nm·s/rad)
M = 5.4;                     % Additional inertia term (kg·m^2)
m = 9.4;                     % Mass of the body (kg)
ls = 0.4;                    % Distance from pivot to center of mass (m)
g = 9.81;                    % Gravitational acceleration (m/s²)
C = 0.22;                    % System-specific constant

%% Effective Motor Constant
KM = C / KM_I;               % Adjusted motor constant
% KM is used for scaling the dynamic equations

%% Compute Model Parameter M*
% M* represents a normalized inertia term influenced by input voltage
M_star = (M + JA * u^2) / (KM * u);
fprintf('Model Parameter M* = %.4f\n', M_star);

%% Coefficients for Nonlinear Dynamics (b*)
% The term b*(q, q_dot) is modeled as: b* = k1 * q_dot + k2 * cos(q)

% Calculate k1 (coefficient of angular velocity)
k1 = (FM * u^2) / (KM * u);

% Calculate k2 (coefficient of gravitational torque component)
k2 = (m * g * ls) / (KM * u);

% Display results
fprintf('k1 = %.4f\n', k1);
fprintf('k2 = %.4f\n', k2);
fprintf('\nb*(q, q_dot) = %.4f * q_dot + %.4f * cos(q)\n', k1, k2);
