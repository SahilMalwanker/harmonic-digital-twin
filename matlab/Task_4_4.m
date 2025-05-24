% Task 4.4: Calculate ramp profile timings for trapezoidal velocity motion
% Given parameters
v_m = 1.5;     % Maximum velocity [rad/s]
b_m = 3;       % Maximum acceleration/deceleration [rad/s²]
s_e = 2*pi;    % Total angular displacement [rad]

% Time calculations
t_b = v_m / b_m;         % Time to accelerate/decelerate [s]
t_e = s_e / v_m + t_b;   % Total motion time (accel + const vel + decel) [s]
t_v = t_e - t_b;         % Time at constant velocity [s]

% Display results
fprintf('Accel/Decel time (t_b): %.4f s\n', t_b);
fprintf('Total motion time (t_e): %.4f s\n', t_e);
fprintf('Constant vel time (t_v): %.4f s\n', t_v);