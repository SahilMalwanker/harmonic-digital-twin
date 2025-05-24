% Design a PI velocity controller. We consider the denominator of the 
% closed loop as a denominator of a PT2-term with dRv = 0.9 and TRv = 0.08.

%% Calculate KP and TN

dRv = 0.9;           % Damping ratio
TRv = 0.08;          % Time constant

% PI Controller Design
Tn = 2 * dRv * TRv;             % Integral time (Tn)
Kp = Tn / TRv^2;                % Proportional gain (Kp)

%% Simulate and print the step response of the inner velocity control loop.

% First run CascadedControlSystem.slx

% Plot response
plot(out.tout, out.Step_Response);
xlabel('Time(sec)');
ylabel('Magnitude');
legend('Step Response');
grid on;

%% Determine the rise time and overshoot from the figure and approximate the step response by a PT2 term.

V_inf = out.Step_Response(end);  % Respose at t = inf
V_max = max(out.Step_Response);  % Max value of Res.

Ov = (V_max - V_inf) / V_inf;   % overshoot
dv = 1/sqrt(1+(pi/log(Ov))^2);  % damping ratio

% % Get step response metrics (including rise time)
info = stepinfo(out.Step_Response, out.tout, 'RiseTimeThreshold', [0, 1]);
T_an = info.RiseTime;  % 0 to 100 % rise time

%%  Design the position controller KL on the basis of the PT2 approximation.

Tv = (T_an * sqrt(1 - dv^2)) / (pi - acos(dv));  % Tv as per doc. 

% As Kl should be in range of 0.2/Tv < Kl < 0.3/Tv.
% Choosing mean value.
Kl = 0.25/Tv;   % Propotional gain Kl for position control loop.

%% --- Display Results ---
fprintf('\nStep Response Analysis:\n');
fprintf('V_inf = %.4f\n', V_inf);
fprintf('V_max = %.4f\n', V_max);
fprintf('Overshoot (Ov) = %.4f (%.2f%%)\n', Ov, Ov * 100);
fprintf('Rise Time (T_an) = %.4f s\n', T_an);
fprintf('Estimated Time Constant (Tv) = %.4f s\n', Tv);
fprintf('Position Controller Gain (Kl) = %.4f\n', Kl);