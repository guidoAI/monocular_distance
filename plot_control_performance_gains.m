function plot_control_performance_gains()
% function plot_control_performance_gains()
%
% Performs a landing with a low gain, a high gain, and a linearly
% decreasing gain. It shows that a high gain has a better performance than
% a low gain at the start of the landing, but gets unstable really quickly.
% The low gain will get unstable later. The linearly decreasing gain has a
% high performance and does not get unstable.

% state = [x, vx, y, vy, z, vz, mass, yaw, pitch, roll]
xind = 1;
vxind = 2;
yind = 3;
vyind = 4;
zind = 5;
vzind = 6;
massind = 7;
uxind = 8;
uyind = 9;
uzind = 10;

if(~exist('parameters', 'var') || isempty(parameters)) 
    % get parameters
    parameters = get_standard_parameters();
end
if(~exist('markers', 'var') || isempty(markers))
    markers = {'x', 'o', '+'};
end

% no graphics during the landing
parameters.graphics = false;

% subject the control system to wind gusts:
parameters.wind_gusts = true; 
parameters.wg_magnitude = 0.25;
parameters.wg_period = 1;
parameters.v_air_up = 0; 

% slope parameters:
parameters.slope_z = 4;
original_slope = parameters.slope_z;
parameters.bias_z = 0;
parameters.min_Kz = 0.1;

% type of dynamics / model:
PERFECT_LANDING = 1; % perfectly follow the desired constant divergence trajectory
PERFECT_MEASUREMENTS = 2; % continuous, no delay
PM_ZOH = 3; % zero-order hold, sampling of the continuous signals
PM_ZOH_DELAY = 4; % ZOH plus a delay of 1 or more samples
PM_WIND = 5; % adding wind to a continuous, non-delayed model
PM_WIND_ZOH_DELAY = 6; % adding wind to a zero-order-hold, possibly delayed model
PM_WIND_ZOH_DELAY_PID = 7; % PID controller for the ZOH delayed model
ADAPTIVE_GAIN_CONTROL = 8; % adaptive gain control
parameters.method = PM_WIND_ZOH_DELAY;

% variables to track:
unstable = zeros(3,1);
height = zeros(3,1);
time_ind = zeros(3,1);

% initial conditions
initial_z = 10; initial_vz = 0;

% low gain
parameters.state(zind) = initial_z; 
parameters.state(vzind) = initial_vz;
parameters.K_z = 0.15 * parameters.slope_z * initial_z;
% simulate:
[states_over_time{1}, observations_over_time{1}, parameters_over_time{1}, T{1}] = simulate_landing(parameters);
Div{1} = states_over_time{1}(:,vzind) ./ states_over_time{1}(:,zind);
% get performance characteristics
[rise_time, percentage_overshoot, mean_absolute_error] = get_performance_characteristics(Div{1}, -parameters.ref_omega_z, T{1});
fprintf('Low gain: RT = %f, PO = %f %%, MAE = %f\n', rise_time, percentage_overshoot, mean_absolute_error);
% determine whether the system became unstable:
[unstable(1), height(1), time_ind(1)] = get_height_instability( observations_over_time{1}, states_over_time{1}, parameters);
if(unstable(1))
    fprintf('Gets unstable before landing at height %f m.\n', height(1));
else
    fprintf('No instabilities before landing.\n');
end

% high gain
parameters.state(zind) = initial_z; 
parameters.state(vzind) = initial_vz;
parameters.K_z = parameters.slope_z * initial_z;
% simulate:
[states_over_time{2}, observations_over_time{2}, parameters_over_time{2}, T{2}] = simulate_landing(parameters);
Div{2} = states_over_time{2}(:,vzind) ./ states_over_time{2}(:,zind);
% get performance characteristics
[rise_time, percentage_overshoot, mean_absolute_error] = get_performance_characteristics(Div{2}, -parameters.ref_omega_z, T{2});
fprintf('High gain: RT = %f, PO = %f %%, MAE = %f\n', rise_time, percentage_overshoot, mean_absolute_error);
% determine whether the system became unstable:
[unstable(2), height(2), time_ind(2)] = get_height_instability( observations_over_time{2}, states_over_time{2}, parameters);
if(unstable(2))
    fprintf('Gets unstable before landing at height %f m.\n', height(2));
else
    fprintf('No instabilities before landing.');
end

% gain function:
parameters.state(zind) = initial_z; 
parameters.state(vzind) = initial_vz;
parameters.gain_function = true;
% simulate:
[states_over_time{3}, observations_over_time{3}, parameters_over_time{3}, T{3}] = simulate_landing(parameters);
Div{3} = states_over_time{3}(:,vzind) ./ states_over_time{3}(:,zind);
% get performance characteristics
[rise_time, percentage_overshoot, mean_absolute_error] = get_performance_characteristics(Div{3}, -parameters.ref_omega_z, T{3});
fprintf('Linearly decreasing gain(s): RT = %f, PO = %f %%, MAE = %f\n', rise_time, percentage_overshoot, mean_absolute_error);
% determine whether the system became unstable:
[unstable(3), height(3), time_ind(3)] = get_height_instability( observations_over_time{3}, states_over_time{3}, parameters);
if(unstable(3))
    fprintf('Gets unstable before landing at height %f m.\n', height(3));
else
    fprintf('No instabilities before landing.');
end

% Plot the divergence over time:
LW = 2;
LW_REF = 2;
figure(); set(gcf, 'Color', [1 1 1]);
plot(T{1}, Div{1}, ':', 'Color', [0 0 1], 'LineWidth', LW);
hold on;
plot(T{2}, Div{2}, '-.', 'Color', [1 0 0], 'LineWidth', LW);
plot(T{3}, Div{3}, '-', 'Color', [0 1 0], 'LineWidth', LW);
Tend = max([max(T{1}), max(T{2}), max(T{3})]);
plot([T{1}(1), Tend], -[parameters.ref_omega_z, parameters.ref_omega_z], '--', 'Color', [0 0 0], 'LineWidth', LW_REF);
ylim([-0.5 0.5])
xlabel('Time (s)', 'FontWeight','bold'); ylabel('\omega_z (1/s)', 'FontWeight','bold')

% Plot the trajectory over time:
figure(); set(gcf, 'Color', [1 1 1]);
h1 = plot(T{1}, states_over_time{1}(:,zind), ':', 'Color', [0 0 1], 'LineWidth', LW);
hold on;
h2 = plot(T{2}, states_over_time{2}(:,zind), '-.', 'Color', [1 0 0], 'LineWidth', LW);
h3 = plot(T{3}, states_over_time{3}(:,zind), '-', 'Color', [0 1 0], 'LineWidth', LW);
if(unstable(1))
    plot(T{1}(time_ind(1)), height(1), 'x', 'Color', [0 0 1], 'LineWidth', LW, 'MarkerSize', 5);
end
if(unstable(2))
    plot(T{2}(time_ind(2)), height(2), 'x', 'Color', [1 0 0], 'LineWidth', LW, 'MarkerSize', 5);
end
if(unstable(3))
    plot(T{3}(time_ind(3)), height(3), 'x', 'Color', [0 1 0], 'LineWidth', LW, 'MarkerSize', 5);
end
% Plot the reference trajectory for a perfect landing (starting at the
% right velocity, instead of at 0 m/s)
[z, vz, az, t] = constant_tau_dot_formulas(initial_z, -parameters.ref_omega_z * initial_z, 0, max(T{3}), false);
h4 = plot(t, z, '--', 'Color', [0 0 0], 'LineWidth', LW_REF);
% Legends, labels:
ylim([0 initial_z])
xlabel('Time (s)', 'FontWeight','bold'); ylabel('z (m)', 'FontWeight','bold')
legend([h1, h2, h3, h4], {'Low gain','High gain', 'Linearly decreasing gain', 'Reference model'});