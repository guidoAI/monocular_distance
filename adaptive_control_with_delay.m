function [states_over_time, observations_over_time, parameters_over_time, T] = adaptive_control_with_delay(height, velocity, div_ref, delay_steps, FPS, reached_cov)
% function [states_over_time, observations_over_time, parameters_over_time, T] = adaptive_control_with_delay(height, velocity, div_ref, delay_steps, FPS, reached_cov)
% 
% Input parameters:
% - height: initial height of the landing (positive)
% - velocity: initial vertical velocity of the landing (typically negative)
% - tau_ref: what the time derivative of time-to-contact should be
% (typically positive) If empty [], tau dot ref control will be performed
% - tau_dot_ref: what the time derivative of time-to-contact should be
% (typically negative)
% - delay_steps: one time step is 1/parameters.FPS seconds. delay_steps is
% the delay in time steps between an event and its observation by the
% robot, so that it can be used for control.
% - FPS: frames-per-second. Determines the size of a time step.

% get parameters:
parameters = get_standard_parameters();

% type of dynamics / model:
PERFECT_LANDING = 1;
PERFECT_MEASUREMENTS = 2;
PM_ZOH = 3;
PM_ZOH_DELAY = 4;
PM_WIND = 5; % adding wind to a continuous, non-delayed model
PM_WIND_ZOH_DELAY = 6; % adding wind to a zero-order-hold, possibly delayed model
PM_WIND_ZOH_DELAY_PID = 7; % PID controller for the ZOH delayed model
ADAPTIVE_GAIN_CONTROL = 8; % adaptive gain control
parameters.method = ADAPTIVE_GAIN_CONTROL;

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

% set the height:
parameters.state(zind) = height;
parameters.state(vzind) = velocity;
if(~isempty(delay_steps))
    parameters.delay_steps = delay_steps;
end
if(~isempty(FPS))
    parameters.FPS = FPS;
end

parameters.ref_divergence = div_ref;
parameters.ref_tau_dot = [];
parameters.ref_tau = [];
parameters.K_z = 5 * height;

% whether there are wind gusts:
parameters.wind_gusts = true; % necessary to introduce small disturbances
parameters.wg_magnitude = 0.1;
parameters.wg_period = 2.5;

% parameters.ref_cov = -0.25;
% parameters.reached_cov = false; % if true, it will stop when reaching the covariance
% parameters.cov_interval = 0.1;
parameters.pK_z = 0.15;
parameters.iK_z = 0.01;
parameters.ref_cov = -0.05;
% stop if you reach the reference cov.
parameters.reached_cov = reached_cov;
parameters.cov_interval = 0.01;

% parameters.pK_z = 0.01;   
% parameters.iK_z = 0.00001;%0.02; 
% parameters.dK_z = 0;

% simulate the landing and show the figures:
[states_over_time, observations_over_time, parameters_over_time, T] = simulate_landing(parameters);

f = gcf;
set(f,'name','Adaptive control with delay','numbertitle','off');

figure('Name', 'Gain and cov over time', 'NumberTitle', 'off');
set(gcf, 'Color', [1 1 1]);
subplot(1,2,1);
plot(T, parameters_over_time(:,1), '--', 'Color', [0 0.6 0], 'LineWidth', 2);
hold on;
plot(T, parameters_over_time(:,2), 'Color', [0 0.9 0]);
xlabel('Time (s)');
ylabel('K_z state and used (-)');
legend({'K_z state', 'K_z used'});
subplot(1,2,2);
plot(T, observations_over_time(:,1));
hold on;
plot([1, max(T)], [parameters.ref_cov, parameters.ref_cov]);
xlabel('Time (s)');
ylabel('cov(u_z, div) (-)');
