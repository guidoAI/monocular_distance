function control_with_delay(height, velocity, tau_ref, tau_dot_ref, delay_steps, FPS)
% function control_with_delay(height, velocity, tau_ref, tau_dot_ref, delay_steps, FPS)
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
parameters.method = PM_ZOH_DELAY;

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
% tau-dot or tau control:
if(isempty(tau_ref))
    % set tau dot ref:
    parameters.ref_tau_dot = tau_dot_ref;
    parameters.ref_divergence = [];
    parameters.ref_tau = [];
    parameters.K_z = 0.1;
else
    % set divergence ref (change if you want to do pure tau-control):
    parameters.ref_divergence = 1 / tau_ref;
    parameters.ref_tau_dot = [];
    parameters.ref_tau = [];
    parameters.K_z = 10;
end

% NOTE: implement your control in get_thrust.m!!

% simulate the landing and show the figures:
[states_over_time, observations_over_time, parameters_over_time, T] = simulate_landing(parameters);

f = gcf;
set(f,'name','Control with delay','numbertitle','off');