function control_with_perfect_measurements(height, velocity, tau_ref, tau_dot_ref)
% function control_with_perfect_measurements(height, velocity, tau_ref, tau_dot_ref)
% 
% Input parameters:
% - height: initial height of the landing (positive)
% - velocity: initial vertical velocity of the landing (typically negative)
% - tau_ref: what the time derivative of time-to-contact should be
% (typically positive) If empty [], tau dot ref control will be performed
% - tau_dot_ref: what the time derivative of time-to-contact should be
% (typically negative)

% get parameters:
parameters = get_standard_parameters();

% type of dynamics / model:
PERFECT_LANDING = 1;
PERFECT_MEASUREMENTS = 2;
PM_ZOH = 3;
PM_ZOH_DELAY = 4;
parameters.method = PERFECT_MEASUREMENTS;

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

% NOTE: implement your control in get_thrust_perfect_measurements.m!!

% simulate the landing and show the figures:
[states_over_time, observations_over_time, parameters_over_time, T] = simulate_landing(parameters);

f = gcf;
set(f,'name','Control with perfect measurements','numbertitle','off');
