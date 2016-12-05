function perfect_constant_tau_dot_landing(tau, tau_dot_ref, height)
% function perfect_constant_tau_dot_landing(tau, tau_dot_ref, height)
%
% Input parameters:
% - tau: time-to-contact, a positive number.
% - tau_dot_ref: what the time derivative of time-to-contact should be
% (typically negative)
% - height: initial height of the landing

% get parameters:
parameters = get_standard_parameters();

% type of dynamics / model:
PERFECT_LANDING = 1;
PERFECT_MEASUREMENTS = 2;
PM_ZOH = 3;
PM_ZOH_DELAY = 4;
PM_WIND = 5;
PM_WIND_ZOH_DELAY = 6;
parameters.method = PERFECT_LANDING;

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

% we want a landing with a constant time-to-contact (tau) and its time derivative 
parameters.ref_tau_dot = tau_dot_ref; 
% tau = -z / vz  -> vz = -(z / tau)  
vz = -(height / tau); 
parameters.state(vzind) = vz;

% simulate the landing and show the figures:
[states_over_time, observations_over_time, parameters_over_time, T] = simulate_landing(parameters);

f = gcf;
set(f,'name','Perfect constant tau-dot landing','numbertitle','off');


