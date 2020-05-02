function [unstable, height, time_ind] = get_height_instability( observations_over_time, states_over_time, parameters)
% function [unstable, height, time_ind] = get_height_instability( observations_over_time, states_over_time, parameters)
%
% Automatic detection of instability by means of the covariance
% observation.

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

% observations:
cc_ind = 1;

% instability:
inds = find(observations_over_time(:, cc_ind) < parameters.ref_cov);

% set the return values:
if(isempty(inds))
    unstable = false;
    height = -1;
    time_ind = -1;
else
    unstable = true;
    time_ind = inds(1);
    height = states_over_time(time_ind, zind);
end