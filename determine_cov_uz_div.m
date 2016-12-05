function [observations, valid] = determine_cov_uz_div(states_over_time, delay_steps, parameters)
% function [observations, valid] = determine_cov_uz_div(states_over_time, delay_steps, parameters)

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

% observations
cc_ind = 1;

% window for correlating Uz and Div:
window_size = parameters.window_size;

% whether the cov is valid:
valid = false;

% determine the covariance:
if(size(states_over_time, 1) <= window_size+delay_steps+1)
    observations(cc_ind) = 0;
else
    % uz, div vectors for correlation:
    UZ = states_over_time(max([end-window_size, 1]):end, uzind);
    % TODO: reform everything to make div the right sign:
    DIV = states_over_time(max([end-window_size-delay_steps, 1]):end-delay_steps,vzind) ./ states_over_time(max([end-window_size-delay_steps, 1]):end-delay_steps,zind);
    if(length(UZ) > 5)
        % Determine the covariance between uz and divergence as a proxy for
        % self-induced oscillations:
        cov_uz_div = cov(UZ, DIV);
        observations(cc_ind) = cov_uz_div(1,2);
        valid = true;
    else
        observations(cc_ind) = 0;
    end
end