function [state_Kz, add_Kz] = adapt_gain(parameters, states_over_time, observations, observations_over_time, delay_steps)
% function [state_Kz, add_Kz] = adapt_gain(parameters, states_over_time, observations, observations_over_time, delay_steps)

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

% different ways to determine the covariance:
COV_UZ_DIV = 1;
COV_DIV_PAST = 2;

% determine the error in covariance:
err_cov = parameters.ref_cov - observations(cc_ind);

% Change the gain to enforce a chosen covariance.

% The hover experiments in the article were done by simply
% increasing the P-gain:
% K_z = parameters.K_z + parameters.iK_z;

% Here the more elaborate adaptive control is performed that was
% also used for landing on the edge of oscillation.

% Limit the error to have the error range symmetrical:
if(parameters.limit_error)
    FACTOR = 10;
    if(err_cov > FACTOR*abs(parameters.ref_cov))
        err_cov = FACTOR*abs(parameters.ref_cov);
    elseif(err_cov < -FACTOR*abs(parameters.ref_cov))
        err_cov = -FACTOR*abs(parameters.ref_cov);
    end
end
% If the system becomes too unstable, reduce the gain quickly:
if(parameters.PIO_method ~= COV_UZ_DIV && err_cov > 0.25) % 
    emergency_Kz = true;
else
    emergency_Kz = false;
end
% change the gain to enforce a chosen covariance, I-gain:
state_Kz = parameters.K_z - err_cov * (parameters.iK_z * parameters.K_z);
% The gain should stay positive:
if(parameters.K_z < 0)
    parameters.K_z = 0;
end
% information for the D-gain:
derr_cov =  observations(cc_ind) - observations_over_time(end, cc_ind);
% apply adaptive P-gain and I-gain:
add_Kz = (parameters.pK_z * parameters.K_z) * (-err_cov) + (parameters.dK_z * parameters.K_z) * (-derr_cov);
% set the gain to return:
if(state_Kz + add_Kz < 0)
    state_Kz = 1E-5;
    add_Kz = 0;
end

% apply emergency measure if necessary:
if(emergency_Kz)
    state_Kz = 0.5 * state_Kz;
    add_Kz = 0.5 * add_Kz;
end



