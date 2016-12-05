function [uz, I] = get_thrust(state, parameters, states_over_time, delay_steps, I)
% function [uz, I] = get_thrust(state, parameters, states_over_time, delay_steps, I)
% 
% Implementation of control law for constant tau / tau-dot:

if(~exist('I', 'var'))
    I = []; % initialize and return as empty if the I-gain is not used
end

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

% control:
if(~isempty(parameters.ref_tau_dot))
    if(size(states_over_time, 1) <= delay_steps)
        % retrieve vertical acceleration:
        az = (state(uzind) / state(massind) - parameters.gravity);
        % get the "perfect" tau dot:
        tau_dot = (state(zind) * az) / (state(vzind) * state(vzind)) - 1;
    else
        % delayed measurement:
        % retrieve vertical acceleration:
        az = (states_over_time(end-delay_steps,uzind) / states_over_time(end-delay_steps,massind) - parameters.gravity);
        % get the "perfect" tau dot:
        tau_dot = (states_over_time(end-delay_steps,zind) * az) / (states_over_time(end-delay_steps,vzind) * states_over_time(end-delay_steps,vzind)) - 1;
    end
    % put an I-gain on uz:
    desired_duz = -parameters.K_z * (tau_dot - parameters.ref_tau_dot);
    uz = state(uzind) + desired_duz;
elseif(~isempty(parameters.ref_divergence))
    
    if(size(states_over_time, 1) <= delay_steps)
        % perfect divergence measurement based on the state:
        divergence = -state(vzind) / state(zind);
    else
        % delayed measurement:
        divergence = -states_over_time(end-delay_steps, vzind) / states_over_time(end-delay_steps, zind);
    end
    
    % simple P-gain with an offset due to gravity:
    uz =  state(massind) * (parameters.gravity + parameters.K_z * (divergence - parameters.ref_divergence)); 
    
    if(exist('I', 'var') && ~isempty(I))
       % add the I-gain:
       I = I + parameters.I_z * (divergence - parameters.ref_divergence);
       uz = uz + I;
    end
end