function uz = get_thrust_perfect_measurements(state, parameters)
% function uz = get_thrust_perfect_measurements(state, parameters)
% 
% Implementation of control law for constant tau / tau-dot:

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
    % retrieve vertical acceleration:
    az = (state(uzind) / state(massind) - parameters.gravity);
    % get the "perfect" tau dot:
    tau_dot = (state(zind) * az) / (state(vzind) * state(vzind)) - 1;
    % put a I-gain on uz:
    desired_duz = -parameters.K_z * (tau_dot - parameters.ref_tau_dot);
    uz = state(uzind) + desired_duz;
elseif(~isempty(parameters.ref_divergence))
    % perfect divergence measurement based on the state
    divergence = -state(vzind) / state(zind); 
    % simple P-gain with an offset due to gravity:
    uz =  state(massind) * (parameters.gravity + parameters.K_z * (divergence - parameters.ref_divergence)); 
end