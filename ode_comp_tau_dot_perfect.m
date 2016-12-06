%---ODE definition---------------------------------------------------------
function dydt = ode_comp_tau_dot_perfect(t , state, observations, parameters)

% observations = [omega_x, omega_y, tau_dot]
omega_x_ind = 1;
omega_y_ind = 2;
tau_dot_ind = 3;
ftind = 4;
mtind = 5;

% state = [x vx y vy z vz m uz]
xind = 1;
vxind = 2;
yind = 3;
vyind = 4;
zind = 5;
vzind = 6;

% c^2 determines the tau dot to be followed:
c2 = -parameters.ref_tau_dot;

% Dynamics : system of equations
dydt = zeros(10,1);    % a column vector
dydt(xind) = state(vxind); % dx
dydt(vxind) = 0; %observations(omega_x_ind) * state(vzind); % dvx
dydt(yind) = state(vyind); % dy
dydt(vyind) = 0; %observations(omega_y_ind) * state(vzind); % dvx; % dvy
dydt(zind) = state(vzind); % dz
% cdTTC:
dydt(vzind) = (1-c2) * (state(vzind) * state(vzind)) / state(zind); % dz

end

