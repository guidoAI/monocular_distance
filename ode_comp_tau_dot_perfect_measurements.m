%---ODE definition---------------------------------------------------------
function dydt = ode_comp_tau_dot_perfect_measurements(t , state, observations, parameters)

% state = [x vx y vy z vz m uz]
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

% determine ux and uy, which directly depend on the observations:
ux = state(uxind);
uy = state(uyind); 

% Dynamics : system of equations
dydt = zeros(10,1);    % a column vector
dydt(xind) = state(vxind); % dx
dydt(vxind) = ux/state(massind); % dvx
dydt(yind) = state(vyind); % dy
dydt(vyind) = uy/state(massind); % dvy
desired_ux = parameters.K_ventral * (state(vxind) / state(zind) - parameters.ref_omega_x);
desired_dux = desired_ux - state(uxind);
dydt(uxind) = desired_dux;
desired_uy = 0; %parameters.K_ventral * (observations(omega_y_ind)- parameters.ref_omega_y);
desired_duy = desired_uy - state(uyind);
dydt(uyind) = desired_duy;

% z-axis dynamics / control:
uz = get_thrust_perfect_measurements(state, parameters);
dydt(zind) = state(vzind); % dz
dydt(vzind) = (uz/state(massind) - parameters.gravity); % dvz
dydt(uzind) = uz - state(uzind);
