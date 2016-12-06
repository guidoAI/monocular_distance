%---ODE definition---------------------------------------------------------
function dydt = ode_comp_tau_dot_perfect_measurements_wind(t , state, observations, parameters)

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
omega_z = -state(vzind) / state(zind);
uz =  parameters.gravity + parameters.K_z * (omega_z - parameters.ref_omega_z); % mass * gravity?
v_air = parameters.v_air_up - state(vzind);
drag = double(sign(v_air)) .* 0.5 .* parameters.rho .* parameters.CD .* parameters.A .* v_air.^2;
dydt(zind) = state(vzind); % dz
dydt(vzind) = ((uz + drag)/state(massind) - parameters.gravity); % dvz
dydt(uzind) = uz - state(uzind);


% % enforce maximum thrust
% nc = sqrt(ux*ux + uz*uz + uy*uy);
% if(nc > parameters.max_thrust)
%     ux = (ux / nc) * parameters.max_thrust;
%     uy = (uy / nc) * parameters.max_thrust;
%     
%     % is this the correct way?
%     % Guido:
%     uz = (uz / nc) * parameters.max_thrust;
%     %     state(uzind) = uz;
%     % Dario:
%     % uz = uz / nc;
% end
%     % Dynamics : system of equations
%     dydt = zeros(11,1);    % a column vector
%     dydt(xind) = state(vxind); % dx
%     dydt(vxind) = ux/state(massind); % dvx
%     dydt(yind) = state(vyind); % dy
%     dydt(vyind) = uy/state(massind); % dvy
%     dydt(zind) = state(vzind); % dz
%     dvz = (uz/state(massind) - parameters.gravity);
%     dydt(vzind) = dvz; % dvz
%     % dydt(7) = -sqrt(ux^2 + uy^2 + state(uzind)^2 ) / (parameters.Isp * parameters.g0); % dm
%     dydt(massind) = -sqrt(ux^2 + uy^2 + uz^2 ) / (parameters.Isp * parameters.g0); % dm
%     % limit du:
%     tau_dot = (state(zind) * dvz) / state(vzind)^2 - 1;
%     desired_ux = parameters.K_ventral * (state(vxind) / state(zind) - parameters.ref_omega_x);
%     desired_dux = desired_ux - state(uxind);
%     desired_uy = 0; %parameters.K_ventral * (observations(omega_y_ind)- parameters.ref_omega_y);
%     desired_duy = desired_uy - state(uyind);
%     if(observations(ftind) >= 0)
%         desired_duz = parameters.K_z * (tau_dot - parameters.ref_tau_dot);
%     else
%         % thrust downwards, if we are going up:
%         desired_duz = -parameters.max_d_thrust;
%         % can't we invert the above law, in order not to get excessive
%         % thrusting?
%     end
%     ndu = sqrt(desired_dux*desired_dux + desired_duy*desired_duy + desired_duz*desired_duz);
%     if(ndu > parameters.max_d_thrust)
%         dux = (desired_dux / ndu) * parameters.max_d_thrust;
%         duy = (desired_duy / ndu) * parameters.max_d_thrust;
%         duz = (desired_duz / ndu) * parameters.max_d_thrust;
%     else
%         dux = desired_dux;
%         duy = desired_duy;
%         duz = desired_duz;
%     end
%     if(state(uzind) > parameters.max_thrust && desired_duz > 0)
%         duz = 0;
%     elseif(state(uzind) < 0 && desired_duz < 0)
%         duz = 0;
%     end
%     
%     dydt(uxind) = dux;
%     dydt(uyind) = duy;
%     dydt(uzind) = duz;
%     % update the integral of the error (only used for logging purposes here):
%     dydt(interrind) = (tau_dot - parameters.ref_tau_dot * (state(zind) / -state(vzind)));
end

