function parameters = get_standard_parameters()

% parameters:

% environment:
parameters.gravity = 9.81;
parameters.g0 = 9.81;

%drag:
parameters.rho = 1.204; % (at 20 C)
parameters.CD = 0.25; % 1
parameters.A = 0.25; %1
parameters.v_air_up = 0; % 2
% whether there are wind gusts:
parameters.wind_gusts = false;
parameters.wg_magnitude = 1;
parameters.wg_period = 1;
% planned single disturbance:
parameters.planned_disturbance = false;
parameters.disturbance_time = 0.1;
parameters.disturbance_duration = 0.25;
parameters.disturbance_magnitude = 1;
% whether actuator effectiveness changes with the wind:
parameters.actuator_effectiveness = false;

% thruster:
parameters.max_thrust = 45760; % maximum thrust
parameters.Isp = 311; % specific impulse thruster
% limit the change in thrust per second, duz / s:
parameters.max_d_thrust = 6000; %3000 for optimal landing

% standard state if none else provided:
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
state(xind) = 0; 
state(vxind) = 0; 
state(yind) = 0; 
state(vyind) = 0; 
state(zind) = 20; 
state(vzind) = -2; % hover test needs a slight offset!!!
state(massind) = 1;
state(uxind) = 0;
state(uyind) = 0;
state(uzind) = state(massind) * parameters.gravity;
parameters.state = state;

% control params:
parameters.K_ventral = 10;
parameters.K_z = 50; % Kz = [50, 25, 5] -> make a function to plot all z together (landing)
                     % Kz = [2,10,50] at 2m / 5m: 10 is too heavy for 2
                     % meters but not for 5, 50 is then still too heavy,
                     % but not at 20m. 
                     
                     % Identifying what happens around the
                     % critical points is most interesting!!!
parameters.I_z = 1;
parameters.pK_z = 0.1;   
parameters.iK_z = 0.0001;%0.02; 
parameters.dK_z = 0;
parameters.limit_error = false;

% gain function:
% Kz = slope_z * z + bias_z
parameters.gain_function = false; 
parameters.slope_z = 1/0.07; 
parameters.bias_z = 0;
parameters.min_Kz = 1;
parameters.kappa = 100; % K_z / I_z
parameters.gf_noise_std = 0; 
parameters.gain_function_delay_steps = 0;

% reference parameters:
parameters.ref_omega_x = state(vxind) / state(zind);
parameters.ref_omega_z = -state(vzind) / state(zind); % hover test shouldn't set it to -vz / z!
parameters.ref_omega_y = state(vyind) / state(zind);
parameters.ref_divergence = -state(vzind) / state(zind); % hover test shouldn't set it to -vz / z!
parameters.ref_tau = -state(zind) / state(vzind);
parameters.ref_tau_dot = []; % used to be 0
parameters.ref_cov = -1;

% parameters for end-of-landing detection:
parameters.CC_threshold = 0.01;
parameters.div_threshold = 0.01;
parameters.window_size = 20;
parameters.phase_threshold = pi / 2;
parameters.magnitude_threshold = 12;

% type of dynamics / model:
PERFECT_LANDING = 1;
PERFECT_MEASUREMENTS = 2;
PM_ZOH = 3;
PM_ZOH_DELAY = 4;
parameters.method = PM_ZOH_DELAY;

% camera = 1 / time step:
parameters.FPS = 30;
% time delay:
parameters.delay_steps = 5;

% method of PIO detection:
COV_UZ_DIV = 1;
COV_DIV_PAST = 2;
parameters.PIO_method = COV_UZ_DIV;
parameters.past_steps = 2*parameters.delay_steps;

% end of the simulation:
parameters.max_extra_height = 10; % the robot should not end up this in m higher than the initial state
parameters.landing_altitude = 0.1;
parameters.time_threshold = 240;
parameters.reached_cov = true;
parameters.cov_interval = 0.1;

% show plots:
parameters.graphics = true;