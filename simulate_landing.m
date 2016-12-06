function [states_over_time, observations_over_time, parameters_over_time, T] = simulate_landing(parameters)

if(~exist('parameters', 'var') || isempty(parameters))
    parameters = get_standard_parameters();
end

% whether to show plots:
graphics = parameters.graphics;

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

% initialize the time span between image frames:
t = 0;
FPS = parameters.FPS;
t_step = 1 / FPS;

% ode settings:
ode_options = odeset('RelTol', 1e-8, 'AbsTol', 1e-8);
state_size_ode = 10;

% start state:
if(~exist('state', 'var') || isempty(state))
    state = parameters.state;
end
states_over_time = [state];
all_states = [state];

% initialize observations: []
cc_ind = 1;
observations = [0];
observations_over_time = [observations];

K_z_state_ind = 1;
K_z_used_ind = 2;
parameters_over_time = [parameters.K_z, parameters.K_z];

% window for correlating Uz and Div:
window_size = parameters.window_size;
% detection thresholds:
CC_threshold = parameters.CC_threshold;
div_threshold = parameters.div_threshold;

% type of dynamics / model - most have perfect measurements (PM):
PERFECT_LANDING = 1; % perfectly follow the desired constant divergence trajectory
PERFECT_MEASUREMENTS = 2; % continuous, no delay
PM_ZOH = 3; % zero-order hold, sampling of the continuous signals
PM_ZOH_DELAY = 4; % ZOH plus a delay of 1 or more samples
PM_WIND = 5; % adding wind to a continuous, non-delayed model
PM_WIND_ZOH_DELAY = 6; % adding wind to a zero-order-hold, possibly delayed model
PM_WIND_ZOH_DELAY_PID = 7; % PID controller for the ZOH delayed model
ADAPTIVE_GAIN_CONTROL = 8; % adaptive gain control
METHOD = parameters.method;
delay_steps = parameters.delay_steps;

% below this altitude, we have landed:
landing_altitude = parameters.landing_altitude;
time_threshold = parameters.time_threshold;
at_cov = false;
state_Kz = parameters.K_z;
add_Kz = 0;
T = 0;

% for integral control:
I = 0; 

while(state(zind) >= landing_altitude ...
        && state(zind) <= states_over_time(1,zind)+parameters.max_extra_height ...
        && T(end) < time_threshold ...
        && ~at_cov)
    
    tspan = [t, t+t_step];
    % TODO: observations var can be removed from the ode function
    % depending on the method, call the right ode function:
    if(METHOD == PERFECT_LANDING)
        [t,ode_out] = ode45(@(t,x) ode_comp_tau_dot_perfect(t, state, observations, parameters), tspan, state(1:state_size_ode), ode_options);
    elseif(METHOD == PERFECT_MEASUREMENTS)
        [t,ode_out] = ode45(@(t,x) ode_comp_tau_dot_perfect_measurements(t, state, observations, parameters), tspan, state(1:state_size_ode), ode_options);
    elseif(METHOD == PM_ZOH)
        state(uzind) = get_thrust(state, parameters, states_over_time, 0); % 0 delay steps
        [t,ode_out] = ode45(@(t,x) ode_comp_tau_dot_perfect_measurements_ZOH(t, state, observations, parameters), tspan, state(1:state_size_ode), ode_options);
    elseif(METHOD == PM_ZOH_DELAY)
        state(uzind) = get_thrust(state, parameters, states_over_time, delay_steps);
        [t,ode_out] = ode45(@(t,x) ode_comp_tau_dot_perfect_measurements_ZOH(t, state, observations, parameters), tspan, state(1:state_size_ode), ode_options);
    elseif(METHOD == PM_WIND)
        [t,ode_out] = ode45(@(t,x) ode_comp_tau_dot_perfect_measurements_wind(t, state, observations, parameters), tspan, state(1:state_size_ode), ode_options);
    elseif(METHOD == PM_WIND_ZOH_DELAY)
        state(uzind) = get_thrust(state, parameters, states_over_time, delay_steps);
        [t,ode_out] = ode45(@(t,x) ode_comp_tau_dot_perfect_measurements_ZOH_wind(t, state, observations, parameters), tspan, state(1:state_size_ode), ode_options);
    elseif(METHOD == PM_WIND_ZOH_DELAY_PID)
        % Actually only PI-gains for now:
        [state(uzind), I] = get_thrust(state, parameters, states_over_time, delay_steps, I);
        [t,ode_out] = ode45(@(t,x) ode_comp_tau_dot_perfect_measurements_ZOH_wind(t, state, observations, parameters), tspan, state(1:state_size_ode), ode_options);
    elseif(METHOD == ADAPTIVE_GAIN_CONTROL)
        % TODO: split the determination of cov(uz, div) and adaptive gain
        % control: the cov is also used for a fixed gain landing
        [observations, valid] = determine_cov_uz_div(states_over_time, delay_steps, parameters);
        if(valid)
            % only perform adaptive gain control if the covariance
            % measurement is valid:
            [state_Kz, add_Kz] = adapt_gain(parameters, states_over_time, observations, observations_over_time, delay_steps);
            parameters.K_z = state_Kz + add_Kz;
        end
        state(uzind) = get_thrust(state, parameters, states_over_time, delay_steps);
        [t,ode_out] = ode45(@(t,x) ode_comp_tau_dot_perfect_measurements_ZOH_wind(t, state, observations, parameters), tspan, state(1:state_size_ode), ode_options);
        parameters.K_z = state_Kz;
    end
    
    all_states = [all_states; ode_out];
    
    % get the state and put it in a matrix
    state(1:state_size_ode) = ode_out(end, :);
    
    % log all values for history tracking:
    states_over_time =  [states_over_time; state];
    observations_over_time = [observations_over_time; observations];
    parameters_over_time = [parameters_over_time; parameters.K_z, parameters.K_z + add_Kz];
    
    % possibly stop the simulator when reached a given cov:
    if(parameters.reached_cov)
        at_cov = false;
        [observations, valid] = determine_cov_uz_div(states_over_time, delay_steps, parameters);
        if(valid)
            err_cov = parameters.ref_cov - observations(cc_ind);
            % can be used to stop the simulation (for example in the hover
            % experiment)
            if(abs(err_cov) < parameters.cov_interval)
                at_cov = true;
            end
        end
    end
    
    % update the time vector:
    T = [T, T(end) + t_step];
    t = T(end);
end

% calculate accelerations:
az = gradient(states_over_time(:, vzind)) * FPS;

if(graphics)
    % plot standard graphs with state / observation info:
    figure();
    set(gcf, 'Color', [1 1 1]);
    subplot(2,2,1);
    plot(T, states_over_time(:,zind));
    xlabel('Time (s)');
    ylabel('z (m)');
    subplot(2,2,2);
    plot(T, states_over_time(:,vzind));
    xlabel('Time (s)');
    ylabel('v_z (m/s)');
    subplot(2,2,3);
    %     plot(T, states_over_time(:,uzind));
    %     hold on;
    plot(T, az, 'red');
    ylabel('a_z (m/s^2)');
    xlabel('Time (s)');
    subplot(2,2,4);
    %plot(T, - states_over_time(:,zind) ./ states_over_time(:,vzind));
    %ylabel('Time-to-contact (s)');
    plot(T, states_over_time(:,vzind) ./ states_over_time(:,zind));
    ylabel('Divergence (1/s)');
    xlabel('Time (s)');
end

