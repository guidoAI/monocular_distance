function plot_performance_gains_over_time(plot_trajectory)
% function plot_performance_gains_over_time(plot_trajectory)
%
% for different gain schemes, we will perturb the system at different times
% and gather statistics on the rise time, etc.

if(~exist('plot_trajectory', 'var') || isempty(plot_trajectory))
    % set to true if you want a plot of a single trajectory to show what
    % happens when the drone is perturbed:
    plot_trajectory = false;
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

if(~exist('parameters', 'var') || isempty(parameters)) 
    % get parameters
    parameters = get_standard_parameters();
end
if(~exist('markers', 'var') || isempty(markers))
    markers = {'x', 'o', '+'};
end

% no graphics during the landing:
parameters.graphics = false;

parameters.reached_cov = false;

% introduce wind gusts
parameters.wind_gusts = true; 
parameters.wg_magnitude = 0.25;
parameters.wg_period = 1;

parameters.v_air_up = 0; 

% slope parameters:
parameters.slope_z = 4; 
original_slope = parameters.slope_z;
parameters.bias_z = 0;
parameters.min_Kz = 0.1;

% type of dynamics / model:
PERFECT_LANDING = 1;
PERFECT_MEASUREMENTS = 2;
PM_ZOH = 3;
PM_ZOH_DELAY = 4;
PM_WIND = 5;
PM_WIND_ZOH_DELAY = 6;
HOVER_DE = 7;
LAND_DE = 8;
PM_WIND_ZOH_DELAY_PID = 9;
parameters.method = PM_WIND_ZOH_DELAY;

% variables for tracking different strategies
unstable = zeros(3,1);
height = zeros(3,1);
time_ind = zeros(3,1);

% planned disturbance:
parameters.planned_disturbance = true;
parameters.disturbance_time = 0.1;
parameters.disturbance_duration = 0.5;
parameters.disturbance_magnitude = 1;
std_disturbance = 3; 
average_disturbance = 9;

% initial conditions:
initial_z = 10; initial_vz = 0;

i = 1;
n_tests = 10; % number of tests per strategy to gather statistics
disturbance_times = 5:5:40;
for disturbance_time = disturbance_times    
    parameters.disturbance_time = disturbance_time;
    fprintf('Disturbance time = %d\n', disturbance_time);
    
    for tst = 1:n_tests
        % low gain
        fprintf('Low gain, experiment %d\n', i);
        if(~plot_trajectory)
            parameters.disturbance_magnitude = average_disturbance + std_disturbance*randn(1);
        else
            parameters.disturbance_magnitude = average_disturbance + std_disturbance;
        end
        if(parameters.disturbance_magnitude < 0)
            parameters.disturbance_magnitude = 0;
        end
        % perform landing:
        parameters.state(zind) = initial_z;
        parameters.state(vzind) = initial_vz;
        parameters.gain_function = false;
        parameters.K_z = 0.15 * parameters.slope_z * initial_z; 
        [states_over_time{1}, observations_over_time{1}, parameters_over_time{1}, T{1}] = simulate_landing(parameters);
        Div{1} = states_over_time{1}(:,vzind) ./ states_over_time{1}(:,zind);
        
        if(plot_trajectory)
            figure();
            set(gcf, 'Color', [1 1 1]);
            col_pl = [0 0 0.75];
            lower = -0.4; upper = 0.2;
            plot_variance([parameters.disturbance_time, parameters.disturbance_time+parameters.disturbance_duration],[lower, lower],[upper, upper],[0.9, 0.9, 0.9])
            hold on;
            plot(T{1}, Div{1}, ':', 'Color', col_pl, 'LineWidth', 2);
            plot([0, max(T{1})], [-parameters.ref_omega_z, -parameters.ref_omega_z], '--', 'Color', [0 0 0], 'LineWidth', 2)
        end
        
        % analyze the data after the disturbance:
        inds_larger = find(T{1} >= parameters.disturbance_time+parameters.disturbance_duration);
        if(length(inds_larger) > 3 && inds_larger(1) < length(Div{1}) - 10)
            ind_start = inds_larger(1);
            % get performance parameters:
            [rise_time, percentage_overshoot, mean_absolute_error] = get_performance_characteristics(Div{1}(ind_start:end), -parameters.ref_omega_z, T{1}(ind_start:end)- T{1}(ind_start));
            fprintf('Low gain: RT = %f, PO = %f %%, MAE = %f\n', rise_time, percentage_overshoot, mean_absolute_error);
            RT{1}(tst,i) = rise_time;
            PO{1}(tst,i) = percentage_overshoot;
            MAE{1}(tst,i) = mean_absolute_error;
        else
            % no data here, became unstable before...
            % these values are so large, we can exclude them from the range of
            % the plot:
            RT{1}(tst,i) = 100;
            PO{1}(tst,i) = 1000;
            MAE{1}(tst,i) = 100000;
        end
        [unstable(1), height(1), time_ind(1)] = get_height_instability( observations_over_time{1}, states_over_time{1}, parameters);
        if(unstable(1))
            fprintf('Gets unstable before landing at height %f m.\n', height(1));
        else
            fprintf('No instabilities before landing.\n');
        end
        
        % high gain:
        parameters.state(zind) = initial_z;
        parameters.state(vzind) = initial_vz;
        parameters.K_z = parameters.slope_z * initial_z;
        parameters.gain_function = false;
        [states_over_time{2}, observations_over_time{2}, parameters_over_time{2}, T{2}] = simulate_landing(parameters);
        
        Div{2} = states_over_time{2}(:,vzind) ./ states_over_time{2}(:,zind);
        
        if(plot_trajectory)
            col_pl = [0.75 0 0];
            plot(T{2}, Div{2}, '-.', 'Color', col_pl, 'LineWidth', 2);
        end
        
        % analyze the data after the disturbance:
        inds_larger = find(T{2} >= parameters.disturbance_time+parameters.disturbance_duration);
        if(length(inds_larger) > 3)
            ind_start = inds_larger(1);
            % get performance parameters:
            [rise_time, percentage_overshoot, mean_absolute_error] = get_performance_characteristics(Div{2}(ind_start:end), -parameters.ref_omega_z, T{2}(ind_start:end)- T{2}(ind_start));
            fprintf('High gain: RT = %f, PO = %f %%, MAE = %f\n', rise_time, percentage_overshoot, mean_absolute_error);
            RT{2}(tst,i) = rise_time;
            PO{2}(tst,i) = percentage_overshoot;
            MAE{2}(tst,i) = mean_absolute_error;
        else
            % no data here, became unstable before...
            % these values are so large, we can exclude them from the range of
            % the plot:
            RT{2}(tst,i) = 100;
            PO{2}(tst,i) = 1000;
            MAE{2}(tst,i) = 100000;
        end
        [unstable(2), height(2), time_ind(2)] = get_height_instability( observations_over_time{2}, states_over_time{2}, parameters);
        if(unstable(2))
            fprintf('Gets unstable before landing at height %f m.\n', height(2));
        else
            fprintf('No instabilities before landing.\n');
        end
        
        % gain function:
        parameters.state(zind) = initial_z;
        parameters.state(vzind) = initial_vz;
        parameters.gain_function = true;
        [states_over_time{3}, observations_over_time{3}, parameters_over_time{3}, T{3}] = simulate_landing(parameters);
        Div{3} = states_over_time{3}(:,vzind) ./ states_over_time{3}(:,zind);
        
        if(plot_trajectory)
            col_pl = [0 0.75 0];
            plot(T{3}, Div{3}, 'Color', col_pl, 'LineWidth', 2);
            xlabel('Time (s)','fontsize',12,'fontweight','b');
            ylabel('Divergence (1/s)','fontsize',12,'fontweight','b');
        end
        % analyze the data after the disturbance:
        inds_larger = find(T{3} >= parameters.disturbance_time+parameters.disturbance_duration);
        if(length(inds_larger) > 3)
            ind_start = inds_larger(1);
            % get performance parameters:
            [rise_time, percentage_overshoot, mean_absolute_error] = get_performance_characteristics(Div{3}(ind_start:end), -parameters.ref_omega_z, T{3}(ind_start:end)- T{3}(ind_start));
            fprintf('Linearly decreasing gain: RT = %f, PO = %f %%, MAE = %f\n', rise_time, percentage_overshoot, mean_absolute_error);
            RT{3}(tst,i) = rise_time;
            PO{3}(tst,i) = percentage_overshoot;
            MAE{3}(tst,i) = mean_absolute_error;
        else
            % no data here, became unstable before...
            % these values are so large, we can exclude them from the range of
            % the plot:
            RT{3}(tst,i) = 100;
            PO{3}(tst,i) = 1000;
            MAE{3}(tst,i) = 100000;
        end
        [unstable(3), height(3), time_ind(3)] = get_height_instability( observations_over_time{3}, states_over_time{3}, parameters);
        if(unstable(3))
            fprintf('Gets unstable before landing at height %f m.\n', height(3));
        else
            fprintf('No instabilities before landing.');
        end
        if(plot_trajectory)
            rt = max([RT{1}(tst,i), RT{2}(tst,i), RT{3}(tst,i)])
            axis([min(T{3}), parameters.disturbance_time + 2 * rt, lower, upper]);
            keyboard;
        end
        
    end
    i = i + 1;

    save('RT', 'RT');
    save('PO', 'PO');
    save('MAE', 'MAE');
    
end

dt = 0.4;
j = 1;
for disturbance_time = disturbance_times
    Ls{j} = '';
    j = j + 1;
end

stable_inds_low = find(mean(RT{1}) < 10);
stable_inds_high = find(mean(RT{2}) < 10);

COMPACT_PLOT = true;
figure();
set(gcf, 'Color', [1 1 1]);
plot(disturbance_times(stable_inds_low)-dt, median(RT{1}(:,stable_inds_low)), ':', 'Color', [0 0 1], 'LineWidth', 2);
hold on;
plot(disturbance_times(stable_inds_high)+dt, median(RT{2}(:,stable_inds_high)), '-.', 'Color', [1 0 0], 'LineWidth', 2);
plot(disturbance_times, median(RT{3}), 'Color', [0 1 0], 'LineWidth', 2);
if(COMPACT_PLOT)
    boxplot(RT{1}, 'PlotStyle','compact', 'Color', [0 0 1], 'positions', disturbance_times-dt, 'labels', Ls);
    hold on;
    boxplot(RT{2}, 'PlotStyle','compact', 'Color', [1 0 0], 'positions', disturbance_times+dt, 'labels', Ls);
    boxplot(RT{3}, 'PlotStyle','compact', 'Color', [0 1 0], 'positions', disturbance_times, 'labels', Ls);
else
    boxplot(RT{1}, 'Color', [0 0 1], 'positions', disturbance_times-dt, 'labels', Ls);
    hold on;
    boxplot(RT{2}, 'Color', [1 0 0], 'positions', disturbance_times+dt, 'labels', Ls);
    boxplot(RT{3}, 'Color', [0 1 0], 'positions', disturbance_times, 'labels', Ls);
end
set(gca,'xtickmode','auto','xticklabelmode','auto')
ylim([0 5]);
xlabel('Disturbance time (s)','fontsize',12,'fontweight','b');
ylabel('Rise time after disturbance (s)','fontsize',12,'fontweight','b');
legend('Low fixed gain K', 'High fixed gain K', 'Linearly decreasing gain K')
figure();
set(gcf, 'Color', [1 1 1]);
plot(disturbance_times(stable_inds_low)-dt, median(PO{1}(:,stable_inds_low)), ':', 'Color', [0 0 1], 'LineWidth', 2);
hold on;
plot(disturbance_times(stable_inds_high)+dt, median(PO{2}(:,stable_inds_high)), '-.', 'Color', [1 0 0], 'LineWidth', 2);
plot(disturbance_times, median(PO{3}), 'Color', [0 1 0], 'LineWidth', 2);
if(COMPACT_PLOT)
    boxplot(PO{1}, 'PlotStyle','compact', 'Color', [0 0 1], 'positions', disturbance_times-dt, 'labels', Ls);
    hold on;
    boxplot(PO{2}, 'PlotStyle','compact', 'Color', [1 0 0], 'positions', disturbance_times+dt, 'labels', Ls);
    boxplot(PO{3}, 'PlotStyle','compact', 'Color', [0 1 0], 'positions', disturbance_times, 'labels', Ls);
else
    boxplot(PO{1}, 'Color', [0 0 1], 'positions', disturbance_times-dt, 'labels', Ls);
    hold on;
    boxplot(PO{2}, 'Color', [1 0 0], 'positions', disturbance_times+dt, 'labels', Ls);
    boxplot(PO{3}, 'Color', [0 1 0], 'positions', disturbance_times, 'labels', Ls);    
end
set(gca,'xtickmode','auto','xticklabelmode','auto')
xlabel('Disturbance time (s)','fontsize',12,'fontweight','b');
ylabel('Percentage overshoot (%)','fontsize',12,'fontweight','b');
legend('Low fixed gain K', 'High fixed gain K', 'Linearly decreasing gain K')

function plot_variance(x,lower,upper,color)
set(fill([x,x(end:-1:1), x(1)],[upper,lower(end:-1:1), lower(1)],color),'EdgeColor',color);