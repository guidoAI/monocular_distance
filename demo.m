% a script to check if everything works

% constant_tau_dot_formulas(height, velocity, tau_dot_ref, T);
fprintf('Landing formula...\n');
constant_tau_dot_formulas(10, -1, -0.1, 100);

% perfect_constant_tau_dot_landing(tau, tau_dot_ref, height);
fprintf('Perfect landing simulation...\n');
perfect_constant_tau_dot_landing(10, -0.1, 10);

% control_with_perfect_measurements(height, velocity, tau_ref, tau_dot_ref);
fprintf('Control with perfect measurements...\n');
control_with_perfect_measurements(10, -1, 10, []);

% control_with_delay(height, velocity, tau_ref, tau_dot_ref, delay_steps, FPS);
fprintf('Control with delayed perfect measurements...\n');
control_with_delay(10, -1, 10, [], 10, 30);

% adaptive_control_with_delay(height, velocity, div_ref, delay_steps, FPS, reached_cov)
fprintf('Adaptive gain control with delayed perfect measurements - hover experiment...\n');
heights = 5:5:20;
gains = zeros(1, length(heights));
for h = 1:length(heights)
    [states_over_time{h}, observations_over_time{h}, parameters_over_time{h}, T] = adaptive_control_with_delay(heights(h), -0.1, 0, 10, 30, true);
    gains(h) = parameters_over_time{h}(end, 1);
    fprintf('Height %f, gain %f.\n', heights(h), gains(h));
end
figure('Name', 'Relation gain and height in hover', 'NumberTitle', 'off');
set(gcf, 'Color', [1 1 1]);
plot(gains, heights, 'x');
xlabel('Gain K_z (-)');
ylabel('Initial height (m)');

% landing on the edge of oscillation:
fprintf('Adaptive gain control with delayed perfect measurements - landing on the edge of oscillation...\n');
height = 20;
[states_over_time, observations_over_time, parameters_over_time, T] = adaptive_control_with_delay(height, -0.1, 0.02, 10, 30, false);
zind = 5;
stateind = 1;
figure('Name', 'Relation gain and height while landing', 'NumberTitle', 'off');
set(gcf, 'Color', [1 1 1]);
plot(parameters_over_time(:, stateind), states_over_time(:, zind), 'x');
xlabel('Gain K_z (-)');
ylabel('Height (m)');

