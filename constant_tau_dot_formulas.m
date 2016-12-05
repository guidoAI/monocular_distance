function [z, vz, az, t] = constant_tau_dot_formulas(height, velocity, tau_dot_ref, T, graphics)
% function [z, vz, az, t] = constant_tau_dot_formulas(height, velocity, tau_dot_ref, T, graphics)
%
% Uses the formulas from:
% (2012), Izzo, D., and de Croon, G.C.H.E., "Landing with time-to-contact and ventral optic flow estimates", 
% in Journal of Guidance, Control, and Dynamics, Volume 35, Issue 4, pages 1362-1367
% to calculate the height, velocity, and acceleration along a straight
% vertical landing.
%
% Input parameters:
% - height: initial height of the landing (positive)
% - velocity: initial vertical velocity of the landing (typically negative)
% - tau_dot_ref: what the time derivative of time-to-contact should be
% (typically negative)
% T: number of time steps to plot. If vz and tau_dot_ref are negative, this
% number is replaced by the time step at which z = 0.

if(~exist('graphics', 'var') || isempty(graphics))
    graphics = true;
end

% adapt to convention in the article:
c2 = -(tau_dot_ref);
z0 = height;
vz0 = velocity;
time_step = 0.1;

if(c2 == 0)
    % c2 = 0 requires different formulas - z will never actually reach 0:
    t = 1:time_step:T;
    % closed form solution for c2 = 0 (constant tau landing):
    z = z0 * exp((vz0 / z0) * t);
    vz = vz0 * exp((vz0 / z0) * t);
    az = vz0^2/z0 * exp((vz0 / z0) * t);
else
    % calculate time step at which z = 0:
    if(vz0 < 0 && c2> 0)
        T = -(z0/vz0)/c2;
    end
    t = 1:time_step:T;
    % closed form solutions to z, vz, az:
    z = z0 * (c2 * (vz0/z0) * t + 1).^(1/c2);
    vz = vz0 * (c2 * (vz0/z0) * t + 1).^(1/c2 - 1);
    az = (1-c2) * (vz0^2/z0) * (c2 * (vz0/z0) * t + 1).^(1/c2 - 2);
end

if(graphics)
    % plot the relevant variables:
    figure('Name','Constant tau-dot formula','NumberTitle','off');
    set(gcf, 'Color', [1 1 1]);
    subplot(3,1,1);
    plot(t, z);
    ylabel('z (m)');
    subplot(3,1,2);
    plot(t, vz);
    ylabel('vz (m/s)');
    subplot(3,1,3);
    plot(t, az);
    ylabel('az (m/s^2)');
    xlabel('time (s)');
end
% figure();
% plot(t, vz ./ z);
% title('divergence');
