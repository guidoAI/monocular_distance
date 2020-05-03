function [rise_time, percentage_overshoot, mean_absolute_error] = get_performance_characteristics(signal, reference, times)
% function [rise_time, percentage_overshoot, mean_absolute_error] = get_performance_characteristics(signal, reference, times)
%
% Get rise time, percentage overshoot, and mean absolute error. 

T = length(signal);

% rise time is time till first crossing with the reference:
for t = 1:T-1
    if((signal(t) <= reference && signal(t+1) >= reference) || (signal(t) >= reference && signal(t+1) <= reference) || signal(t) == reference)
        % linear interpolation:
        if(signal(t) ~= signal(t+1))
            alpha = (reference - signal(t+1)) / (signal(t) - signal(t+1));
            rise_time = alpha * times(t) + (1-alpha) * times(t+1);
        else
            % constant value, in principle no crossing
            rise_time = times(t);
        end
        break;
    end
end
if(~exist('rise_time', 'var'))
    % signal does not reach the reference, give a very large value:
    rise_time = 1000;
end

% percentage overshoot absolute value of the first peak divided by the absolute reference value:
try
    if(length(signal) >= 3 && rise_time ~= 1000)
        [pks,locs] = findpeaks(-signal);
        if(isempty(pks))
            pks = [max(-signal)];
        end
        ratio = abs(pks(1)) / abs(reference);
        percentage_overshoot = 100 * (abs(ratio - 1));
        % mean absolute error:
        mean_absolute_error = mean(abs(signal - reference));
    else
        rise_time = 1000;
        percentage_overshoot = 1000;
        mean_absolute_error = 100000;
    end
catch e
    warning('An error occurred in get_performance_characteristics, while searching for peaks.')
end

