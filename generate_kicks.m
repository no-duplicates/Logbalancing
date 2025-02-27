function F=generate_kicks(dt,end_time,num_kicks,F_mean,F_sd)
% Define time array from 0 to 10 seconds with 0.01s step
t=0:dt:end_time;
F = zeros(size(t)); % Initialize force array with zeros



% Generate random kick times
kick_times = sort(randperm(floor(length(t)*4/5), num_kicks)); % Random kick time indices

% Define possible force magnitudes and durations
force_magnitudes = [100,100 ,  100]; % Possible force values in Newtons
time_durations = [0.1, 0.1]; % Possible contact durations in seconds

% Loop through each kick
for i = 1:num_kicks
    kick_idx = kick_times(i); % Get kick time index
    kick_force = normrnd(F_mean,F_sd);%force_magnitudes(randi(length(force_magnitudes))); % Random force
    kick_duration = time_durations(randi(length(time_durations))); % Random duration
    
    % Convert duration to indices
    kick_end_idx = min(kick_idx + round(kick_duration / 0.01), length(t)); 
    
    % Apply force during kick duration
    F(kick_idx:kick_end_idx) = kick_force;
end


end