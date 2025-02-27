% Define the matrices of initial states you want to test
init_states = [
    %-0.15, -0.025, -0.025, 0, 0, 0; %COM=0.021
    -0.05, -0.01, -0.00, 0, 0, 0; %COM=0.006
]; % Example initial states, adjust as needed

% Preallocate cell array to store the results
num_states = size(init_states, 1);
flat_results_3 = cell(num_states, 1);

% Loop over each initial state matrix
for i = 1:num_states
    init_state = init_states(i, :); % Extract the current matrix
    
    % Call the function with the current initial state
    [t,COMs,COMvs,betas,thetas,target_thetas,controller_state_his,State_variables_his,Model_consts]=main_flatmodelpid(init_state);


    
    % Store the results in the cell array
    flat_results_3{i} = struct('t', t, 'COMs', COMs, 'COMvs', COMvs, 'betas', betas, 'thetas', thetas, 'target_thetas', target_thetas,'controller_state_his',controller_state_his,'State_variables_his',State_variables_his,'Model_consts',Model_consts);
end

% Optionally, save the results to a MAT-file
save('flat_results_3.mat', 'flat_results_3');
