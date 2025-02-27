
% Define the constants in the model.
Model_consts=containers.Map; 
Model_consts('r')=0.1;
Model_consts('l0')=0.1;
Model_consts('l1')=0.8;
Model_consts('l2')=0.7;
Model_consts('m0')=15;
Model_consts('m1')=35;
Model_consts('m2')=20;
Model_consts('g')=9.81;
Model_consts('dt')=0.01;

% Initialize the state variables.
State_variables=init(Model_consts); %theta,alpha,beta,dtheta,dalpha,dbeta

dt=Model_consts('dt');

target_thetas=[];
thetas=[];
COMs=[];
COMvs=[];
controller_state_his=[];
betas=[];
[ddot_theta_sol, ddot_alpha_sol, ddot_beta_sol]=solve_dd;
solutions_dd=[ddot_theta_sol, ddot_alpha_sol, ddot_beta_sol];

disp(State_variables);
State_variables=State_variables+init_state
init_State_variables=State_variables;

Pid_Is=containers.Map;
Pid_Is('S1')=0;
Pid_Is('S2')=0;
Pid_Is('S3')=0;
end_time=8;




% Create a VideoWriter object
video = VideoWriter('Init96cm.avi'); % You can change the file format if needed
video.FrameRate = 1/dt/2; % Set the frame rate to match 0.05 seconds per frame
open(video); % Open the video file for writing

kj=1;
Model_consts=results_noised_3{kj}.Model_consts;
State_variables_his_cur=results_noised_3{kj}.State_variables_his;
controller_state_his_cur=results_noised_3{kj}.controller_state_his;
t=results_noised_3{kj}.t;
for i = 1:length(results_noised_3{kj}.t)
    State_variables=State_variables_his_cur(i,:);
    [COM_x,COM_y]=getCOM(Model_consts,State_variables);
    [COMv_x,COMv_y]=getCOMv(Model_consts,State_variables,solutions_dd);
    COMs=[COMs,COM_x];
    COMvs=[COMvs,COMv_x];
    % Define the mean and standard deviations
    
     
    
    controller_state=controller_state_his_cur(i);
    show_fig(Model_consts,State_variables,t(i));
    
    text(COM_x, -2*Model_consts('r') - 0.5, sprintf('COMv: %.3f', COMv_x), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    text(COM_x+1.2, -2*Model_consts('r') + 1.1, sprintf('Controller Case: %d', controller_state), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    %if controller_state==2
    %    pbs='0.05x';
    %else
        pbs='0.5x';
    %end
    text(COM_x+1.2, -2*Model_consts('r') + 1.3, sprintf(['Playback Speed: ',pbs]), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    frame = getframe(gcf);
    writeVideo(video, frame);
    %if controller_state==2
    %    for j = 1:9
    %            writeVideo(video, frame);
    %    end
    %end

end

% Close the video file
close(video);
% Example data (replace with your actual data)
t =0:dt:end_time ;  % Time vector


