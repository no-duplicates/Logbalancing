function [t,COMs,COMvs,betas,thetas,target_thetas,controller_state_his,State_variables_his,Model_consts]=main_flatmodelpid(init_state)
% This function is used to test how an ankle pid that is tuned for flat
% ground works in this example. 
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
State_variables_his=[];
[ddot_theta_sol, ddot_alpha_sol, ddot_beta_sol]=solve_dd;
solutions_dd=[ddot_theta_sol, ddot_alpha_sol, ddot_beta_sol];

disp(State_variables);
State_variables=State_variables+init_state
init_State_variables=State_variables;

Pid_Is=containers.Map;
Pid_Is('S1')=0;
Pid_Is('S2')=0;
Pid_Is('S3')=0;
end_time=5;




% Create a VideoWriter object
%video = VideoWriter('state1to3demo.avi'); % You can change the file format if needed
%video.FrameRate = 1/dt/3; % Set the frame rate to match 0.05 seconds per frame
%open(video); % Open the video file for writing
controller_state=1;
for i = 0:dt:end_time
    [COM_x,COM_y]=getCOM(Model_consts,State_variables);
    [COMv_x,COMv_y]=getCOMv(Model_consts,State_variables,solutions_dd);
    COMs=[COMs,COM_x];
    COMvs=[COMvs,COMv_x];
    [Controls_u,t_theta,new_Pid_I]=flat_state1control(Model_consts,State_variables,solutions_dd,Pid_Is);
    Pid_Is('S1')=new_Pid_I;
    
    
     %Controls_u=[70,100];
    thetas=[thetas,State_variables(1)];
    t_theta;
    target_thetas=[target_thetas,t_theta];
    controller_state_his=[controller_state_his,controller_state];
    betas=[betas,State_variables(3)];
    State_variables_his=[State_variables_his;State_variables];
    show_fig(Model_consts,State_variables,i);
    
    text(COM_x, -2*Model_consts('r') - 0.5, sprintf('COMv: %.3f', COMv_x), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    text(COM_x, -2*Model_consts('r') - 0.7, sprintf('state: %.3f', controller_state), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    %frame = getframe(gcf);
    %writeVideo(video, frame);
    new_State_variables=update_rk4(Model_consts,State_variables,Controls_u,dt,solutions_dd);
    State_variables=new_State_variables;
end

% Close the video file
%close(video);
% Example data (replace with your actual data)
t =0:dt:end_time ;  % Time vector


end