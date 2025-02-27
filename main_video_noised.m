function [t,COMs,COMvs,betas,thetas,target_thetas]=main_video_noised(init_state)
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
Model_consts('dt')=0.005;

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
video = VideoWriter('Init81cm.avi'); % You can change the file format if needed
video.FrameRate = 1/dt/2; % Set the frame rate to match 0.05 seconds per frame
open(video); % Open the video file for writing
controller_state=2;
for i = 0:dt:end_time
    [COM_x,COM_y]=getCOM(Model_consts,State_variables);
    [COMv_x,COMv_y]=getCOMv(Model_consts,State_variables,solutions_dd);
    COMs=[COMs,COM_x];
    COMvs=[COMvs,COMv_x];
    % Define the mean and standard deviations
    mu = [0, 0, 0, 0, 0, 0];  % Mean vector
    sigma = [0.01, 0.01, 0.01, 0.005, 0.005, 0.005];  % Standard deviation vector
    
    % Generate a 1x6 matrix with the specified mean and sigma
    noise = normrnd(mu, sigma);
    State_variables=State_variables+noise;% Generate sensor noise.
    switch controller_state
        case 1
            if abs(COM_x)<0.003
                [Controls_u,t_theta,new_Pid_I]=state3control(Model_consts,State_variables,solutions_dd,Pid_Is);
                Pid_Is('S3')=new_Pid_I;
                controller_state=3;
            elseif abs(COM_x)<0.04
                [Controls_u,t_theta,new_Pid_I]=state1control(Model_consts,State_variables,solutions_dd,Pid_Is);
                Pid_Is('S1')=new_Pid_I;
            else 
                [Controls_u,t_theta,new_Pid_I]=state2control(Model_consts,State_variables,solutions_dd,Pid_Is);
                
                Pid_Is('S2')=new_Pid_I;
                controller_state=2;
            end
        case 2
            if abs(COM_x)>0.011
                [Controls_u,t_theta,new_Pid_I]=state2control(Model_consts,State_variables,solutions_dd,Pid_Is);
                
                Pid_Is('S2')=new_Pid_I;
            else 
                [Controls_u,t_theta,new_Pid_I]=state3control(Model_consts,State_variables,solutions_dd,Pid_Is);
                Pid_Is('S3')=new_Pid_I;
                controller_state=3;
            end
        case 3
            if abs(COM_x)<0.04
                [Controls_u,t_theta,new_Pid_I]=state3control(Model_consts,State_variables,solutions_dd,Pid_Is);
                Pid_Is('S3')=new_Pid_I;
                if abs(State_variables(3)-init_State_variables(3))<0.1
                    controller_state=1;
                end
            else 
                [Controls_u,t_theta,new_Pid_I]=state2control(Model_consts,State_variables,solutions_dd,Pid_Is);
                
                Pid_Is('S2')=new_Pid_I;
                controller_state=2;

            end
    end
    
    
    State_variables=State_variables-noise; % Sensor noise removed for physics simulation.
    %Controls_u=[70,100];
    thetas=[thetas,State_variables(1)];
    t_theta;
    target_thetas=[target_thetas,t_theta];
    controller_state_his=[controller_state_his,controller_state];
    betas=[betas,State_variables(3)];
    show_fig(Model_consts,State_variables,i);
    
    text(COM_x, -2*Model_consts('r') - 0.5, sprintf('COMv: %.3f', COMv_x), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    text(COM_x+1.2, -2*Model_consts('r') + 1.1, sprintf('Controller Case: %d', controller_state), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    if controller_state==2
        pbs='0.05x';
    else
        pbs='0.5x';
    end
    text(COM_x+1.2, -2*Model_consts('r') + 1.3, sprintf(['Playback Speed: ',pbs]), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    frame = getframe(gcf);
    writeVideo(video, frame);
    if controller_state==2
        for j = 1:9
                writeVideo(video, frame);
        end
    end
    new_State_variables=update_rk4(Model_consts,State_variables,Controls_u,dt,solutions_dd);
    State_variables=new_State_variables;
end

% Close the video file
close(video);
% Example data (replace with your actual data)
t =0:dt:end_time ;  % Time vector


end