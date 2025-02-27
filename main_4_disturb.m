function [t,COMs,COMvs,betas,thetas,target_thetas,controller_state_his,State_variables_his]=main_4_disturb(Model_consts,end_time,num_kicks,solutions_dd,solutions_dd_disturb,f_tau_1,f_tau_2,f3_tau_2)

State_variables=init(Model_consts); %theta,alpha,beta,dtheta,dalpha,dbeta
init_State_variables=State_variables;
%h = waitbar(0, 'Processing...');
dt=Model_consts('dt');
F=generate_kicks(dt,end_time,num_kicks);
step=0;
target_thetas=[];
thetas=[];
COMs=[];
COMvs=[];
controller_state_his=[];
betas=[];
State_variables_his=[];
controller_state=1;
Pid_Is=containers.Map;
Pid_Is('S1')=0;
Pid_Is('S2')=0;
Pid_Is('S3')=0;
for i = 0:dt:end_time
    step=step+1;
    %waitbar(i/end_time, h, sprintf('Progress: %d%%', round(i/end_time * 100)));
    [COM_x,COM_y]=getCOM(Model_consts,State_variables);
    [COMv_x,COMv_y]=getCOMv(Model_consts,State_variables,solutions_dd);
    COMs=[COMs,COM_x];
    COMvs=[COMvs,COMv_x];
    %disp(controller_state);
    switch controller_state
        case 1
            if abs(COM_x)<0.02
                [Controls_u,t_theta,new_Pid_I]=state1control_fast(Model_consts,State_variables,solutions_dd,Pid_Is,f_tau_2);
                Pid_Is('S1')=new_Pid_I;
            else 
                [Controls_u,t_theta,new_Pid_I]=state2control_fast(Model_consts,State_variables,solutions_dd,Pid_Is,f_tau_1);
                
                Pid_Is('S2')=new_Pid_I;
                controller_state=2;
            end
        case 2
            if abs(COM_x)>0.011
                [Controls_u,t_theta,new_Pid_I]=state2control_fast(Model_consts,State_variables,solutions_dd,Pid_Is,f_tau_1);
                
                Pid_Is('S2')=new_Pid_I;
            else 
                [Controls_u,t_theta,new_Pid_I]=state3control_fast(Model_consts,State_variables,solutions_dd,Pid_Is,f3_tau_2);
                Pid_Is('S3')=new_Pid_I;
                controller_state=3;
            end
        case 3
            if abs(COM_x)<0.042
                [Controls_u,t_theta,new_Pid_I]=state3control_fast(Model_consts,State_variables,solutions_dd,Pid_Is,f3_tau_2);
                Pid_Is('S3')=new_Pid_I;
                if abs(State_variables(3)-init_State_variables(3))<0.1
                    controller_state=1;
                end
            else 
                [Controls_u,t_theta,new_Pid_I]=state2control_fast(Model_consts,State_variables,solutions_dd,Pid_Is,f_tau_1);
                
                Pid_Is('S2')=new_Pid_I;
                controller_state=2;

            end
    end
    
    
     %Controls_u=[70,100];
    thetas=[thetas,State_variables(1)];
    t_theta;
    target_thetas=[target_thetas,t_theta];
    controller_state_his=[controller_state_his,controller_state];
    betas=[betas,State_variables(3)];
    State_variables_his=[State_variables_his;State_variables];
    %show_fig(Model_consts,State_variables,i);
    
    %text(COM_x, -2*Model_consts('r') - 0.5, sprintf('COMv: %.3f', COMv_x), ...
    %    'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    %text(COM_x, -2*Model_consts('r') - 0.7, sprintf('state: %.3f', controller_state), ...
    %    'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    %frame = getframe(gcf);
    %writeVideo(video, frame);
    new_State_variables=update_rk4_disturb(Model_consts,State_variables,Controls_u,dt,solutions_dd_disturb,F(step));
    State_variables=new_State_variables;
end
%close(h);

% Close the video file
%close(video);
% Example data (replace with your actual data)
t =0:dt:end_time ;  % Time vector


end