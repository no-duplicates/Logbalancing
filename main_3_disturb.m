function [t,COMs,COMvs,betas,thetas,target_thetas,controller_state_his,State_variables_his,Model_consts]=main_3_disturb(init_state)
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
[ddot_theta_sol, ddot_alpha_sol, ddot_beta_sol]=Zsolve_dd();
solutions_dd=[ddot_theta_sol, ddot_alpha_sol, ddot_beta_sol];
[ddot_theta_sol, ddot_alpha_sol, ddot_beta_sol]=Zsolve_dd_disturb();
solutions_dd_disturb=[ddot_theta_sol, ddot_alpha_sol, ddot_beta_sol];
disp(State_variables);
State_variables=State_variables+init_state
init_State_variables=State_variables;

Pid_Is=containers.Map;
Pid_Is('S1')=0;
Pid_Is('S2')=0;
Pid_Is('S3')=0;
end_time=10;




% Create a VideoWriter object
%video = VideoWriter('state1to3demo.avi'); % You can change the file format if needed
%video.FrameRate = 1/dt/3; % Set the frame rate to match 0.05 seconds per frame
%open(video); % Open the video file for writing
controller_state=3;
h = waitbar(0, 'Processing...');





%start solving equations
% Define symbolic variables using sym
    theta = sym('theta');
    alpha = sym('alpha');
    beta = sym('beta');
    dtheta = sym('dtheta');
    dalpha = sym('dalpha');
    dbeta = sym('dbeta');
    ddot_theta = sym('ddot_theta');
    ddot_alpha = sym('ddot_alpha');
    ddot_beta = sym('ddot_beta');
    
    % Define additional symbolic variables
    m0 = sym('m0');
    m1 = sym('m1');
    m2 = sym('m2');
    l0 = sym('l0');
    l1 = sym('l1');
    l2 = sym('l2');
    r = sym('r');
    g = sym('g');
    tau1 = sym('tau1');
    tau2 = sym('tau2');
    F_external=sym("F_external");
    init_stateV=init(Model_consts);
    
        f_ddtheta = matlabFunction(solutions_dd(1), 'Vars', {theta, alpha, beta, dtheta, dalpha, dbeta, tau1, tau2, l0, l1, l2, m0, m1, m2, r, g});
        f_ddalpha = matlabFunction(solutions_dd(2), 'Vars', {theta, alpha, beta, dtheta, dalpha, dbeta, tau1, tau2, l0, l1, l2, m0, m1, m2, r, g});
        f_ddbeta  = matlabFunction(solutions_dd(3), 'Vars', {theta, alpha, beta, dtheta, dalpha, dbeta, tau1, tau2, l0, l1, l2, m0, m1, m2, r, g});
    
    solutions_dd = {f_ddtheta, f_ddalpha, f_ddbeta}; % 
    

        
        f_ddtheta = matlabFunction(solutions_dd_disturb(1), 'Vars', {theta, alpha, beta, dtheta, dalpha, dbeta, tau1, tau2, l0, l1, l2, m0, m1, m2, r, g,F_external});
        f_ddalpha = matlabFunction(solutions_dd_disturb(2), 'Vars', {theta, alpha, beta, dtheta, dalpha, dbeta, tau1, tau2, l0, l1, l2, m0, m1, m2, r, g,F_external});
        f_ddbeta  = matlabFunction(solutions_dd_disturb(3), 'Vars', {theta, alpha, beta, dtheta, dalpha, dbeta, tau1, tau2, l0, l1, l2, m0, m1, m2, r, g,F_external});
    
    solutions_dd_disturb = {f_ddtheta, f_ddalpha, f_ddbeta}; % 



    % Define positions and their second derivatives
    
    x0 = -sin(theta)*r + theta*cos(theta)*r-cos(theta)*l0;
    y0 = cos(theta)*r + theta*sin(theta)*r-sin(theta)*l0;
    x1 = x0 - sin(alpha)*l1;
    y1 = y0 + cos(alpha)*l1;
    x2 = x1 - sin(beta)*l2;
    y2 = y1 + cos(beta)*l2;
    
    % Calculate second derivatives
    
    
    
    %ddot_x0 = -cos(theta)*ddot_theta*r - 2*sin(theta)*(diff(theta)^2)*r + theta*(-sin(theta)*ddot_theta*r + cos(theta)*ddot_theta*r);
    %ddot_y0 = -sin(theta)*ddot_theta*r + 2*cos(theta)*(diff(theta)^2)*r + theta*(cos(theta)*ddot_theta*r - sin(theta)*ddot_theta*r);
    %ddot_x0 = -cos(theta)*ddot_theta*r - (sin(theta)*diff(theta)^2 + cos(theta)*ddot_theta)*r + (ddot_theta*cos(theta) - theta*ddot_theta*sin(theta) - theta*diff(theta)^2*sin(theta))*r;
    %ddot_y0 = -sin(theta)*ddot_theta*r - (cos(theta)*diff(theta)^2 + sin(theta)*ddot_theta)*r + (ddot_theta*sin(theta) + theta*ddot_theta*cos(theta) - cos(theta)*diff(theta)^2)*r - cos(theta)*ddot_theta*l0;
    dx_0=diff(x0,theta);
    ddot_x0=diff(dx_0,theta)*dtheta^2+dx_0*ddot_theta;
    dy_0=diff(y0,theta);
    ddot_y0=diff(dy_0,theta)*dtheta^2+dy_0*ddot_theta;
    
    dx_1=diff(x1,alpha);
    ddot_x1=diff(dx_1,alpha)*dalpha^2+dx_1*ddot_alpha+ddot_x0;
    dy_1=diff(y1,alpha);
    ddot_y1=diff(dy_1,alpha)*dalpha^2+dy_1*ddot_alpha+ddot_y0;
    
    dx_2=diff(x2,beta);
    ddot_x2=diff(dx_2,beta)*dbeta^2+dx_2*ddot_beta+ddot_x1;
    dy_2=diff(y2,beta);
    ddot_y2=diff(dy_2,beta)*dbeta^2+dy_2*ddot_beta+ddot_y1;
    %ddot_x1 = ddot_x0 - ddot_alpha*l1;
    %ddot_y1 = ddot_y0 + ddot_alpha*l1;
    %ddot_x2 = ddot_x1 - ddot_beta*l2;
    %ddot_y2 = ddot_y1 + ddot_beta*l2;
    
    % Define the equations
    eq1 = -ddot_x2*(cos(beta)*l2 + cos(alpha)*l1)*m2 - (ddot_y2 + g)*(sin(beta)*l2 + sin(alpha)*l1)*m2 - ddot_x1*cos(alpha)*l1*m1 - (ddot_y1 + g)*sin(alpha)*l1*m1 == tau1;
    eq2 = -ddot_x2*cos(beta)*l2*m2 - (ddot_y2 + g)*sin(beta)*l2*m2 == tau2;
    
    % Define the normal forces
    Nx = sin(theta)*tau1 / (-l0+r*theta);
    Ny = cos(theta)*tau1 / (l0-r*theta);
    
    % Define the normal force equation
    eq3 = Nx*(ddot_x0*m0 + ddot_x1*m1 + ddot_x2*m2) + Ny*((ddot_y0 + g)*m0 + (ddot_y1 + g)*m1 + (ddot_y2 + g)*m2) == Nx^2 + Ny^2;
    
    eq4= ddot_beta-ddot_alpha==6.32*(dalpha-dbeta)+10*(alpha-init_stateV(2)-beta+init_stateV(3));
    
    eq5= ddot_alpha==ddot_theta;
    eq6= ddot_beta-ddot_alpha==10*(dalpha-dbeta);
[ddot_theta_sol, ddot_alpha_sol, ddot_beta_sol,tau_2_sol] = solve([eq1, eq2, eq3,eq4], [ddot_theta, ddot_alpha, ddot_beta,tau2]);
[ddot_theta_sol, ddot_alpha_sol, ddot_beta_sol,tau_1_sol] = solve([eq1, eq2, eq3,eq5], [ddot_theta, ddot_alpha, ddot_beta,tau1]);
[ddot_theta_sol, ddot_alpha_sol, ddot_beta_sol,tau3_2_sol] = solve([eq1, eq2, eq3,eq6], [ddot_theta, ddot_alpha, ddot_beta,tau2]);

tau_1_sol=tau_1_sol(2);
 f_tau_2  = matlabFunction(tau_2_sol, 'Vars', {theta, alpha, beta, dtheta, dalpha, dbeta, tau1,  l0, l1, l2, m0, m1, m2, r, g});
 f_tau_1  = matlabFunction(tau_1_sol, 'Vars', {theta, alpha, beta, dtheta, dalpha, dbeta, tau2,  l0, l1, l2, m0, m1, m2, r, g});
 f3_tau_2  = matlabFunction(tau3_2_sol, 'Vars', {theta, alpha, beta, dtheta, dalpha, dbeta, tau1,  l0, l1, l2, m0, m1, m2, r, g});
   num_kicks=3;
F=generate_kicks(dt,end_time,num_kicks);
step=0;
for i = 0:dt:end_time
    step=step+1;
    waitbar(i/end_time, h, sprintf('Progress: %d%%', round(i/end_time * 100)));
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
close(h);

% Close the video file
%close(video);
% Example data (replace with your actual data)
t =0:dt:end_time ;  % Time vector


end