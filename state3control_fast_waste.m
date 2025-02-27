function [Controls_u,target_theta,new_Pid_I]=state3control_fast_waste(Model_consts,State_variables,solutions_dd,Pid_Is,f3_tau_1,f_tau_2)
    Controls_u=[0,0];
    % Define symbolic variables
    %syms theta alpha beta dtheta dalpha dbeta ddot_theta ddot_alpha ddot_beta
    %syms m0 m1 m2 l1 l2 r g tau1 tau2 l0
    
   

    [COM_x,COM_y]=getCOM(Model_consts,State_variables);
    ka_x=Model_consts('g')/(COM_y-Model_consts('r'));


    [COMv_x,COMv_y]=getCOMv(Model_consts,State_variables,solutions_dd);
    
    


    %Start lqr
    % Define system matrices
    A = [0 1; 
         0 0];
    B = [0; 
         1];
     
    % Define cost function matrices
    Q = [20 0;
         0 10];% Penalize both x and v equally
    R = 1;       % Control effort penalty
    
    % Solve for the LQR gain
    K = lqr(A, B, Q, R);
    
    target_a=-K*[COM_x;COMv_x];


    target_theta=asin((target_a/ka_x-COM_x)/Model_consts('r'));
   
    if target_theta>0.8
        target_theta=0.8;
    end
    if target_theta<-0.5
        target_theta=-0.5;
    end
    %Start for pd controls.

    Kp = 500;   % Proportional gain
    Ki = 100;
    Kd =50; % Derivative gain
    error = target_theta-State_variables(1);
    
    % Proportional term
    P = Kp * error;
    
    new_Pid_I=Pid_Is('S3')+error*Model_consts('dt');
    I = Ki*new_Pid_I;
    % Derivative term
    derivative = State_variables(4);
    D =- Kd * derivative;
    
    % PID control law
    Controls_u(1) =-( P +I+ D); % the positive control will bring negative dtheta
    
        

    Controls_u(1)=Controls_u(1)+Model_consts('g')*(Model_consts('m0')+Model_consts('m1')+Model_consts('m2'))*(Model_consts('l0')+Model_consts('r')*(-State_variables(1)))*cos(State_variables(1));



    params = [Model_consts('l0'), Model_consts('l1'), Model_consts('l2'), ...
              Model_consts('m0'), Model_consts('m1'), Model_consts('m2'), ...
              Model_consts('r'), Model_consts('g')];

    % Extract state variables
    state = State_variables;

    % Extract control inputs
    
    % Correct calling convention
Controls_u(2)= f_tau_2(state(1), state(2), state(3), state(4), state(5), state(6), ...
                              Controls_u(1),  ...
                              params(1), params(2), params(3), params(4), params(5), params(6), params(7), params(8));

    %Start lqr for S3
    % Define system matrices
    A = [0 1; 
         0 0];
    B = [0; 
         1];
     
    % Define cost function matrices
    Q = [1 0;
         0 1];% Penalize both x and v equally
    R = 1;       % Control effort penalty
    
    % Solve for the LQR gain
    K = lqr(A, B, Q, R);
    origin_state=init(Model_consts);
    error_state=State_variables-origin_state;
    u_hip=-K*[error_state(3);error_state(6)];


    
    Controls_u(2)=Controls_u(2);%+u_hip;




    target_a=-K*[COM_x;COMv_x]-u_hip/(Model_consts('l1')+Model_consts('l2'))/(Model_consts('m1')+Model_consts('m2'));


    target_theta=asin((target_a/ka_x-COM_x)/Model_consts('r'));
   
    if target_theta>0.8
        target_theta=0.8;
    end
    if target_theta<-0.5
        target_theta=-0.5;
    end
    %Start for pd controls.

    Kp = 500;   % Proportional gain
    Ki = 100;
    Kd =50; % Derivative gain
    error = target_theta-State_variables(1);
    
    % Proportional term
    P = Kp * error;
    
    new_Pid_I=Pid_Is('S3')+error*Model_consts('dt');
    I = Ki*new_Pid_I;
    % Derivative term
    derivative = State_variables(4);
    D =- Kd * derivative;
    
    % PID control law
    Controls_u(1) =-( P +I+ D); % the positive control will bring negative dtheta
    
        

    Controls_u(1)=Controls_u(1)+Model_consts('g')*(Model_consts('m0')+Model_consts('m1')+Model_consts('m2'))*(Model_consts('l0')+Model_consts('r')*(-State_variables(1)))*cos(State_variables(1));

    
    
    


    
end