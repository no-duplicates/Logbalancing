function [Controls_u,target_theta,new_Pid_I]=state3control_fast(Model_consts,State_variables,solutions_dd,Pid_Is,f3_tau_2)
    Controls_u=[0,0];
  
    




    [COM_x,COM_y]=getCOM(Model_consts,State_variables);
    ka_x=Model_consts('g')/(COM_y-Model_consts('r'));


    [COMv_x,COMv_y]=getCOMv(Model_consts,State_variables,solutions_dd);
    
    
    %Start lqr for S3
    % Define system matrices
    A = [0 1; 
         0 0];
    B = [0; 
         1];
     
    % Define cost function matrices
    Q = [480 0;
         0 480];% Penalize both x and v equally
    R = 1;       % Control effort penalty
    
    % Solve for the LQR gain
    K = lqr(A, B, Q, R);
    origin_state=init(Model_consts);
    error_state=State_variables-origin_state;
    u_hip=-K*[error_state(3)-error_state(2);error_state(6)-error_state(5)];

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

    ddtheta1 = u_hip * (-(Model_consts('l1') + Model_consts('l2'))) / ...
           (Model_consts('l1')^2 * Model_consts('l2') * Model_consts('m1'));

    ddtheta2 = (Model_consts('l1') * ddtheta1 * (-(Model_consts('l1') * Model_consts('m1') ...
           + Model_consts('l1') * Model_consts('m2') + Model_consts('l2') * Model_consts('m2')))) / ...
           (Model_consts('l2') * Model_consts('m2') * (Model_consts('l1') + Model_consts('l2')));
    ddCOM_x = ddtheta2 * ((Model_consts('l2')^2 * Model_consts('m1') * Model_consts('m2'))) / ...
          ((Model_consts('l1') * Model_consts('m1') + Model_consts('l1') * Model_consts('m2') ...
          + Model_consts('l2') * Model_consts('m2')) * (Model_consts('m0') + Model_consts('m1') + Model_consts('m2')));
    
    if COM_x>-10
        target_a=target_a-ddCOM_x*0.2;
    end

    target_theta=asin((target_a/ka_x-COM_x)/Model_consts('r'));
    

    

    if target_theta>0.8
        target_theta=0.8;
    end
    if target_theta<-0.5
        target_theta=-0.5;
    end
    %Start for pd controls.

    

    Kp = 500;   % Proportional gain
    Ki = 2000;
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
    
        

    Controls_u(1)=Controls_u(1)+Model_consts('g')*(Model_consts('m0')+Model_consts('m1')+Model_consts('m2'))*(Model_consts('l0')-State_variables(1)*Model_consts('r'))*cos(State_variables(1));
    % Solve the equations
    
    
    %test=[(double(subs(ddot_beta_sol,values))),(double(subs(ddot_alpha_sol,values))),(double(subs(tau_2_sol,values)))]
    params = [Model_consts('l0'), Model_consts('l1'), Model_consts('l2'), ...
              Model_consts('m0'), Model_consts('m1'), Model_consts('m2'), ...
              Model_consts('r'), Model_consts('g')];
    
    % Extract state variables
    state = State_variables;
    Controls_u(2)= f3_tau_2(state(1), state(2), state(3), state(4), state(5), state(6), ...
                              Controls_u(1),  ...
                              params(1), params(2), params(3), params(4), params(5), params(6), params(7), params(8));

    


    
    Controls_u(2)=Controls_u(2)+u_hip;
    
end