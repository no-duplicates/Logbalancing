function [Controls_u,target_theta,new_Pid_I]=state2control_fast(Model_consts,State_variables,solutions_dd,Pid_Is,f_tau_1)
    Controls_u=[0,0];
    % Define symbolic variables
    %syms theta alpha beta dtheta dalpha dbeta ddot_theta ddot_alpha ddot_beta
    %syms m0 m1 m2 l1 l2 r g tau1 tau2 l0
    
    % Define symbolic variables using sym
   
    %ddot_x1 = ddot_x0 - ddot_alpha*l1;
    %ddot_y1 = ddot_y0 + ddot_alpha*l1;
    %ddot_x2 = ddot_x1 - ddot_beta*l2;
    %ddot_y2 = ddot_y1 + ddot_beta*l2;
    




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
    Q = [200000000 0;
         0 5];% Penalize both x and v equally
    R = 1;       % Control effort penalty
    
    % Solve for the LQR gain
    K = lqr(A, B, Q, R);
    
    target_a=-K*[COM_x+0.25*COMv_x;State_variables(6)];
    

    % PID control law
    Controls_u(2) =target_a-Model_consts('m2')*Model_consts('g')*Model_consts('l2')*sin(State_variables(3)); % the positive control will bring negative dtheta
    
    
    target_theta=-asin(COM_x/Model_consts('r')*1.1);

    

    if target_theta>0.8
        target_theta=0.8;
    end
    if target_theta<-0.5
        target_theta=-0.5;
    end


    Kp = 500;   % Proportional gain
    Ki = 200;
    Kd =50; % Derivative gain
    error = target_theta-State_variables(1);
    
    % Proportional term
    P = Kp * error;
    
    new_Pid_I=Pid_Is('S2')+error*Model_consts('dt');
    I = Ki*new_Pid_I;
    % Derivative term
    derivative = State_variables(4);
    D =- Kd * derivative;
    
    % PID control law
    Controls_u(1) =-( P +I+ D); % the positive control will bring negative dtheta
    
    
    
    % Solve the equations
    %[ddot_theta_sol, ddot_alpha_sol, ddot_beta_sol,tau_1_sol] = solve([eq1, eq2, eq3,eq4], [ddot_theta, ddot_alpha, ddot_beta,tau1]);

    
    %test=[(double(subs(ddot_beta_sol,values))),(double(subs(ddot_alpha_sol,values))),(double(subs(tau_1_sol,values)))];
    %Controls_u(2)=(double(subs(tau_2_sol,values)));
        params = [Model_consts('l0'), Model_consts('l1'), Model_consts('l2'), ...
              Model_consts('m0'), Model_consts('m1'), Model_consts('m2'), ...
              Model_consts('r'), Model_consts('g')];
    
    % Extract state variables
    state = State_variables;
    compensator= f_tau_1(state(1), state(2), state(3), state(4), state(5), state(6), ...
                              Controls_u(2),  ...
                              params(1), params(2), params(3), params(4), params(5), params(6), params(7), params(8));

    Controls_u(1)=Controls_u(1)+compensator;
        
    
    %Controls_u(1)=Controls_u(1)+Model_consts('g')*(Model_consts('m0')+Model_consts('m1')+Model_consts('m2'))*Model_consts('l0')*cos(State_variables(1));
    
end