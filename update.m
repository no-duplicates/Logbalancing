function new_State_variables=update(Model_consts,State_variables,Controls_u,dt,solutions_dd)


    new_State_variables=State_variables;
    
    %Update position
    new_State_variables(1)=State_variables(1)+State_variables(4)*dt;
    new_State_variables(2)=State_variables(2)+State_variables(5)*dt;
    new_State_variables(3)=State_variables(3)+State_variables(6)*dt;


    %Update velocity.
    values = struct('l0',Model_consts('l0'),'l1',Model_consts('l1') , 'l2', ...
        Model_consts('l2'), 'm1',Model_consts('m1'), 'm2',Model_consts('m2'), ...
        'm0', Model_consts('m0'), 'r', Model_consts('r'), ...
        'tau1', Controls_u(1), 'tau2',Controls_u(2), ...
        'theta', State_variables(1), 'alpha',State_variables(2) , 'beta', State_variables(3), ...
        'dtheta',State_variables(4),'dalpha',State_variables(5),'dbeta',State_variables(6), ...
        'g', Model_consts('g'));
    
    
    new_State_variables(4)=State_variables(4)+(double(subs(solutions_dd(1),values)))*dt;
    new_State_variables(5)=State_variables(5)+(double(subs(solutions_dd(2),values)))*dt;
    new_State_variables(6)=State_variables(6)+(double(subs(solutions_dd(3),values)))*dt;


end