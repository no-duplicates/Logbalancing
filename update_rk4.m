function new_State_variables=update_rk4(Model_consts,State_variables,Controls_u,dt,solutions_dd)


    
    
    
    % 计算k1, k2, k3, k4
    k1 = dynamics(Model_consts,State_variables, Controls_u,solutions_dd);
    k2 = dynamics(Model_consts,State_variables + 0.5*dt*k1, Controls_u,solutions_dd);
    k3 = dynamics(Model_consts,State_variables + 0.5*dt*k2, Controls_u,solutions_dd);
    k4 = dynamics(Model_consts,State_variables + dt*k3, Controls_u,solutions_dd);

    % 更新状态
    new_State_variables = State_variables + dt * (k1 + 2*k2 + 2*k3 + k4) / 6;

    

    

end

function dx = dynamics(Model_consts,x, u,solutions_dd)
        values = struct('l0',Model_consts('l0'),'l1',Model_consts('l1') , 'l2', ...
        Model_consts('l2'), 'm1',Model_consts('m1'), 'm2',Model_consts('m2'), ...
        'm0', Model_consts('m0'), 'r', Model_consts('r'), ...
        'tau1', u(1), 'tau2',u(2), ...
        'theta', x(1), 'alpha',x(2) , 'beta', x(3), ...
        'dtheta',x(4),'dalpha',x(5),'dbeta',x(6), ...
        'g', Model_consts('g'));
        ddtheta_val=(double(subs(solutions_dd(1),values)));
        ddalpha_val=(double(subs(solutions_dd(2),values)));
        ddbeta_val=(double(subs(solutions_dd(3),values)));
        dx = [x(4), x(5), x(6), ddtheta_val, ddalpha_val, ddbeta_val];
end