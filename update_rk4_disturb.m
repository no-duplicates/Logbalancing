function new_State_variables=update_rk4_disturb(Model_consts,State_variables,Controls_u,dt,solutions_dd_disturb,F_external)


    
    
    
    % 计算k1, k2, k3, k4
    k1 = double(dynamics(Model_consts,State_variables, Controls_u,solutions_dd_disturb,F_external));
    k2 = double(dynamics(Model_consts,State_variables + 0.5*dt*k1, Controls_u,solutions_dd_disturb,F_external));
    k3 = double(dynamics(Model_consts,State_variables + 0.5*dt*k2, Controls_u,solutions_dd_disturb,F_external));
    k4 = double(dynamics(Model_consts,State_variables + dt*k3, Controls_u,solutions_dd_disturb,F_external));

    % 更新状态
    new_State_variables = State_variables + dt * (k1 + 2*k2 + 2*k3 + k4) / 6;

    

    

end
function dx = dynamics(Model_consts, x, u,solutions_dd_disturb,F_external)
    % Extract model constants
    params = [Model_consts('l0'), Model_consts('l1'), Model_consts('l2'), ...
              Model_consts('m0'), Model_consts('m1'), Model_consts('m2'), ...
              Model_consts('r'), Model_consts('g')];

    % Extract state variables
    state = [x(1), x(2), x(3), x(4), x(5), x(6)];

    % Extract control inputs
    control = [u(1), u(2)];
    % Correct calling convention
ddtheta_val = solutions_dd_disturb{1}(state(1), state(2), state(3), state(4), state(5), state(6), ...
                              control(1), control(2), ...
                              params(1), params(2), params(3), params(4), params(5), params(6), params(7), params(8),F_external);

ddalpha_val = solutions_dd_disturb{2}(state(1), state(2), state(3), state(4), state(5), state(6), ...
                              control(1), control(2), ...
                              params(1), params(2), params(3), params(4), params(5), params(6), params(7), params(8),F_external);

ddbeta_val  = solutions_dd_disturb{3}(state(1), state(2), state(3), state(4), state(5), state(6), ...
                              control(1), control(2), ...
                              params(1), params(2), params(3), params(4), params(5), params(6), params(7), params(8),F_external);


    % Return state derivatives
    dx = [x(4), x(5), x(6), ddtheta_val, ddalpha_val, ddbeta_val];
end

function dx = dynamics_waste(Model_consts,x, u,solutions_dd)
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