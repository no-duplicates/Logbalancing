%Change these parameters
%#########################
num_rounds=1000;
num_kicks=3;
kick_force_mean=80;
kick_force_sd=50;
%#########################


num_states=num_rounds;
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


[ddot_theta_sol, ddot_alpha_sol, ddot_beta_sol]=Zsolve_dd();
solutions_dd=[ddot_theta_sol, ddot_alpha_sol, ddot_beta_sol];
[ddot_theta_sol, ddot_alpha_sol, ddot_beta_sol]=Zsolve_dd_disturb();
solutions_dd_disturb=[ddot_theta_sol, ddot_alpha_sol, ddot_beta_sol];
%disp(State_variables);
%State_variables=State_variables+init_state;
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


% Define the matrices of initial states you want to test
init_states = [
    %-0.2, -0.12, -0.11, 0, 0, 0; %COM=0.096
    %-0.2, -0.11, -0.06, 0, 0, 0; %COM=0.081
    %-0.2, -0.08, -0.08, 0, 0, 0; %COM=0.066
    %-0.2, -0.06, -0.06, 0, 0, 0; %COM=0.050
    %-0.2, -0.04, -0.04, 0, 0, 0; %COM=0.034
    -0.15, -0.025, -0.025, 0, 0, 0; %COM=0.021
    -0.05, -0.01, -0.00, 0, 0, 0; %COM=0.006
]; % Example initial states, adjust as needed

% Preallocate cell array to store the results
%num_states = size(init_states, 1);

results = cell(num_states, 1);
success_count=0;
fail_count=0;
hh = waitbar(0, 'Processing...');
% Loop over each initial state matrix
for i = 1:num_states
    %init_state = init_states(i, :); % Extract the current matrix
    waitbar(i/num_states, hh, sprintf('Progress: %d%%', round(i/num_states * 100)));
    
    % Call the function with the current initial state
    [t,COMs,COMvs,betas,thetas,target_thetas,controller_state_his,State_variables_his]=main_4_disturb(Model_consts,end_time,num_kicks,solutions_dd,solutions_dd_disturb,f_tau_1,f_tau_2,f3_tau_2);

    if COMs(end)<=0.04
        success_count=success_count+1;
    else
        fail_count=fail_count+1;
    end

    
    % Store the results in the cell array
    %results{i} = struct('t', t, 'COMs', COMs, 'COMvs', COMvs, 'betas', betas, 'thetas', thetas, 'target_thetas', target_thetas,'controller_state_his',controller_state_his,'State_variables_his',State_variables_his,'Model_consts',Model_consts);
end
total_count = success_count + fail_count;
fprintf('Total attempts: %d (Successes: %d, Failures: %d)\n', total_count, success_count, fail_count);
% Optionally, save the results to a MAT-file
%save('results.mat', 'results');
