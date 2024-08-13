function []=main()
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

% Initialize the state variables.
State_variables=init(Model_consts); %theta,alpha,beta,dtheta,dalpha,dbeta

dt=0.02;

target_thetas=[];
thetas=[];
COMs=[];
COMvs=[];
controller_state_his=[];
[ddot_theta_sol, ddot_alpha_sol, ddot_beta_sol]=solve_dd;
solutions_dd=[ddot_theta_sol, ddot_alpha_sol, ddot_beta_sol];

disp(State_variables);
State_variables=State_variables+[-0.06,-0.06,-0.06,0,0,0];


end_time=3;




% Create a VideoWriter object
video = VideoWriter('state2demo.avi'); % You can change the file format if needed
video.FrameRate = 5; % Set the frame rate to match 0.05 seconds per frame
open(video); % Open the video file for writing
controller_state=1;
for i = 0:dt:end_time
    [COM_x,COM_y]=getCOM(Model_consts,State_variables);
    [COMv_x,COMv_y]=getCOMv(Model_consts,State_variables,solutions_dd);
    COMs=[COMs,COM_x];
    COMvs=[COMvs,COMv_x];
    switch controller_state
        case 1
            if abs(COM_x)<0.04
                [Controls_u,t_theta]=state1control(Model_consts,State_variables,solutions_dd);
            else 
                [Controls_u,t_theta]=state2control(Model_consts,State_variables,solutions_dd);
                controller_state=2;
            end
        case 2
            if abs(COM_x)>0.011
                [Controls_u,t_theta]=state2control(Model_consts,State_variables,solutions_dd);
            else 
                [Controls_u,t_theta]=state1control(Model_consts,State_variables,solutions_dd);
                controller_state=1;
            end
    end
    

    %Controls_u=[70,100];
    thetas=[thetas,State_variables(1)];
    t_theta;
    target_thetas=[target_thetas,t_theta];
    controller_state_his=[controller_state_his,controller_state];
    show_fig(Model_consts,State_variables,i);
    text(COM_x, -2*Model_consts('r') - 0.5, sprintf('State: %.3f', controller_state), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    frame = getframe(gcf);
    writeVideo(video, frame);
    new_State_variables=update_rk4(Model_consts,State_variables,Controls_u,dt,solutions_dd);
    State_variables=new_State_variables;
end

% Close the video file
close(video);
% Example data (replace with your actual data)
t =0:dt:end_time ;  % Time vector

% Create a new figure
figure;

% Plot thetas
plot(t, thetas, 'b-', 'LineWidth', 2); % Blue solid line
hold on;  % Keep the current plot
plot(t, target_thetas, 'r--', 'LineWidth', 2); % Red dashed line

% Add grid
grid on;

% Add title and labels
title('Thetas vs. Target Thetas', 'FontSize', 14);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Theta Value', 'FontSize', 12);

% Add legend
legend({'Thetas', 'Target Thetas'}, 'Location', 'Best', 'FontSize', 12);

% Adjust axes for better visualization
axis tight;
figure;
plot(t, COMs, 'g-', 'LineWidth', 2);

% Add title and axis labels
title('Center of Mass over Time', 'FontSize', 14);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Center of Mass (COM)', 'FontSize', 12);

% Add a grid for better readability
grid on;

% Optionally, set axis limits if needed
xlim([min(t) max(t)]);
ylim([min(COMs) max(COMs)]);

% Add a legend if necessary
legend('COM', 'Location', 'best');

% Adjust the font size of tick labels
set(gca, 'FontSize', 12);

% Optionally, save the figure as an image
% saveas(gcf, 'COM_plot.png');

axis tight;
figure;
plot(t, COMvs, 'b-', 'LineWidth', 2);

% Add title and axis labels
title('Center of Mass Velocity over Time', 'FontSize', 14);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Center of Mass Velocity (COMv)', 'FontSize', 12);

% Add a grid for better readability
grid on;


figure;
plot(t, controller_state_his, 'b-', 'LineWidth', 2);

% Add title and axis labels
title('controller state over Time', 'FontSize', 14);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Cs', 'FontSize', 12);
end