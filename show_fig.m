%Model consts is a dictionary of settings r,l0,l1,l2,m0,m1,m2,g
%state variable is a matrix of theta,alpha,beta, dtheta,dalpha,dbeta
function []=show_fig(Model_consts,State_variables,time_i) 
    
    cla;
    clf('reset');
    xlim([-1.5, 1.5]);
    ylim([-1, 2]);
    hold on;
    axis equal;
    
    r=Model_consts('r');
    l0=Model_consts('l0');
    l1=Model_consts('l1');
    l2=Model_consts('l2');
    m0=Model_consts('m0');
    m1=Model_consts('m1');
    m2=Model_consts('m2');

    theta=State_variables(1);
    alpha=State_variables(2);
    beta=State_variables(3);

    x0 = -sin(theta)*r + theta*cos(theta)*r-cos(theta)*l0;
    y0 = cos(theta)*r + theta*sin(theta)*r-sin(theta)*l0;
    x1 = x0 - sin(alpha)*l1;
    y1 = y0 + cos(alpha)*l1;
    x2 = x1 - sin(beta)*l2;
    y2 = y1 + cos(beta)*l2;
    x3=-sin(theta)*r + theta*cos(theta)*r+cos(theta)*l0;
    y3=cos(theta)*r + theta*sin(theta)*r+sin(theta)*l0;
    
    rectangle('Position', [-r, -r, 2*r, 2*r], 'Curvature', [1, 1]);
    plot([x3, x0], [y3, y0], 'r', 'LineWidth', 2);
    plot([x0, x1], [y0, y1], 'g', 'LineWidth', 2);
    plot([x1, x2], [y1, y2], 'b', 'LineWidth', 2);
    
    

    % Coordinates of the COM
    %COM_x = (x0*m0 + x1*m1 + x2*m2 ) / (m0+m1+m2);  % Example, adjust depending on your object
    %COM_y = (y0*m0 + y1*m1 + y2*m2 ) / (m0+m1+m2);
    [COM_x,COM_y]=getCOM(Model_consts,State_variables);
    % Plotting the COM
    
    plot(COM_x, COM_y, 'ko', 'MarkerFaceColor', 'k');  % Black dot for COM
    
    % Drawing the vertical dashed line from COM to the bottom of the plot
    
    plot([COM_x, COM_x], [-2*r, COM_y], 'k--');
    
    % Annotating the COM coordinates
    text(COM_x, -2*r - 0.1, sprintf('COM: (%.3f, %.3f)', COM_x, COM_y), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    text(COM_x, -2*r - 0.3, sprintf('Time: %.3f', time_i), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    % Update the plot
    drawnow;
end