function [COM_x,COM_y]=getCOM(Model_consts,State_variables)
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
    
    
    
    

    % Coordinates of the COM
    COM_x = (x0*m0 + x1*m1 + x2*m2 ) / (m0+m1+m2);  % Example, adjust depending on your object
    COM_y = (y0*m0 + y1*m1 + y2*m2 ) / (m0+m1+m2);
end