function State_variables=init(Model_consts)
    r=Model_consts('r');
    l0=Model_consts('l0');
    l1=Model_consts('l1');
    l2=Model_consts('l2');
    m0=Model_consts('m0');
    m1=Model_consts('m1');
    m2=Model_consts('m2');
    
    State_variables=[0,0,0,0,0,0];
    State_variables(3)=-asin((m0+m1+m2)/m2*l0/l2);
    %State_variables(3)=-asin((l0*(m0 + m1 + m2))/(m2*(l1 + l2) + l1*m1));

    %State_variables(2)=-asin((l0*(m0 + m1 + m2))/(m2*(l1 + l2) + l1*m1));
end
