function [COMv_x,COMv_y]=getCOMv(Model_consts,State_variables,solutions_dd)
    dt=0.0001;
    [COM_x0,COM_y0]=getCOM(Model_consts,State_variables);
    Controls_zero=[0,0];
    new_State_variables=update_rk4(Model_consts,State_variables,Controls_zero,dt,solutions_dd);
    [COM_x1,COM_y1]=getCOM(Model_consts,new_State_variables);
    COMv_x=(COM_x1-COM_x0)/dt;
    COMv_y=(COM_y1-COM_y0)/dt;
end