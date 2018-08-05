fileName = ['Test' num2str(testN) '.mat'];
uisave({'MM_man_measure','ur5_man_measure','dist','q_limit','dq_limit','xi_pos_error','xi','q','dq','mp_vel','xi_des','dxi_des','time'},fileName);
clear fileName;