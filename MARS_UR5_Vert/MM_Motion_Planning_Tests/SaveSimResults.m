fileName = ['Test' num2str(testN) '.mat'];
[file,path]=uiputfile(fileName);

% Simulation data
sim_data_path = [path file];
save(sim_data_path,'MM_man_measure','ur5_man_measure','W_measure','dist_elbow','dist_wrist','wrist_pos','q_limit','dq_limit','xi_pos_error','xi_orient_error','xi','q','dq','mp_vel','xi_des','time');