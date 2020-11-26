fileName = ['Test' num2str(testN) '.mat'];
[file,path]=uiputfile(fileName);

% Simulation data
sim_data_path = [path file];
save(sim_data_path,'xi_des','xi_pos_error','xi_orient_error','q','time','q_limit','dq_limit');