fileName = ['Test' num2str(testN) '.mat'];
[file,path]=uiputfile(fileName);

% Simulation data
sim_data_path = [path file];
save(sim_data_path,'q','mp_vel','time','dq_limit');