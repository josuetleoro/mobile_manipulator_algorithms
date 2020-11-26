close all
clear all

%% Read the motion planning results files

% Mobile platform motion planning data
disp('Select the mobile platform motion planning data')
[mp_file, path] = uigetfile('*.mat');
mp_data = load(fullfile(path,mp_file));

disp('Select the arm motion planning data')
[arm_file, path] = uigetfile('*.mat');
arm_data = load(fullfile(path,arm_file));

disp('Select the mobile manipulator motion planning data')
[mm_file, path] = uigetfile('*.mat');
mm_data = load(fullfile(path,mm_file));

%% Put the data for simulation together

% Mobile platform
mp_des = mp_data.q(:,end);
q_mp = mp_data.q;

% Robot arm
arm_des = arm_data.xi_des(:,end);
xi_arm = arm_data.xi_des;
q_arm = arm_data.q;

% Mobile manipulator
mm_des = mm_data.xi_des(:,end);
xi_mm = mm_data.xi_des;
q_mm = mm_data.q;

% Save the obtained data
fileName = 'Results.mat';
[file,path]=uiputfile(fileName);

% Simulation data
sim_data_path = [path file];
save(sim_data_path,'mp_des', 'q_mp', 'arm_des', 'xi_arm', 'q_arm', 'mm_des', 'xi_mm', 'q_mm');