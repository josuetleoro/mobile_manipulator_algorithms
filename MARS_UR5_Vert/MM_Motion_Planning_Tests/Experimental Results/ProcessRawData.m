clc
clear all
addpath '..\TrajPlan'
addpath '..\MARS_UR5'

%% Read test results files
[file, path]=uigetfile('*.csv');
file_name_parts = split(file,["_","."]);
test_name = file_name_parts{1};
% path = "C:\Users\josue\Documents\GitHub\mobile_manipulator_algorithms\MARS_UR5_Vert\MM_Motion_Planning_Tests\Experimental Results\2019_02_21\Test3\";
% test_name = "Test3";
file_path_traj_stamps = strcat(path, test_name, "_traj_stamps.csv");
file_path_tf = strcat(path, test_name, "_tf_record.csv");
file_path_mob_plat = strcat(path, test_name, "_mob_plat_record.csv");
file_path_joint_states = strcat(path, test_name, "_joint_states_record.csv");

%Read the files
data_stamps = csvread(file_path_traj_stamps);
data_tf_raw = csvread(file_path_tf);
data_mob_plat_raw = csvread(file_path_mob_plat);
data_jstates_raw = csvread(file_path_joint_states);

%% Get the data corresponding to the trajectory execution period
% Stamps of the trajectory send to robot
stamp_start = data_stamps(1,1:2);
stamp_end = data_stamps(2,1:2);
traj_duration = floor(duration_from_stamps(stamp_end, stamp_start));
fprintf('Trajectory duration: %f\n', traj_duration);
fprintf('Reading tf raw data\n');
data_tf = get_traj_data(data_tf_raw, stamp_start, traj_duration);
fprintf('Reading mob plat raw data\n');
data_mob_plat = get_traj_data(data_mob_plat_raw, stamp_start, traj_duration);
fprintf('Reading joint states raw data\n');
data_jstates = get_traj_data(data_jstates_raw, stamp_start, traj_duration);

%% Mobile platform and end effector
N_tf = length(data_tf);
time_tf = data_tf(1,:);
mp = zeros(3,N_tf);
mp(1,:)=data_tf(2,:);
mp(2,:)=data_tf(3,:);
for i=1:N_tf
    mp_orient_euler = quat2eul(data_tf(5:8,i)'); %ZYX order
    mp(3,i)=mp_orient_euler(1);
end
mp_vel = data_mob_plat(2:3,:);
time_mp_vel = data_mob_plat(1,:);
ee_xi = data_tf(9:15,:);

%% Joint positions and velocities
N_jstates = length(data_jstates);
time_jstates = data_jstates(1,:);
z_pj = data_jstates(2,:);
qa = data_jstates(3:8,:);
dz_pj = data_jstates(9,:);
dqa = data_jstates(10:15,:);

%% Calculate original trajectory with the time_tf vector
file_path_original_traj = strcat(path, test_name, ".mat");
load(file_path_original_traj,'xi_des','time');
ts = time(2)-time(1);
[xi_pos_error,xi_orient_error]=CalcTrajError(ee_xi,xi_des,time_tf,ts);
PlotEvolutionPretty_Exp

%% Save data in a .mat file
%file_name_proc_data = strcat(path, test_name, "_proc.mat");
%save(file_name_proc_data,'time_tf','mp','ee_xi','time_mp_vel','mp_vel','time_jstates','z_pj','dz_pj','qa','dqa');





