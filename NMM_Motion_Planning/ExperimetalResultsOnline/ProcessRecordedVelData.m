clc
clear all

%% Read test results files
path = uigetdir;

file_path_time_stamps = fullfile(path, 'traj_stamps.csv');
%file_path_tf = fullfile(path, 'tf_data','TestN_tf_record.csv');
file_path_mob_plat = fullfile(path, 'tf_data', "TestN_mob_plat_record.csv");
file_path_joint_states = fullfile(path, 'tf_data', "TestN_joint_states_record.csv");
file_path_joint_constrainst = fullfile(path,'joint_constraints.csv');

%Read the files
data_stamps = csvread(file_path_time_stamps);
%data_tf_raw = csvread(file_path_tf);
data_mob_plat_raw = csvread(file_path_mob_plat);
data_jstates_raw = csvread(file_path_joint_states);
joint_constraints_raw = csvread(file_path_joint_constrainst);

%% Get the data corresponding to the trajectory execution period
% Stamps of the trajectory send to robot
stamp_start = data_stamps(1,1:2);
stamp_end = data_stamps(2,1:2);
traj_duration = duration_from_stamps(stamp_end, stamp_start);
fprintf('Trajectory duration: %f\n', traj_duration);
% fprintf('Reading tf raw data\n');
% data_tf = get_traj_data(data_tf_raw, stamp_start, traj_duration);
fprintf('Reading mob plat raw data\n');
data_mob_plat = get_traj_data(data_mob_plat_raw, stamp_start, traj_duration);
fprintf('Reading joint states raw data\n');
data_jstates = get_traj_data(data_jstates_raw, stamp_start, traj_duration);

%% Joint positions and velocities
% Check which one is the vector with shortest length
N_jstates = length(data_jstates);
N_mob_plat = length(data_mob_plat);
N_tf = 9999999999;
N_all = min([N_jstates, N_mob_plat, N_tf]) - 1;

% Extract the data correspoding to the shortest length
time_mp_vel = data_mob_plat(1,1:N_all);
time_jstates = data_jstates(1,1:N_all);
mp_vel = data_mob_plat(2:3,1:N_all);

% Joint positions
q = zeros(10, N_all);
q(4,:) = data_jstates(2,1:N_all);
q(5:10,:) = data_jstates(3:8,1:N_all);

% Joint velocities
dq = zeros(10, N_all);
dq(4,:) = data_jstates(9,1:N_all);
dq(5:10,:) = data_jstates(10:15,1:N_all);

% Joint constraints
q_limit = joint_constraints_raw(1:2,:)';
dq_limit = joint_constraints_raw(3,:)';

%% Plot
PlotJointVels





