clc
clear all

%% Read test results files
path = uigetdir;

file_path_time = fullfile(path,'time.csv');
file_path_manip = fullfile(path,'manip.csv');
file_path_pose_error = fullfile(path,'pose_error.csv');
file_path_joint_pos = fullfile(path,'joint_pos.csv');
file_path_joint_vel = fullfile(path,'joint_vel.csv');
file_path_col_dist = fullfile(path,'col_dist.csv');
file_path_joint_constrainst = fullfile(path,'joint_constraints.csv');
file_path_ee_poses = fullfile(path,'ee_poses.csv');

% Read the files
time_raw = csvread(file_path_time)';
manip_raw = csvread(file_path_manip)';
pose_error_raw = csvread(file_path_pose_error)';
joint_pos_raw = csvread(file_path_joint_pos)';
joint_vel_raw = csvread(file_path_joint_vel)';
col_dist_raw = csvread(file_path_col_dist)';
joint_constraints_raw = csvread(file_path_joint_constrainst);
ee_poses_raw = csvread(file_path_ee_poses)';

% Erase path variables
clear file_path_time file_path_manip file_path_pose_error file_path_joint_pos
clear file_path_joint_vel file_path_col_dist file_path_joint_constrainst file_path_ee_poses
 
%% Assign the variables that correspond to the vectors in the plotting file

% Time
time = time_raw;

% Manipulabilities
MM_man_measure = manip_raw(1,:);
ur5_man_measure = manip_raw(2,:);
% W_measure = manip_raw(3,:);

% Position and orientation errors
xi_pos_error = pose_error_raw(1:3,:);
xi_orient_error = pose_error_raw(4:6,:);

% Joint positions
q = joint_pos_raw;

% MobPlatVelocities
mp_vel = joint_vel_raw(1:2,:);

% Joint velocities
dq = zeros(10, length(time));
dq(4:10,:) = joint_vel_raw(3:9,:);

% Joint constraints
q_limit = joint_constraints_raw(1:2,:)';
dq_limit = joint_constraints_raw(3,:)';

% Elbow and wrist collition distance
dist_elbow = col_dist_raw(1,:);
dist_wrist = col_dist_raw(2,:);
height_wrist = col_dist_raw(3,:);

% End-effector poses
xi = ee_poses_raw;


%% Plot the results
PlotEvolutionStylesAndColors

