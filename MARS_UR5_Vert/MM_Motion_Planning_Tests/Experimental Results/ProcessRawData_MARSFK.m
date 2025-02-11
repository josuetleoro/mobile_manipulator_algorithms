clc
clear all
addpath '../TrajPlan'
addpath '../MARS_UR5'

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
traj_duration = duration_from_stamps(stamp_end, stamp_start);
fprintf('Trajectory duration: %f\n', traj_duration);
fprintf('Reading tf raw data\n');
data_tf = get_traj_data(data_tf_raw, stamp_start, traj_duration);
fprintf('Reading mob plat raw data\n');
data_mob_plat = get_traj_data(data_mob_plat_raw, stamp_start, traj_duration);
fprintf('Reading joint states raw data\n');
data_jstates = get_traj_data(data_jstates_raw, stamp_start, traj_duration);


%% Joint positions and velocities
% Check which one is the vector with shortest length
N_jstates = length(data_jstates);
N_mob_plat = length(data_mob_plat);
N_tf = length(data_tf);
N_all = min([N_jstates, N_mob_plat, N_tf]) - 1;

% Extract the data correspoding to the shortest length
time_tf = data_tf(1,1:N_all);
time_mp_vel = data_mob_plat(1,1:N_all);
time_jstates = data_jstates(1,1:N_all);
mp_vel_exp = data_mob_plat(2:3,1:N_all);
z_pj_exp = data_jstates(2,1:N_all);
qa_exp = data_jstates(3:8,1:N_all);
dz_pj_exp = data_jstates(9,1:N_all);
dqa_exp = data_jstates(10:15,1:N_all);

%% Mobile platform and end effector (From vive position and from odom)
%Create a MMUR5 object
MARS=MARS_UR5();

% For vive position
xi_exp = zeros(7,N_all);
mp = zeros(3,N_all);
mp(1,:)=data_tf(2,1:N_all); % x
mp(2,:)=data_tf(3,1:N_all); % y

% For odom
ee_xi_odom = zeros(7,N_all);
mp_odom = zeros(3,N_all);
mp_odom(1,1)=data_tf(2,1); % x
mp_odom(2,1)=data_tf(3,1); % y
mp_orient_euler = quat2eul(data_tf(5:8,1)'); %ZYX order
mp_odom(3,1)=mp_orient_euler(1);
for i=1:N_all
    %% Vive pose
    mp_orient_euler = quat2eul(data_tf(5:8,i)'); %ZYX order
    mp(3,i)=mp_orient_euler(1);
    joints=[mp(1,i);mp(2,i);mp(3,i);z_pj_exp(i);qa_exp(:,i)];
    
    %Find the position of the end effector
    Tee=MARS.forwardKin(joints);
    xi_exp(1:3,i) = Tee(1:3,4);
    xi_exp(4:7,i) = cartToQuat(Tee(1:3,1:3));
    
    %% Odom    
    if i > 1
        ts=time_mp_vel(i)-time_mp_vel(i-1);        
        
        mp_odom(1,i)=mp_odom(1,i-1)+ts*mp_vel_exp(1,i)*cos(mp_odom(3,i-1));
        mp_odom(2,i)=mp_odom(2,i-1)+ts*mp_vel_exp(1,i)*sin(mp_odom(3,i-1));
        mp_odom(3,i)=mp_odom(3,i-1)+ts*mp_vel_exp(2,i);        
    end    
    joints=[mp_odom(1,i);mp_odom(2,i);mp_odom(3,i);z_pj_exp(i);qa_exp(:,i)];    
    %Find the position of the end effector
    Tee=MARS.forwardKin(joints);
    ee_xi_odom(1:3,i) = Tee(1:3,4);
    ee_xi_odom(4:7,i) = cartToQuat(Tee(1:3,1:3));    
end
mp = mp_odom;
xi_exp = ee_xi_odom;

%Create the joint positions matrix
q_exp = zeros(10,N_all);
q_exp(1:3,:)=mp_odom;
q_exp(4,:)=z_pj_exp;
q_exp(5:10,:)=qa_exp;

%% Calculate original trajectory with the time_tf vector
file_path_original_traj = strcat(path, test_name, ".mat");
load(file_path_original_traj,'q','xi_des','xi','time');
ts = time(2)-time(1);
[xi_pos_error,xi_orient_error]=CalcTrajError(xi_exp,xi_des,time_tf,ts);
PlotEvolutionPretty_Exp

%% Compare original joint velocities with obtained joint velocities
dq_exp = [dz_pj_exp; dqa_exp];
PlotJointVelComp(strcat(path, test_name, ".mat"),time_mp_vel,mp_vel_exp,time_jstates,dq_exp)

PlotEEComp




