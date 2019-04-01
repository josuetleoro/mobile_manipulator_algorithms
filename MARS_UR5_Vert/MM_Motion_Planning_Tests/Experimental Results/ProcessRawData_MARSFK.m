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
%Check which one is the vector with shortest length

N_jstates = length(data_jstates);
N_mob_plat = length(data_mob_plat);
N_tf = length(data_tf);
N_all = min([N_jstates, N_mob_plat, N_tf]);

time_jstates = data_jstates(1,1:N_all);
z_pj_exp = data_jstates(2,1:N_all);
qa_exp = data_jstates(3:8,1:N_all);
dz_pj_exp = data_jstates(9,1:N_all);
dqa_exp = data_jstates(10:15,1:N_all);

%% Mobile platform and end effector
time_tf = data_tf(1,1:N_all);
ee_xi = zeros(7,N_all);
mp = zeros(3,N_all);
mp(1,:)=data_tf(2,1:N_all);
mp(2,:)=data_tf(3,1:N_all);
%Create a MMUR5 object
MARS=MARS_UR5();
for i=1:N_all
    mp_orient_euler = quat2eul(data_tf(5:8,i)'); %ZYX order
    mp(3,i)=mp_orient_euler(1);
    joints=[mp(1,i);mp(2,i);mp(3,i);z_pj_exp(i);qa_exp(:,i)];
    
    %Find the position of the end effector
    Tee=MARS.forwardKin(joints);
    %pause()
    %Pos_0=T0(1:3,4)
    %quat_0=cartToQuat(T0(1:3,1:3))
    ee_xi(1:3,i) = Tee(1:3,4);
    ee_xi(4:7,i) = cartToQuat(Tee(1:3,1:3));
end
mp_vel_exp = data_mob_plat(2:3,1:N_all);
time_mp_vel = data_mob_plat(1,1:N_all);
%ee_xi = data_tf(9:15,:);

%% Calculate original trajectory with the time_tf vector
file_path_original_traj = strcat(path, test_name, ".mat");
load(file_path_original_traj,'xi_des','time');
ts = time(2)-time(1);
[xi_pos_error,xi_orient_error]=CalcTrajError(ee_xi,xi_des,time_tf,ts);
PlotEvolutionPretty_Exp

%% Compare original joint velocities with obtained joint velocities
PlotJointVelComp(strcat(path, test_name, ".mat"),time_mp_vel,mp_vel_exp,time_jstates,dqa_exp)

%% Save data in a .mat file
%file_name_proc_data = strcat(path, test_name, "_proc.mat");
%save(file_name_proc_data,'time_tf','mp','ee_xi','time_mp_vel','mp_vel','time_jstates','z_pj','dz_pj','qa','dqa');





