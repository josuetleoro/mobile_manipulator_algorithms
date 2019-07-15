clc
clear all
addpath '../TrajPlan'
addpath '../MARS_UR5'

%% Read test results files
path = uigetdir;

file_path_stamps = fullfile(path,'traj_stamps.csv');
file_path_tf = fullfile(path,'tf_record.csv');
file_path_traj_param = fullfile(path,'traj_parameters.csv');

%Read the files
data_stamps = csvread(file_path_stamps);
data_tf_raw = csvread(file_path_tf);
data_traj_parameters_raw = csvread(file_path_traj_param);

%% Get the data corresponding to the trajectory execution period
% Stamps of the trajectory send to robot
stamp_start = data_stamps(1,1:2);
stamp_end = data_stamps(2,1:2);
traj_duration = duration_from_stamps(stamp_end, stamp_start);
fprintf('Trajectory duration: %f\n', traj_duration);
fprintf('Reading tf raw data\n');
data_tf = get_traj_data(data_tf_raw, stamp_start, traj_duration);
ee_xi = data_tf(2:8,:);

%% Calculate original trajectory with the time_tf vector
xi_des = zeros(7,2);
xi_des(:,1) = data_traj_parameters_raw(1,1:7)';
xi_des(:,2) = data_traj_parameters_raw(2,1:7)';
ts = data_traj_parameters_raw(3,1);
tf = data_traj_parameters_raw(4,1);
time = data_tf(1,:);
[xi_pos_error,xi_orient_error]=CalcTrajError(ee_xi,xi_des,time,ts);

%% Plot the position and orientation error
%Plots properties
blue=[0    0.4470    0.7410];
red=[0.8500    0.3250    0.0980];
yellow=[0.9290    0.6940    0.1250];
purple=[0.4940    0.1840    0.5560];
green=[0.4660    0.6740    0.1880];
cyan=[0.3010    0.7450    0.9330];
brown=[0.6350    0.0780    0.1840];
labelFontSize=14;
lineWidth=1.8;

set(0,'defaulttextinterpreter','latex')
set(0,'defaulttextfontname', 'Times')
set(0,'defaulttextfontsize',16)

set(0, 'defaultAxesTickLabelInterpreter','latex');
set(0, 'defaultLegendInterpreter','latex');
set(0,'defaultaxesfontsize',16)
set(0,'defaultaxesfontname', 'Times')

plots_end_time = time(end);

figure()
plot(time,xi_pos_error(1,:),'LineWidth',lineWidth); hold on
plot(time,xi_pos_error(2,:),'LineWidth',lineWidth); hold on
plot(time,xi_pos_error(3,:),'LineWidth',lineWidth);
xlim([0 plots_end_time])
xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(m)$','interpreter','latex','FontSize',labelFontSize)
legend('$e_{Px}$','$e_{Py}$','$e_{Pz}$','interpreter','latex','FontSize',labelFontSize)
grid on
title('Position error(m)')
fprintf('\nFinal Position Error:');
xi_pos_error(:,end)'
fprintf('Pos norm error: %fm\n',norm(xi_pos_error(:,end))');

figure()
plot(time,xi_orient_error(1,:),'LineWidth',lineWidth); hold on
plot(time,xi_orient_error(2,:),'LineWidth',lineWidth); hold on
plot(time,xi_orient_error(3,:),'LineWidth',lineWidth);
xlim([0 plots_end_time])
xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(rad)$','interpreter','latex','FontSize',labelFontSize)
legend('$e_{Ox}$','$e_{Oy}$','$e_{Oz}$','interpreter','latex','FontSize',labelFontSize)
grid on
title('Orientation error(rad)')
fprintf('\nFinal Orientation Error');
xi_orient_error(:,end)'
fprintf('Orientation norm error: %f\n',norm(xi_orient_error(:,end))');
