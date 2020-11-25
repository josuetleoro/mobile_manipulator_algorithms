clear all
addpath SimulateMotionFunctions

%Name of the test
testN='MobPlat';

%% Plots properties
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

%% Initial joints values
tx=0;
ty=0;
phi_mp=0;
tz=0.2;
% Arm joint values
qa=deg2rad([0;-80;110;-120;-90;0.0]);
% Show platform initial pose
disp('Desired Pose')
q_des=[tx;ty;phi_mp]

tx_d=0.779;
ty_d=-1.345;
phi_mp_d=0;
% Show platform desired pose
disp('Desired Pose')
q_des=[tx_d;ty_d;phi_mp_d]

JointConstraints

tf=8;
ts=1/20;

%% Path planning
[x,y,phi,v,w]=path_planning(tx,ty,phi_mp,tx_d,ty_d,phi_mp_d,2,0,tf,ts);
time=0:ts:tf;
N = length(time);

q = [x;y;phi;repmat(tz,1,N);repmat(qa,1,N)];

mp_vel = [v;w];

%% Simulate motion in 2D

% SimulateMotion


%% Plot all the variables
PlotEvolutionPretty
