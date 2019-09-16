clear all
close all

%Create an MARS_UR5 object
MARS=MARS_UR5();

%Joints values
% tx=2.65;
% ty=-1;
% phi_mp=0;
% tz=0.1;

% tx=0.378;
% ty=-0.284;
% phi_mp=0;
% tz=0.1;

tx=-1.65;
ty=-0.35;
phi_mp=0;
tz=0.24;

%qa=deg2rad([0; -70; 130; -150; -90; 0]);
%qa=[-0.341894000000000;-1.34758700000000;1.16749900000000;-1.39068200000000;-1.57089400000000;4.59478900000000];

qa=[4.80000000000000e-05;-1.39617400000000;1.91975800000000;-2.09436000000000;-1.57090600000000;8.40000000000000e-05];

%initial values of the generalized coordinates of the MM
q=[tx;ty;phi_mp;tz;qa];
Tf=MARS.forwardKin(q)
trans = Tf(1:3,4)
quat = rotm2quat(Tf(1:3,1:3))

%ROS x,y,z,w
%Matlab w,x,y,z
% temp = quat(1);
% quat(1:3)=quat(2:4);
% quat(4)=temp;
% quat

% T0=[1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];
% plotFrames(T0,Tf,0.5)
% %%% Euler angles %%%
% euler=rotm2eul(Tf(1:3,1:3),'ZYX')*180/pi

