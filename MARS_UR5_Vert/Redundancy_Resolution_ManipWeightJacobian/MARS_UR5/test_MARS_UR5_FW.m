clear all
close all

%Create an MARS_UR5 object
MARS=MARS_UR5();

%Joints values
tx=0.0;
ty=0.0;
phi_mp=0.0;
tz=0.0;
%qa=[0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
%qa=[pi;-pi/4;pi/4;0;pi/2;0];
%qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
qa=[0.4;0.6;-0.8;1.0;-1.2;1.4];
%qa=[0.0; -pi; 3*pi/4; -3*pi/4; -pi/2; 0.0];

%initial values of the generalized coordinates of the MM
q=[tx;ty;phi_mp;tz;qa];
Tf=MARS.forwardKin(q)
trans = Tf(1:3,4)
quat = rotm2quat(Tf(1:3,1:3));

%ROS x,y,z,w
%Matlab w,x,y,z
temp = quat(1);
quat(1:3)=quat(2:4);
quat(4)=temp;
quat

T0=[1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];

plotFrames(T0,Tf,0.5)

%%% Euler angles %%%
euler=rotm2eul(Tf(1:3,1:3),'ZYX')*180/pi

