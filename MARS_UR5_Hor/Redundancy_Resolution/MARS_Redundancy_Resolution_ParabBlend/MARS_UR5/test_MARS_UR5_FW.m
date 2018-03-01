clear all
close all

%Create an MARS_UR5 object
MARS=MARS_UR5();

%Joints values
tx=0.0;
ty=0.0;
phi_mp=0.0;
tz=0.0;
%qa=[0.1; -0.2; 0.3; -0.4; 0.5; -0.6];
%qa=[pi; -pi/4; pi/4; 0; pi/2; 0];
%qa=[0;-180; 90;-90; 0; 0]*pi/180;
%qa=[0;-pi; pi/2; -pi/2; 0; 0];
qa=[4.5;-3.4; 2.3; -1.2; -0.1; 0];
%qa=[0.0; -pi; 3*pi/4; -3*pi/4; -pi/2; 0.0];

%initial values of the generalized coordinates of the MM
q=[tx;ty;phi_mp;tz;qa];
Tf=MARS.forwardKin(q)
translation = Tf(1:3,4)
orientation = rotm2quat(Tf(1:3,1:3));
quat(1:3)=orientation(2:4);
quat(4)=orientation(1)


orientation2 = orientation*-1;
quat2(1:3)=orientation2(2:4);
quat2(4)=orientation2(1)
R2=quat2rotm(orientation2);

% T0=[1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];
% 
% plotFrames(T0,Tf,0.5)

%%% Euler angles %%%
euler=rotm2eul(Tf(1:3,1:3),'ZYX')*180/pi

