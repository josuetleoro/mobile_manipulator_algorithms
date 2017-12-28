clear all
close all

%Create an MARS_UR5 object
MARS=MARS_UR5();

%Joints values
tx=1.0;
ty=2.0;
phi_mp=0.5;
tz=0.2;
qa=[0.12; -1.5; 2.06; -2.06; -0.56; 0.48];
%qa=[pi;-pi/4;pi/4;0;pi/2;0];

%initial values of the generalized coordinates of the MM
q=[tx;ty;phi_mp;tz;qa];
T0=MARS.forwardKin(q);
translation = T0(1:3,4)
orientation = rotm2quat(T0(1:3,1:3))
