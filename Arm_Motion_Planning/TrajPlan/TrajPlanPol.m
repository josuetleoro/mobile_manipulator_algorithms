function MotPlan=TrajPlanPol(pos0,q0,posf,qf,ts,tf)

%% Motion planning for positon (x,y,z)
[x,dx,~]=Traj5(pos0(1),posf(1),ts,0,tf);
[y,dy,~]=Traj5(pos0(2),posf(2),ts,0,tf);
[z,dz,~]=Traj5(pos0(3),posf(3),ts,0,tf);

%% Motion planning for orientation using quaternion polynomial
%Create the quaternion objects
Q0=Quat(q0');
Qf=Quat(qf');

%Desired initial and final angular velocities and accelerations
w0=[0;0;0];
dw0=[0;0;0];
wf=[0;0;0];
dwf=[0;0;0];
%Do the interpolation
%[quat,w,~]=quatInterpolation(Q0,w0,dw0,Qf,wf,dwf,0,tf,ts);
[quat,w,~]=quatPolynomInterpolation(Q0,w0,dw0,Qf,wf,dwf,0,tf,ts,'fixed');

time=0:ts:tf;

%Plots properties
blue=[0    0.4470    0.7410];
labelFontSize=14;
lineWidth=1.8;

set(0,'defaulttextinterpreter','latex')
set(0,'defaulttextfontname', 'Times')
set(0,'defaulttextfontsize',16)

set(0, 'defaultAxesTickLabelInterpreter','latex');
set(0, 'defaultLegendInterpreter','latex');
set(0,'defaultaxesfontsize',16)
set(0,'defaultaxesfontname', 'Times')

% %Show the end effector motion in 3D
% figure()
% Rd=zeros(4,4,length(time));
% %Form the T6Traj matrix
% for i=1:length(x)
%    Rd(:,4,i)=[x(i);y(i);z(i);1];
%    Rd(1:3,1:3,i)=quatToRotMat(quat(:,i));
%    %Rd(1:3,1:3,i)=quat2rotm(quat(:,i)');
% end
% plotEndEffectorMotion(Rd,5)
% pause()

%Plot the evolution of x, y and z
% figure()
% subplot(1,3,1)
% plot(time,x,'Color',blue,'LineWidth',2); grid on
% xlabel('$t(s)$')
% ylabel('$x$')
% 
% subplot(1,3,2)
% plot(time,y,'Color',blue,'LineWidth',2); grid on
% xlabel('$t(s)$')
% ylabel('$y$')
% 
% subplot(1,3,3)
% plot(time,z,'Color',blue,'LineWidth',2); grid on
% xlabel('$t(s)$')
% ylabel('$z$')
% 
% %Plot the evolution of dx, dy and dz
% figure()
% subplot(1,3,1)
% plot(time,dx,'Color',blue,'LineWidth',2); grid on
% xlabel('$t(s)$')
% ylabel('$\dot{x}$')
% 
% subplot(1,3,2)
% plot(time,dy,'Color',blue,'LineWidth',2); grid on
% xlabel('$t(s)$')
% ylabel('$\dot{d}$')
% 
% subplot(1,3,3)
% plot(time,dz,'Color',blue','LineWidth',2); grid on
% xlabel('$t(s)$')
% ylabel('$\dot{z}$')
% 
% %Plot the evolution of wx, wy and wz
% figure()
% subplot(1,3,1)
% plot(time,w(1,:),'Color',blue,'LineWidth',2); grid on
% xlabel('$t(s)$')
% ylabel('$\omega_x$')
% 
% subplot(1,3,2)
% plot(time,w(2,:),'Color',blue,'LineWidth',2); grid on
% xlabel('$t(s)$')
% ylabel('$\omega_y$')
% 
% subplot(1,3,3)
% plot(time,w(3,:),'Color',blue,'LineWidth',2); grid on
% xlabel('$t(s)$')
% ylabel('$\omega_z$')
% 
% %Plot the evolution of quat
% time=0:ts:tf;
% figure()
% subplot(2,2,1)
% plot(time,quat(1,:),'Color',blue,'LineWidth',2); grid on
% xlabel('$t(s)$')
% ylabel('$w_{quat}$')
% 
% subplot(2,2,2)
% plot(time,quat(2,:),'Color',blue,'LineWidth',2); grid on
% xlabel('$t(s)$')
% ylabel('$x_{quat}$')
% 
% subplot(2,2,3)
% plot(time,quat(3,:),'Color',blue,'LineWidth',2); grid on
% xlabel('$t(s)$')
% ylabel('$y_{quat}$')
% 
% subplot(2,2,4)
% plot(time,quat(4,:),'Color',blue,'LineWidth',2); grid on
% xlabel('$t(s)$')
% ylabel('$z_{quat}$')
% pause()

%Return the motion planning data
MotPlan={};
MotPlan.x=x;
MotPlan.dx=dx;
MotPlan.y=y;
MotPlan.dy=dy;
MotPlan.z=z;
MotPlan.dz=dz;
MotPlan.quat=quat;
MotPlan.w=w;
MotPlan.time=time;
end

