clear all
close all

%Open a dialog box to look for the motion data
uiopen();

% Show the final position
xi(:,end)

addpath MARS_UR5
%Create a MMUR5 object
MARS=MARS_UR5();
qf=(q(:,end));
Tf=MARS.forwardKin(qf);

%% Manipulability plots

f1=figure(1);
plot(time,MM_man_measure,'b','LineWidth',1.5); hold on;
plot(time,ur5_man_measure,'r','LineWidth',1.5); hold off
legend('MM_{manip}','UR5_{manip}')
xlabel('time(s)')
grid on
movegui(f1,'northwest')

%% Mobile platform velocities (including prismatic joint)
f2 = figure(2);
subplot(1,3,1)
plot(time,mp_vel(1,:),'LineWidth',1.5); grid on
% maxVel = refline(0,dq_limit(1));
% maxVel.Color = 'r';
% minVel = refline(0,-dq_limit(1));
% minVel.Color = 'r';
xlabel('time(s)')
ylabel('Linear velocity v(m/s)')

subplot(1,3,2)
plot(time,mp_vel(2,:),'LineWidth',1.5); grid on
% maxVel = refline(0,dq_limit(2));
% maxVel.Color = 'r';
% minVel = refline(0,-dq_limit(2));
% minVel.Color = 'r';
xlabel('time(s)')
ylabel('Angular velocity w(rad/s)')
title('Mobile platform velocities')

subplot(1,3,3)
plot(time,mp_vel(3,:),'LineWidth',1.5); grid on
% maxVel = refline(0,dq_limit(3));
% maxVel.Color = 'r';
% minVel = refline(0,-dq_limit(3));
% minVel.Color = 'r';
xlabel('time(s)')
ylabel('Prismatic joint velociti dz(m/s)')
movegui(f2,'northeast')


%% UR5 joints velocities
f3 = figure(3);
subplot(2,3,1)
plot(time,dq(5,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('dq1(rad/s)')
title('UR5 dq1')

subplot(2,3,2)
plot(time,dq(6,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('dq2(rad/s)')
title('UR5 dq2')

subplot(2,3,3)
plot(time,dq(7,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('dq3(rad/s)')
title('UR5 dq3')

subplot(2,3,4)
plot(time,dq(8,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('dq4(rad/s)')
title('UR5 dq4')

subplot(2,3,5)
plot(time,dq(9,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('dq5(rad/s)')
title('UR5 dq5')

subplot(2,3,6)
plot(time,dq(10,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('dq6(rad/s)')
title('UR5 dq6')
movegui(f3,'southwest')

%% Mobile Platform Trajectory

%Draw the mobile manipulator in the current position
%drawMobilePlatform(q(1,1),q(2,1),q(3,1),xi(1,1),xi(2,1))

f4=figure(4);
clear PosNF

q1=q(1,:);
q2=q(2,:);
q3=q(3,:);

%In the reference frame
plot(q1,q2,'LineWidth',1.5); hold on; grid on

%In the body frame
n=length(q1);
for i=1:n
Pos=[q1(i);q2(i);0];
R=[cos(q3(i)),sin(q3(i)),0;-sin(q3(i)),cos(q3(i)),0;0,0,1];
PosNF(:,i)=R*Pos;
end
%plot(PosNF(2,:),PosNF(1,:),'LineWidth',1.5); hold on; grid on

title('Mobile Platform Trajectory')
xlabel('x(m)');
ylabel('y(m)');
%axis([-0.9,0.9,-0.2,2]);
hold off;
movegui(f4,'southeast')