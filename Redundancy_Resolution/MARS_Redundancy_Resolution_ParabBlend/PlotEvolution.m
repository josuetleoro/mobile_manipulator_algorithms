% clear all
% 
% %Open a dialog box to look for the motion data
% uiopen();
% 
% % Show the final position
% xi(:,end)

addpath MARS_UR5
%Create a MMUR5 object
MARS=MARS_UR5();
qf=(q(:,end));
Tf=MARS.forwardKin(qf);

%% Manipulability plots

figure()
plot(time,MM_man_measure,'b','LineWidth',1.5); hold on;
plot(time,ur5_man_measure,'r','LineWidth',1.5); hold off
legend('MM_{manip}','UR5_{manip}')
grid on

%% End Effector Position
figure()
subplot(1,3,1)
plot(time,xi(1,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('End effector pos x(m)')

subplot(1,3,2)
plot(time,xi(2,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('End effector pos y(m)')
title('End effector position')

subplot(1,3,3)
plot(time,xi(3,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('End effector pos z(m)')

figure()
subplot(2,2,1)
plot(time,xi(4,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('quat_w')
tix=get(gca,'ytick')';
set(gca,'yticklabel',num2str(tix,'%.3f'))

subplot(2,2,2)
plot(time,xi(5,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('quat_x')
tix=get(gca,'ytick')';
set(gca,'yticklabel',num2str(tix,'%.3f'))

subplot(2,2,3)
plot(time,xi(6,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('quat_y')
tix=get(gca,'ytick')';
set(gca,'yticklabel',num2str(tix,'%.3f'))

subplot(2,2,4)
plot(time,xi(6,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('quat_z')
tix=get(gca,'ytick')';
set(gca,'yticklabel',num2str(tix,'%.3f'))



%% Mobile platform movement
figure()
subplot(2,2,1)
plot(time,q(1,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('Mobile Platform x(m)')

subplot(2,2,2)
plot(time,q(2,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('Mobile Platform y(m)')

subplot(2,2,3)
plot(time,q(3,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('Mobile Platform phi(rad)')

subplot(2,2,4)
plot(time,q(4,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('Mobile Platform z(m)')

%% Mobile platform velocities (including prismatic joint)
figure()
subplot(1,3,1)
plot(time,mp_vel(1,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('Linear velocity v(m/s)')

subplot(1,3,2)
plot(time,mp_vel(2,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('Angular velocity w(rad/s)')
title('Mobile platform velocities')

subplot(1,3,3)
plot(time,mp_vel(3,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('Prismatic joint velociti dz(m/s)')

%% Angles of the joints of the UR5
figure()
subplot(2,3,1)
plot(time,q(5,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('q1(rad)')
title('UR5 q1')

subplot(2,3,2)
plot(time,q(6,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('q2(rad)')
title('UR5 q2')

subplot(2,3,3)
plot(time,q(7,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('q3(rad)')
title('UR5 q3')

subplot(2,3,4)
plot(time,q(8,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('q4(rad)')
title('UR5 q4')

subplot(2,3,5)
plot(time,q(9,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('q5(rad)')
title('UR5 q5')

subplot(2,3,6)
plot(time,q(10,:),'LineWidth',1.5); grid on
xlabel('time(s)')
ylabel('q6(rad)')
title('UR5 q6')

%% UR5 joints velocities
figure()
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

%% Mobile Platform Trajectory

%Draw the mobile manipulator in the current position
%drawMobilePlatform(q(1,1),q(2,1),q(3,1),xi(1,1),xi(2,1))

figure()
clear PosNF

q1=q(1,:);
q2=q(2,:);
q3=q(3,:);

%In the reference frame
plot(q2,q1,'LineWidth',1.5); hold on; grid on

% %In the body frame
% n=length(q1);
% for i=1:n
% Pos=[q1(i);q2(i);0];
% R=[cos(q3(i)),sin(q3(i)),0;-sin(q3(i)),cos(q3(i)),0;0,0,1];
% PosNF(:,i)=R*Pos;
% end
% %plot(PosNF(2,:),PosNF(1,:),'LineWidth',1.5); hold on; grid on
% plot(time,PosNF(2,:),'LineWidth',1.5); hold on; grid on

title('Mobile Platform Trajectory')
xlabel('y(m)');
ylabel('x(m)');
%axis([-0.9,0.9,-0.2,2]);
hold off;