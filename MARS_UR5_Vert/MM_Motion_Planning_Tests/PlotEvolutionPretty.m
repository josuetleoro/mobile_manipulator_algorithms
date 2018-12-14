% close all
% clear all
% %Open a dialog box to look for the motion data
% uiopen();
% % Show the final position
% xi(:,end)

addpath MARS_UR5
%Create a MMUR5 object
MARS=MARS_UR5();
qf=(q(:,end));
Tf=MARS.forwardKin(qf);

%Plots properties
blue=[0    0.4470    0.7410];
red=[0.8500    0.3250    0.0980];
yellow=[0.9290    0.6940    0.1250];
purple=[0.4940    0.1840    0.5560];
green=[0.4660    0.6740    0.1880];
cyan=[0.3010    0.7450    0.9330];
brown=[0.6350    0.0780    0.1840];
labelFontSize=12;
lineWidth=1.8;
set(0,'defaulttextinterpreter','latex')
set(0,'DefaultTextFontname', 'CMU Serif')
set(0,'DefaultAxesFontName', 'CMU Serif')

%% Manipulability plots
figure()
plot(time,MM_man_measure,'b','LineWidth',lineWidth); hold on;
plot(time,ur5_man_measure,'r','LineWidth',lineWidth);
plot(time,W_measure,'g','LineWidth',1.5); hold off
legend('$\Omega_{p+a}$','$\Omega_{a}$','$\Omega_{MM}$','interpreter','latex','FontSize',labelFontSize)
%legend('\Omega_{p+a}','\Omega_{a}','\Omega_{MM}')
xlabel('time[s]')
%title('Manipulability measure')
grid on

%% Position and orientation error
figure()
plot(time,xi_pos_error(1,:),'LineWidth',lineWidth); hold on
plot(time,xi_pos_error(2,:),'LineWidth',lineWidth); hold on
plot(time,xi_pos_error(3,:),'LineWidth',lineWidth);
xlabel('time[s]','FontName','cmr12')
ylabel('[m]')
legend('$e_{Px}$','$e_{Py}$','$e_{Pz}$','interpreter','latex','FontSize',labelFontSize)
%legend('e_{Px}','e_{Py}','e_{Pz}')
%title('Position error[m]')
grid on

figure()
plot(time,xi_orient_error(1,:),'LineWidth',lineWidth); hold on
plot(time,xi_orient_error(2,:),'LineWidth',lineWidth); hold on
plot(time,xi_orient_error(3,:),'LineWidth',lineWidth);
xlabel('time[s]')
ylabel('[rad]')
legend('$e_{Ox}$','$e_{Oy}$','$e_{Oz}$','interpreter','latex','FontSize',labelFontSize)
%legend('e_{Ox}','e_{Oy}','e_{Oz}')
grid on
%title('Orientation error[rad]')

%% Mobile Platform Trajectory
%Draw the mobile manipulator in the current position
%drawMobilePlatform(q(1,1),q(2,1),q(3,1),xi(1,1),xi(2,1))
figure()
clear PosNF
q1=q(1,:);
q2=q(2,:);
q3=q(3,:);

%In the reference frame
trajH=plot(q1,q2,'LineWidth',lineWidth); hold on; grid on
set(get(get(trajH(1),'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

%Start and Final Positions
plot(q(1,1),q(2,1),'s', 'MarkerEdgeColor','k', 'MarkerFaceColor', red, 'MarkerSize',10);
plot(q(1,end),q(2,end),'o', 'MarkerEdgeColor','k','MarkerFaceColor',[.49 1 .63], 'MarkerSize',10);

legend('Start Pos','Final Pos', ...       
       'interpreter','latex','FontSize',10)
%title('Mobile Platform Trajectory')
xlabel('x[m]');
ylabel('y[m]');

%% Mobile platform velocities
figure()
plot(time,mp_vel(1,:),'LineWidth',lineWidth,'Color',blue); hold on
yline(-dq_limit(1),'--','LineWidth',lineWidth,'Color',blue); 
yline(dq_limit(1),'-.','LineWidth',lineWidth,'Color',blue);

plot(time,mp_vel(2,:),'LineWidth',lineWidth,'Color',red);
yline(-dq_limit(2),'--','LineWidth',lineWidth,'Color',red);
yline(dq_limit(2),'-.','LineWidth',lineWidth,'Color',red);

xlabel('time[s]')
%legend('$v[m/s]$','$v^-$','$v^+$','$\omega[rad/s]$','$\omega^-$','$\omega^+$','interpreter','latex','FontSize',labelFontSize)
ylabel('[m/s]   /   [rad/s]')
legend('$v$','$v^-$','$v^+$', ...
       '$\omega$','$\omega^-$','$\omega^+$', ...
       'interpreter','latex','FontSize',labelFontSize)
%legend('dv[m/s]','dv_{max}','dv_{min}''w[rad/s]')
%title('Mobile platform velocity commands')
grid on

%% Prismatic joint
figure()
plot(time,q(4,:),'LineWidth',lineWidth,'Color',yellow); hold on
yline(q_limit(3,1),'--','LineWidth',lineWidth,'Color',yellow); 
yline(q_limit(3,2),'-.','LineWidth',lineWidth,'Color',yellow); 

plot(time,dq(4,:),'LineWidth',lineWidth,'Color',cyan); 
yline(-dq_limit(3),'--','LineWidth',lineWidth,'Color',cyan); 
yline(dq_limit(3),'-.','LineWidth',lineWidth,'Color',cyan);

xlabel('time[s]')
ylabel('[m]   /   [m/s]')
legend('$z$','$z^-$','$z^+$', ...
       '$\dot{z}$','$\dot{z}^-$','$\dot{z}^+$', ...
       'interpreter','latex','FontSize',labelFontSize)
% legend('$z[m]$','$z^-$','$z^+$', ...
%     '$\dot{z}[m/s]$','$\dot{z}^-$','$\dot{z}^+$', ...
%     'interpreter','latex','FontSize',labelFontSize)
%legend('z[m]','z_{max}','z_{min}','dz[m/s]','dz_{max}','dz_{min}')
%title('Prismatic joint')
grid on

%% UR5 joints position

% Joint 1 and 2
figure()
plot(time,q(5,:),'LineWidth',lineWidth,'Color',blue); hold on
yline(q_limit(4,1),'--','LineWidth',lineWidth,'Color',blue); 
yline(q_limit(4,2),'-.','LineWidth',lineWidth,'Color',blue); 

plot(time,q(6,:),'LineWidth',lineWidth,'Color',red); 
yline(q_limit(5,1),'--','LineWidth',lineWidth,'Color',red); 
yline(q_limit(5,2),'-.','LineWidth',lineWidth,'Color',red);

xlabel('time[s]')
ylabel('[rad]')
legend('$q_{a1}$','$q_{a1}^-$','$q_{a1}^+$', ...
    '$q_{a2}$','$q_{a2}^-$','$q_{a2}^+$', ...
    'interpreter','latex','FontSize',labelFontSize)
grid on

% Joint 3 and 4
figure()
plot(time,q(7,:),'LineWidth',lineWidth,'Color',yellow); hold on
yline(q_limit(6,1),'--','LineWidth',lineWidth,'Color',yellow); 
yline(q_limit(6,2),'-.','LineWidth',lineWidth,'Color',yellow); 

plot(time,q(8,:),'LineWidth',lineWidth,'Color',purple); 
yline(q_limit(7,1),'--','LineWidth',lineWidth,'Color',purple); 
yline(q_limit(7,2),'-.','LineWidth',lineWidth,'Color',purple);

xlabel('time[s]')
ylabel('[rad]')
legend('$q_{a3}$','$q_{a3}^-$','$q_{a3}^+$', ...
    '$q_{a4}$','$q_{a4}^-$','$q_{a4}^+$', ...
    'interpreter','latex','FontSize',labelFontSize)
grid on

% Joint 5 and 6
figure()
plot(time,q(9,:),'LineWidth',lineWidth,'Color',green); hold on
yline(q_limit(8,1),'--','LineWidth',lineWidth,'Color',green); 
yline(q_limit(8,2),'-.','LineWidth',lineWidth,'Color',green); 

plot(time,q(10,:),'LineWidth',lineWidth,'Color',cyan); 
yline(q_limit(9,1),'--','LineWidth',lineWidth,'Color',cyan); 
yline(q_limit(9,2),'-.','LineWidth',lineWidth,'Color',cyan);

xlabel('time[s]')
ylabel('[rad]')
legend('$q_{a5}$','$q_{a5}^-$','$q_{a5}^+$', ...
    '$q_{a6}$','$q_{a6}^-$','$q_{a6}^+$', ...
    'interpreter','latex','FontSize',labelFontSize)
grid on

%% UR5 joints velocities
 
%%Same plot for all the joints
figure()
plot(time,dq(5,:),'LineWidth',lineWidth); hold on
plot(time,dq(6,:),'LineWidth',lineWidth); hold on
plot(time,dq(7,:),'LineWidth',lineWidth); hold on
plot(time,dq(8,:),'LineWidth',lineWidth); hold on
plot(time,dq(9,:),'LineWidth',lineWidth); hold on
plot(time,dq(10,:),'LineWidth',lineWidth); hold on
xlabel('time[s]')
ylabel('[rad/s]')
legend('$\dot{q}_{a1}$','$\dot{q}_{a2}$','$\dot{q}_{a3}$', ...
    '$\dot{q}_{a4}$','$\dot{q}_{a5}$','$\dot{q}_{a6}$', ...
    'interpreter','latex','FontSize',labelFontSize)
grid on

%% Elbow and wrist collision distance
figure()
plot(time,dist_elbow,'LineWidth',lineWidth); hold on
%yline(0.0,'--','LineWidth',lineWidth,'Color',blue); 

plot(time,dist_wrist,'LineWidth',lineWidth);
%yline(0.0,'--','LineWidth',lineWidth,'Color',red); 

plot(time,wrist_pos(3,:),'LineWidth',lineWidth);

legend('$d_{elbow}$','$d_{wrist}$','$h_{wrist}$', ...
       'interpreter','latex','FontSize',labelFontSize)

xlabel('time[s]')
ylabel('[m]')
grid on

