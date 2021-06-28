close all
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
labelFontSize=14;
lineWidth=1.8;

set(0,'defaulttextinterpreter','latex')
set(0,'defaulttextfontname', 'Times')
set(0,'defaulttextfontsize',16)

set(0, 'defaultAxesTickLabelInterpreter','latex');
set(0, 'defaultLegendInterpreter','latex');
set(0,'defaultaxesfontsize',16)
set(0,'defaultaxesfontname', 'Times')

%% Manipulability plots
figure()
plot(time,MM_man_measure,'b','LineWidth',lineWidth); hold on;
plot(time,ur5_man_measure,'r','LineWidth',lineWidth);
legend('$\Omega_{p+a}$','$\Omega_{a}$','interpreter','latex','FontSize',labelFontSize)
xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
%title('Manipulability measure')
grid on

%% Position and orientation error
figure()
plot(time(1:k),xi_pos_error(1,1:k),'LineWidth',lineWidth); hold on
plot(time,xi_pos_error(2,1:k),'LineWidth',lineWidth); hold on
plot(time,xi_pos_error(3,1:k),'LineWidth',lineWidth);
xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(m)$','interpreter','latex','FontSize',labelFontSize)
legend('$e_{Px}$','$e_{Py}$','$e_{Pz}$','interpreter','latex','FontSize',labelFontSize)
%title('Position error(m)')
grid on

figure()
plot(time(1:k),xi_orient_error(1,1:k),'LineWidth',lineWidth); hold on
plot(time(1:k),xi_orient_error(2,1:k),'LineWidth',lineWidth); hold on
plot(time(1:k),xi_orient_error(3,1:k),'LineWidth',lineWidth);
xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(rad)$','interpreter','latex','FontSize',labelFontSize)
legend('$e_{Ox}$','$e_{Oy}$','$e_{Oz}$','interpreter','latex','FontSize',labelFontSize)
grid on
%title('Orientation error(rad)')

%% Mobile Platform Trajectory
%Draw the mobile manipulator in the current position
%drawMobilePlatform(q(1,1),q(2,1),q(3,1),xi(1,1),xi(2,1))
figure()
clear PosNF
q1=q(1,1:k);
q2=q(2,1:k);
q3=q(3,1:k);

%In the reference frame
trajH=plot(q1,q2,'LineWidth',lineWidth); hold on; grid on
set(get(get(trajH(1),'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

%Start and Final Positions
plot(q(1,1),q(2,1),'s', 'MarkerEdgeColor','k', 'MarkerFaceColor', red, 'MarkerSize',10);
plot(q(1,end),q(2,end),'o', 'MarkerEdgeColor','k','MarkerFaceColor',[.49 1 .63], 'MarkerSize',10);

legend('Start Pos','Final Pos', ...       
       'interpreter','latex','FontSize',10)
%title('Mobile Platform Trajectory')
xlabel('$x(m)$','interpreter','latex','FontSize',labelFontSize);
ylabel('$y(m)$','interpreter','latex','FontSize',labelFontSize);

%% Mobile platform velocities
figure()
plot(time(1:k),mp_vel(1,1:k),'LineWidth',lineWidth,'Color',blue); hold on
yline(-dq_limit(1),'--','LineWidth',lineWidth,'Color',blue); 
yline(dq_limit(1),'-.','LineWidth',lineWidth,'Color',blue);

plot(time,mp_vel(2,1:k),'LineWidth',lineWidth,'Color',red);
yline(-dq_limit(2),'--','LineWidth',lineWidth,'Color',red);
yline(dq_limit(2),'-.','LineWidth',lineWidth,'Color',red);

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
%legend('$v(m/s)$','$v^-$','$v^+$','$\omega(rad/s)$','$\omega^-$','$\omega^+$','interpreter','latex','FontSize',labelFontSize)
ylabel('$v(m/s)$   ,  $\omega$(rad/s)','Interpreter','latex')
legend('$v$','$v^-$','$v^+$', ...
       '$\omega$','$\omega^-$','$\omega^+$', ...
       'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
grid on

%% Prismatic joint
figure()
plot(time(1:k),q(4,1:k),'LineWidth',lineWidth,'Color',yellow); hold on
yline(q_limit(3,1),'--','LineWidth',lineWidth,'Color',yellow); 
yline(q_limit(3,2),'-.','LineWidth',lineWidth,'Color',yellow); 

plot(time(1:k),dq(4,1:k),'LineWidth',lineWidth,'Color',cyan); 
yline(-dq_limit(3),'--','LineWidth',lineWidth,'Color',cyan); 
yline(dq_limit(3),'-.','LineWidth',lineWidth,'Color',cyan);

ylim([-0.3 0.3])

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$z$(m)   ,   $\dot{z}$(m/s)', 'interpreter','latex')
legend('$z$','$z^-$','$z^+$', ...
       '$\dot{z}$','$\dot{z}^-$','$\dot{z}^+$', ...
       'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
grid on

%% UR5 joints position

% Joint 1 and 2
figure()
plot(time(1:k),q(5,1:k),'LineWidth',lineWidth,'Color',blue); hold on
yline(q_limit(4,1),'--','LineWidth',lineWidth,'Color',blue); 
yline(q_limit(4,2),'-.','LineWidth',lineWidth,'Color',blue); 

plot(time(1:k),q(6,1:k),'LineWidth',lineWidth,'Color',red); 
yline(q_limit(5,1),'--','LineWidth',lineWidth,'Color',red); 
yline(q_limit(5,2),'-.','LineWidth',lineWidth,'Color',red);

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(rad)$','interpreter','latex','FontSize',labelFontSize)
legend('$q_{a1}$','$q_{a1}^-$','$q_{a1}^+$', ...
    '$q_{a2}$','$q_{a2}^-$','$q_{a2}^+$', ...
    'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
grid on

% Joint 3 and 4
figure()
plot(time(1:k),q(7,1:k),'LineWidth',lineWidth,'Color',yellow); hold on
yline(q_limit(6,1),'--','LineWidth',lineWidth,'Color',yellow); 
yline(q_limit(6,2),'-.','LineWidth',lineWidth,'Color',yellow); 

plot(time(1:k),q(8,1:k),'LineWidth',lineWidth,'Color',purple); 
yline(q_limit(7,1),'--','LineWidth',lineWidth,'Color',purple); 
yline(q_limit(7,2),'-.','LineWidth',lineWidth,'Color',purple);

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(rad)$','interpreter','latex','FontSize',labelFontSize)
legend('$q_{a3}$','$q_{a3}^-$','$q_{a3}^+$', ...
    '$q_{a4}$','$q_{a4}^-$','$q_{a4}^+$', ...
    'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
grid on

% Joint 5 and 6
figure()
plot(time(1:k),q(9,1:k),'LineWidth',lineWidth,'Color',green); hold on
yline(q_limit(8,1),'--','LineWidth',lineWidth,'Color',green); 
yline(q_limit(8,2),'-.','LineWidth',lineWidth,'Color',green); 

plot(time(1:k),q(10,1:k),'LineWidth',lineWidth,'Color',cyan); 
yline(q_limit(9,1),'--','LineWidth',lineWidth,'Color',cyan); 
yline(q_limit(9,2),'-.','LineWidth',lineWidth,'Color',cyan);

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(rad)$','interpreter','latex','FontSize',labelFontSize)
legend('$q_{a5}$','$q_{a5}^-$','$q_{a5}^+$', ...
    '$q_{a6}$','$q_{a6}^-$','$q_{a6}^+$', ...
    'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
grid on

%% UR5 joints velocities
 
%%Same plot for all the joints
figure()
plot(time(1:k),dq(5,1:k),'LineWidth',lineWidth); hold on
plot(time(1:k),dq(6,1:k),'LineWidth',lineWidth); hold on
plot(time(1:k),dq(7,1:k),'LineWidth',lineWidth); hold on
plot(time(1:k),dq(8,1:k),'LineWidth',lineWidth); hold on
plot(time(1:k),dq(9,1:k),'LineWidth',lineWidth); hold on
plot(time(1:k),dq(10,1:k),'LineWidth',lineWidth); hold on
xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(rad/s)$','interpreter','latex','FontSize',labelFontSize)
legend('$\dot{q}_{a1}$','$\dot{q}_{a2}$','$\dot{q}_{a3}$', ...
    '$\dot{q}_{a4}$','$\dot{q}_{a5}$','$\dot{q}_{a6}$', ...
    'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
grid on

%% Elbow and wrist collision distance
figure()
plot(time(1:k),dist_elbow(1:k),'LineWidth',lineWidth); hold on
%yline(0.0,'--','LineWidth',lineWidth,'Color',blue); 

plot(time(1:k),dist_wrist(1:k),'LineWidth',lineWidth);
%yline(0.0,'--','LineWidth',lineWidth,'Color',red); 

plot(time(1:k),wrist_pos(3,1:k),'LineWidth',lineWidth);

legend('$d_{elbow}$','$d_{wrist}$','$h_{wrist}$', ...
       'interpreter','latex','FontSize',labelFontSize)

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(m)$','interpreter','latex','FontSize',labelFontSize)
grid on

