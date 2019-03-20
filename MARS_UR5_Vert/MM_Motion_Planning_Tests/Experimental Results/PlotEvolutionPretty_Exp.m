close all
% clear all
% %Open a dialog box to look for the motion data
% uiopen();
% % Show the final position
% xi(:,end)

%Tf=MARS.forwardKin(qf);

JointConstraints

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
% figure()
% plot(time,MM_man_measure,'b','LineWidth',lineWidth); hold on;
% plot(time,ur5_man_measure,'r','LineWidth',lineWidth);
% plot(time,W_measure,'g','LineWidth',1.5); hold off
% legend('$\Omega_{p+a}$','$\Omega_{a}$','$\Omega_{MM}$','interpreter','latex','FontSize',labelFontSize)
% %legend('\Omega_{p+a}','\Omega_{a}','\Omega_{MM}')
% xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
% %title('Manipulability measure')
% grid on

%% Position and orientation error
% figure()
% plot(time_tf,ee_xi(1,:),'LineWidth',lineWidth); hold on
% plot(time_tf,ee_xi(2,:),'LineWidth',lineWidth); hold on
% plot(time_tf,ee_xi(3,:),'LineWidth',lineWidth);
% xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
% ylabel('$(m)$','interpreter','latex','FontSize',labelFontSize)
% legend('$x$','$y$','$z$','interpreter','latex','FontSize',labelFontSize)
% title('Position')
% grid on

figure()
plot(time_tf,xi_pos_error(1,:),'LineWidth',lineWidth); hold on
plot(time_tf,xi_pos_error(2,:),'LineWidth',lineWidth); hold on
plot(time_tf,xi_pos_error(3,:),'LineWidth',lineWidth);
xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(m)$','interpreter','latex','FontSize',labelFontSize)
legend('$e_{Px}$','$e_{Py}$','$e_{Pz}$','interpreter','latex','FontSize',labelFontSize)
grid on
title('Position error(m)')
fprintf('\nFinal Position Error:');
xi_pos_error(:,end)'
fprintf('Pos norm error: %fm\n',norm(xi_pos_error(:,end))');

figure()
plot(time_tf,xi_orient_error(1,:),'LineWidth',lineWidth); hold on
plot(time_tf,xi_orient_error(2,:),'LineWidth',lineWidth); hold on
plot(time_tf,xi_orient_error(3,:),'LineWidth',lineWidth);
xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(rad)$','interpreter','latex','FontSize',labelFontSize)
legend('$e_{Ox}$','$e_{Oy}$','$e_{Oz}$','interpreter','latex','FontSize',labelFontSize)
grid on
title('Orientation error(rad)')
fprintf('\nFinal Orientation Error');
xi_orient_error(:,end)'
fprintf('Orientation norm error: %f\n',norm(xi_orient_error(:,end))');

%% Mobile Platform Trajectory
figure()

%In the reference frame
trajH=plot(mp(1,:),mp(2,:),'LineWidth',lineWidth); hold on; grid on
set(get(get(trajH(1),'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

%Start and Final Positions
plot(mp(1,1),mp(2,1),'s', 'MarkerEdgeColor','k', 'MarkerFaceColor', red, 'MarkerSize',10);
plot(mp(1,end),mp(2,end),'o', 'MarkerEdgeColor','k','MarkerFaceColor',[.49 1 .63], 'MarkerSize',10);

legend('Start Pos','Final Pos', ...       
       'interpreter','latex','FontSize',10)
%title('Mobile Platform Trajectory')
xlabel('$x(m)$','interpreter','latex','FontSize',labelFontSize);
ylabel('$y(m)$','interpreter','latex','FontSize',labelFontSize);

%% Mobile platform velocities
figure()
plot(time_mp_vel,mp_vel(1,:),'LineWidth',lineWidth,'Color',blue); hold on
yline(-dq_limit(1),'--','LineWidth',lineWidth,'Color',blue); 
yline(dq_limit(1),'-.','LineWidth',lineWidth,'Color',blue);

plot(time_mp_vel,mp_vel(2,:),'LineWidth',lineWidth,'Color',red);
yline(-dq_limit(2),'--','LineWidth',lineWidth,'Color',red);
yline(dq_limit(2),'-.','LineWidth',lineWidth,'Color',red);

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$v(m/s)$   ,  $\omega$(rad/s)','Interpreter','latex')
legend('$v$','$v^-$','$v^+$', ...
       '$\omega$','$\omega^-$','$\omega^+$', ...
       'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
title('Mobile platform velocity commands')
grid on

%% Prismatic joint
figure()
plot(time_jstates,z_pj,'LineWidth',lineWidth,'Color',yellow); hold on
yline(q_limit(3,1),'--','LineWidth',lineWidth,'Color',yellow); 
yline(q_limit(3,2),'-.','LineWidth',lineWidth,'Color',yellow); 

plot(time_jstates,dz_pj,'LineWidth',lineWidth,'Color',cyan); 
yline(-dq_limit(3),'--','LineWidth',lineWidth,'Color',cyan); 
yline(dq_limit(3),'-.','LineWidth',lineWidth,'Color',cyan);

ylim([-0.3 0.3])

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$z$(m)   ,   $\dot{z}$(m/s)', 'interpreter','latex')
legend('$z$','$z^-$','$z^+$', ...
       '$\dot{z}$','$\dot{z}^-$','$\dot{z}^+$', ...
       'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
title('Prismatic joint')
grid on

%% UR5 joints position

% Joint 1 and 2
figure()
plot(time_jstates,qa(1,:),'LineWidth',lineWidth,'Color',blue); hold on
yline(q_limit(4,1),'--','LineWidth',lineWidth,'Color',blue); 
yline(q_limit(4,2),'-.','LineWidth',lineWidth,'Color',blue); 

plot(time_jstates,qa(2,:),'LineWidth',lineWidth,'Color',red); 
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
plot(time_jstates,qa(3,:),'LineWidth',lineWidth,'Color',yellow); hold on
yline(q_limit(6,1),'--','LineWidth',lineWidth,'Color',yellow); 
yline(q_limit(6,2),'-.','LineWidth',lineWidth,'Color',yellow); 

plot(time_jstates,qa(4,:),'LineWidth',lineWidth,'Color',purple); 
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
plot(time_jstates,qa(5,:),'LineWidth',lineWidth,'Color',green); hold on
yline(q_limit(8,1),'--','LineWidth',lineWidth,'Color',green); 
yline(q_limit(8,2),'-.','LineWidth',lineWidth,'Color',green); 

plot(time_jstates,qa(6,:),'LineWidth',lineWidth,'Color',cyan); 
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
plot(time_jstates,dqa(6,:),'LineWidth',lineWidth,'Color',cyan); hold on
plot(time_jstates,dqa(5,:),'LineWidth',lineWidth,'Color',green); 
plot(time_jstates,dqa(4,:),'LineWidth',lineWidth,'Color',purple); 
plot(time_jstates,dqa(3,:),'LineWidth',lineWidth,'Color',yellow); 
plot(time_jstates,dqa(2,:),'LineWidth',lineWidth,'Color',red);
plot(time_jstates,dqa(1,:),'LineWidth',lineWidth,'Color',blue);
xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(rad/s)$','interpreter','latex','FontSize',labelFontSize)
legend('$\dot{q}_{a1}$','$\dot{q}_{a2}$','$\dot{q}_{a3}$', ...
    '$\dot{q}_{a4}$','$\dot{q}_{a5}$','$\dot{q}_{a6}$', ...
    'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
grid on

% % Joint 1 and 2
% figure()
% plot(time_jstates,dqa(1,:),'LineWidth',lineWidth,'Color',blue); hold on
% plot(time_jstates,dqa(2,:),'LineWidth',lineWidth,'Color',red); hold on
% xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
% ylabel('$(rad/s)$','interpreter','latex','FontSize',labelFontSize)
% legend('$\dot{q}_{a1}$','$\dot{q}_{a2}$', ...
%     'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
% grid on
% 
% % Joint 3 and 4
% figure()
% plot(time_jstates,dqa(3,:),'LineWidth',lineWidth,'Color',yellow); hold on
% plot(time_jstates,dqa(4,:),'LineWidth',lineWidth,'Color',purple); hold on
% xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
% ylabel('$(rad/s)$','interpreter','latex','FontSize',labelFontSize)
% legend('$\dot{q}_{a3}$', '$\dot{q}_{a4}$', ...
%     'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
% grid on
% 
% % Joint 5 and 6
% figure()
% plot(time_jstates,dqa(5,:),'LineWidth',lineWidth,'Color',green); hold on
% plot(time_jstates,dqa(6,:),'LineWidth',lineWidth,'Color',cyan); hold on
% xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
% ylabel('$(rad/s)$','interpreter','latex','FontSize',labelFontSize)
% legend('$\dot{q}_{a5}$', '$\dot{q}_{a6}$', ...
%     'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
% grid on


% %% Elbow and wrist collision distance
% figure()
% plot(time,dist_elbow,'LineWidth',lineWidth); hold on
% %yline(0.0,'--','LineWidth',lineWidth,'Color',blue); 
% 
% plot(time,dist_wrist,'LineWidth',lineWidth);
% %yline(0.0,'--','LineWidth',lineWidth,'Color',red); 
% 
% plot(time,wrist_pos(3,:),'LineWidth',lineWidth);
% 
% legend('$d_{elbow}$','$d_{wrist}$','$h_{wrist}$', ...
%        'interpreter','latex','FontSize',labelFontSize)
% 
% xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
% ylabel('$(m)$','interpreter','latex','FontSize',labelFontSize)
% grid on

