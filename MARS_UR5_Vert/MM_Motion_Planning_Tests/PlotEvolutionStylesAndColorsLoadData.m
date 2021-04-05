close all
clear all

addpath '3DPlots'

%Open a dialog box to load the motion data
uiopen();

%Plots properties
colors = distinguishableColors(6);
blue=colors(1,:);
red=colors(2,:);
green=colors(3,:);
black=colors(4,:);
pink=colors(5,:);
yellow=colors(6,:);
labelFontSize=14;
lineWidth=1.4;
limLineWidth = 1.8;

set(0,'defaulttextinterpreter','latex')
set(0,'defaulttextfontname', 'Times')
set(0,'defaulttextfontsize',16)

set(0, 'defaultAxesTickLabelInterpreter','latex');
set(0, 'defaultLegendInterpreter','latex');
set(0,'defaultaxesfontsize',16)
set(0,'defaultaxesfontname', 'Times')

plots_end_time = time(end);
N = length(time);
spc = floor(N/15);
markerIdx = 8:spc:(N-1);
markerSize = 8;

%% Mobile Platform Trajectory
figure()
clear PosNF
q1=q(1,:);
q2=q(2,:);
q3=q(3,:);

%Plot trajectory
trajH=plot(q1,q2,'--','LineWidth',lineWidth); hold on; grid on
%set(get(get(trajH(1),'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

%Plot triangles
for i = 1:150:length(time)
    tri = genTrianglePoints(q1(i), q2(i), q3(i), 0.25);
    triStartH = plot(tri(1,:), tri(2,:),'LineWidth',lineWidth,'Color', blue);
    set(get(get(triStartH(1),'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
end

%Final triangle
tri = genTrianglePoints(q1(end), q2(end), q3(end), 0.25);
triEndH = plot(tri(1,:), tri(2,:),'LineWidth',lineWidth,'Color', blue);
set(get(get(triEndH(1),'Annotation'),'LegendInformation'),'IconDisplayStyle','off');    

%Start and Final Positions
plot(q(1,1),q(2,1),'o', 'MarkerEdgeColor','k', 'MarkerFaceColor', green, 'MarkerSize',8);
plot(q(1,end),q(2,end),'s', 'MarkerEdgeColor','k','MarkerFaceColor', red, 'MarkerSize',8);

xlim([-2 2])
ylim([-2 2])

% set(gca,'xtick',-2.5:0.5:2.5)
% set(gca,'ytick',-2.5:0.5:2.5)

legend('Traj','Start','End', ...       
       'interpreter','latex','FontSize',10)
%title('Mobile Platform Trajectory')
xlabel('$x(m)$','interpreter','latex','FontSize',labelFontSize);
ylabel('$y(m)$','interpreter','latex','FontSize',labelFontSize);
set(gcf, 'Position',  [200, 500, 490, 310])

%% Manipulability plots
figure()
% MM_man_max = 2.6141267;
% ur5_man_max = 0.1192;
% W_measure_max = 0.2626694;
% MM_man_measure = MM_man_measure / MM_man_max;
% ur5_man_measure = ur5_man_measure / ur5_man_max;
% W_measure = W_measure / W_measure_max;
% plot(time,MM_man_measure,'b','LineWidth',lineWidth,'Color',blue,'Marker','o','MarkerIndices',markerIdx,'MarkerSize',markerSize); hold on;
% plot(time,ur5_man_measure,'r','LineWidth',lineWidth,'Color',red,'Marker','d','MarkerIndices',markerIdx,'MarkerSize',markerSize);
% plot(time,W_measure,'g','LineWidth',lineWidth,'Color',green,'Marker','square','MarkerIndices',markerIdx,'MarkerSize',markerSize); hold off
% legend('$\Omega_{p+a}$','$\Omega_{a}$','$\Omega_{MM}$','interpreter','latex','FontSize',labelFontSize);

MM_man_max = max(MM_man_measure);
ur5_man_max = max(ur5_man_measure);
MM_man_measure = MM_man_measure / MM_man_max;
ur5_man_measure = ur5_man_measure / ur5_man_max;
plot(time,MM_man_measure,'LineWidth',lineWidth,'Color',blue,'LineStyle','-'); hold on;
plot(time,ur5_man_measure,'LineWidth',lineWidth,'Color',red,'LineStyle','--');
legend('$\Omega_{p+a}$','$\Omega_{a}$','interpreter','latex','FontSize',labelFontSize);

xlim([0 plots_end_time])
xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
grid on
set(gcf, 'Position',  [200, 500, 490, 310])

%% Position and orientation error
figure()
markerIdxError = 20:20:(length(time)-1);
h1=plot(time,xi_pos_error(1,:),'LineWidth',1.0,'Color',blue,'Marker','o','MarkerIndices',markerIdx,'MarkerSize',markerSize); hold on
h2=plot(time,xi_pos_error(2,:),'LineWidth',1.0,'Color',red,'Marker','d','MarkerIndices',markerIdx,'MarkerSize',markerSize);
h3=plot(time,xi_pos_error(3,:),'LineWidth',1.0,'Color',green,'Marker','square','MarkerIndices',markerIdx,'MarkerSize',markerSize); hold off
uistack(h3,'top')
uistack(h2,'top')
uistack(h1,'top')
legend([h1 h2 h3],'$e_{Px}$','$e_{Py}$','$e_{Pz}$','interpreter','latex','FontSize',labelFontSize)
xlim([0 plots_end_time])
ylim([-0.0005 0.0005])
xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(m)$','interpreter','latex','FontSize',labelFontSize)
grid on
set(gcf, 'Position',  [200, 500, 490, 310])

figure()
h1=plot(time,xi_orient_error(1,:),'LineWidth',1.0,'Color',blue,'Marker','o','MarkerIndices',markerIdx,'MarkerSize',markerSize); hold on
h2=plot(time,xi_orient_error(2,:),'LineWidth',1.0,'Color',red,'Marker','d','MarkerIndices',markerIdx,'MarkerSize',markerSize); 
h3=plot(time,xi_orient_error(3,:),'LineWidth',1.0,'Color',green,'Marker','square','MarkerIndices',markerIdx,'MarkerSize',markerSize); hold off
uistack(h3,'top')
uistack(h2,'top')
uistack(h1,'top')
legend([h1 h2 h3],'$e_{Ox}$','$e_{Oy}$','$e_{Oz}$','interpreter','latex','FontSize',labelFontSize)
xlim([0 plots_end_time])
ylim([-0.0005 0.0005])
xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(rad)$','interpreter','latex','FontSize',labelFontSize)
grid on
set(gcf, 'Position',  [200, 500, 490, 310])

%% Mobile platform velocities
figure()
plot(time,mp_vel(1,:),'LineWidth',lineWidth,'Color',blue,'Marker','o','MarkerIndices',markerIdx,'MarkerSize',markerSize); hold on
yline(-dq_limit(1),'-','LineWidth',limLineWidth,'Color',blue); 
yline(dq_limit(1),'--','LineWidth',limLineWidth,'Color',blue);

plot(time,mp_vel(2,:),'LineWidth',lineWidth,'Color',red,'Marker','d','MarkerIndices',markerIdx,'MarkerSize',markerSize); hold on
yline(-dq_limit(2),':','LineWidth',limLineWidth,'Color',red);
yline(dq_limit(2),'-.','LineWidth',limLineWidth,'Color',red);

xlim([0 plots_end_time])

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$v(m/s)$   ,  $\omega$(rad/s)','Interpreter','latex')
legend('$v$','$v^-$','$v^+$', ...
       '$\omega$','$\omega^-$','$\omega^+$', ...
       'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
grid on
set(gcf, 'Position',  [200, 500, 490, 310])

%% Prismatic joint
figure()
plot(time,q(4,:),'LineWidth',lineWidth,'Color',blue,'Marker','o','MarkerIndices',markerIdx,'MarkerSize',markerSize); hold on
yline(q_limit(3,1),'-','LineWidth',limLineWidth,'Color',blue); 
yline(q_limit(3,2),'--','LineWidth',limLineWidth,'Color',blue); 

plot(time,dq(4,:),'LineWidth',lineWidth,'Color',red,'Marker','d','MarkerIndices',markerIdx,'MarkerSize',markerSize); 
yline(-dq_limit(3),':','LineWidth',limLineWidth,'Color',red); 
yline(dq_limit(3),'-.','LineWidth',limLineWidth,'Color',red);

xlim([0 plots_end_time])
ylim([-0.1 0.3])

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$z$(m)   ,   $\dot{z}$(m/s)', 'interpreter','latex')
legend('$z$','$z^-$','$z^+$', ...
       '$\dot{z}$','$\dot{z}^-$','$\dot{z}^+$', ...
       'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
grid on
set(gcf, 'Position',  [200, 500, 490, 310])

%% UR5 joints position

% Joint 1 and 2
figure()
plot(time,q(5,:),'LineWidth',lineWidth,'Color',blue,'Marker','o','MarkerIndices',markerIdx,'MarkerSize',markerSize); hold on
yline(q_limit(4,1),'-','LineWidth',limLineWidth,'Color',blue); 
yline(q_limit(4,2),'--','LineWidth',limLineWidth,'Color',blue); 

plot(time,q(6,:),'LineWidth',lineWidth,'Color',red,'Marker','d','MarkerIndices',markerIdx,'MarkerSize',markerSize); 
yline(q_limit(5,1),':','LineWidth',limLineWidth,'Color',red); 
yline(q_limit(5,2),'-.','LineWidth',limLineWidth,'Color',red);

xlim([0 plots_end_time])

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(rad)$','interpreter','latex','FontSize',labelFontSize)
legend('$q_{a1}$','$q_{a1}^-$','$q_{a1}^+$', ...
    '$q_{a2}$','$q_{a2}^-$','$q_{a2}^+$', ...
    'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
grid on
set(gcf, 'Position',  [200, 500, 490, 310])

% Joint 3 and 4
figure()
plot(time,q(7,:),'LineWidth',lineWidth,'Color',blue,'Marker','o','MarkerIndices',markerIdx,'MarkerSize',markerSize); hold on
yline(q_limit(6,1),'-','LineWidth',limLineWidth,'Color',blue); 
yline(q_limit(6,2),'--','LineWidth',limLineWidth,'Color',blue); 

plot(time,q(8,:),'LineWidth',lineWidth,'Color',red,'Marker','d','MarkerIndices',markerIdx,'MarkerSize',markerSize); 
yline(q_limit(7,1),':','LineWidth',limLineWidth,'Color',red); 
yline(q_limit(7,2),'-.','LineWidth',limLineWidth,'Color',red);

xlim([0 plots_end_time])
ylim([-6.5 6.5])

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(rad)$','interpreter','latex','FontSize',labelFontSize)
legend('$q_{a3}$','$q_{a3}^-$','$q_{a3}^+$', ...
    '$q_{a4}$','$q_{a4}^-$','$q_{a4}^+$', ...
    'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
grid on
set(gcf, 'Position',  [200, 500, 490, 310])

% Joint 5 and 6
figure()
plot(time,q(9,:),'LineWidth',lineWidth,'Color',blue,'Marker','o','MarkerIndices',markerIdx,'MarkerSize',markerSize); hold on
yline(q_limit(8,1),'-','LineWidth',limLineWidth,'Color',blue); 
yline(q_limit(8,2),'--','LineWidth',limLineWidth,'Color',blue); 

plot(time,q(10,:),'LineWidth',lineWidth,'Color',red,'Marker','d','MarkerIndices',markerIdx,'MarkerSize',markerSize); 
yline(q_limit(9,1),':','LineWidth',limLineWidth,'Color',red);
yline(q_limit(9,2),'-.','LineWidth',limLineWidth,'Color',red);

xlim([0 plots_end_time])
ylim([-6.5 6.5])

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(rad)$','interpreter','latex','FontSize',labelFontSize)
legend('$q_{a5}$','$q_{a5}^-$','$q_{a5}^+$', ...
    '$q_{a6}$','$q_{a6}^-$','$q_{a6}^+$', ...
    'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
grid on
set(gcf, 'Position',  [200, 500, 490, 310])

%% UR5 joints velocities
 
%%Same plot for all the joints
figure()
h1=plot(time,dq(5,:),'LineWidth',lineWidth,'Color',blue,'Marker','o','MarkerIndices',markerIdx,'MarkerSize',markerSize); hold on
h2=plot(time,dq(6,:),'LineWidth',lineWidth,'Color',red,'Marker','d','MarkerIndices',markerIdx,'MarkerSize',markerSize);  hold on
h3=plot(time,dq(7,:),'LineWidth',lineWidth,'Color',green,'Marker','square','MarkerIndices',markerIdx,'MarkerSize',markerSize); hold on
h4=plot(time,dq(8,:),'LineWidth',lineWidth,'Color',black,'Marker','p','MarkerIndices',markerIdx); hold on
h5=plot(time,dq(9,:),'LineWidth',lineWidth,'Color',pink,'Marker','v','MarkerIndices',markerIdx,'MarkerSize',markerSize); hold on
h6=plot(time,dq(10,:),'LineWidth',lineWidth,'Color',yellow,'Marker','x','MarkerIndices',markerIdx,'MarkerSize',markerSize); hold off
uistack(h6,'top')
uistack(h5,'top')
uistack(h4,'top')
uistack(h3,'top')
uistack(h2,'top')
uistack(h1,'top')
legend([h1 h2 h3 h4 h5 h6],'$\dot{q}_{a1}$','$\dot{q}_{a2}$','$\dot{q}_{a3}$', ...
    '$\dot{q}_{a4}$','$\dot{q}_{a5}$','$\dot{q}_{a6}$', ...
    'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)

xlim([0 plots_end_time])
%ylim([-1 1])
xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(rad/s)$','interpreter','latex','FontSize',labelFontSize)
grid on
set(gcf, 'Position',  [200, 500, 490, 310])
 
%% Elbow and wrist collision distance
figure()
plot(time,dist_elbow,'LineWidth',lineWidth,'Color',blue,'LineStyle','-'); hold on
plot(time,dist_wrist,'LineWidth',lineWidth,'Color',red,'LineStyle','--');

xlim([0 plots_end_time])
legend('$d_{elbow}$','$d_{wrist}$', ...
       'interpreter','latex','FontSize',labelFontSize)

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(m)$','interpreter','latex','FontSize',labelFontSize)
grid on
set(gcf, 'Position',  [200, 500, 490, 310])

% %% End effector motion
disp('Creating 3D plot')
%Show the end effector motion in 3D
figure()
Rd=zeros(4,4,length(time));
%Form the T6Traj matrix
for i=1:length(xi)
   Rd(:,4,i)=[xi(1,i);xi(2,i);xi(3,i);1];
   Rd(1:3,1:3,i)=quatToRotMat(xi(4:7,i));
end
plotEndEffectorMotion2(Rd,0.4,600,8)
set(gcf, 'Position',  [200, 500, 490, 310])
