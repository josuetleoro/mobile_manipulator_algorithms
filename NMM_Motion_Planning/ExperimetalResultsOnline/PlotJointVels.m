close all

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

plots_end_time = time_jstates(end);

%% Mobile platform velocities
figure()
plot(time_mp_vel,mp_vel(1,:),'LineWidth',lineWidth,'Color',blue); hold on
yline(-dq_limit(1),'--','LineWidth',lineWidth,'Color',blue); 
yline(dq_limit(1),'-.','LineWidth',lineWidth,'Color',blue);

plot(time_mp_vel,mp_vel(2,:),'LineWidth',lineWidth,'Color',red);
yline(-dq_limit(2),'--','LineWidth',lineWidth,'Color',red);
yline(dq_limit(2),'-.','LineWidth',lineWidth,'Color',red);

xlim([0 plots_end_time])

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
%legend('$v(m/s)$','$v^-$','$v^+$','$\omega(rad/s)$','$\omega^-$','$\omega^+$','interpreter','latex','FontSize',labelFontSize)
ylabel('$\mbox{v}(m/s)$   ,  $\omega$(rad/s)','Interpreter','latex')
legend('$\mbox{v}$','$\mbox{v}^-$','$\mbox{v}^+$', ...
       '$\omega$','$\omega^-$','$\omega^+$', ...
       'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
grid on
set(gcf, 'Position',  [500, 500, 490, 310])
%title('Mobile platform velocity commands')


%% Prismatic joint
figure()
plot(time_jstates,q(4,:),'LineWidth',lineWidth,'Color',yellow); hold on
yline(q_limit(3,1),'--','LineWidth',lineWidth,'Color',yellow); 
yline(q_limit(3,2),'-.','LineWidth',lineWidth,'Color',yellow); 

plot(time_jstates,dq(4,:),'LineWidth',lineWidth,'Color',cyan); 
yline(-dq_limit(3),'--','LineWidth',lineWidth,'Color',cyan); 
yline(dq_limit(3),'-.','LineWidth',lineWidth,'Color',cyan);

xlim([0 plots_end_time])
ylim([-0.2 0.3])

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$z$(m)   ,   $\dot{z}$(m/s)', 'interpreter','latex')
legend('$z$','$z^-$','$z^+$', ...
       '$\dot{z}$','$\dot{z}^-$','$\dot{z}^+$', ...
       'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
% legend('$z(m)$','$z^-$','$z^+$', ...
%     '$\dot{z}(m/s)$','$\dot{z}^-$','$\dot{z}^+$', ...
%     'interpreter','latex','FontSize',labelFontSize)
%legend('z(m)','z_{max}','z_{min}','dz(m/s)','dz_{max}','dz_{min}')
grid on
set(gcf, 'Position',  [500, 500, 490, 310])
%title('Prismatic joint')

%% UR5 joints position

% Joint 1 and 2
figure()
plot(time_jstates,q(5,:),'LineWidth',lineWidth,'Color',blue); hold on
yline(q_limit(4,1),'--','LineWidth',lineWidth,'Color',blue); 
yline(q_limit(4,2),'-.','LineWidth',lineWidth,'Color',blue); 

plot(time_jstates,q(6,:),'LineWidth',lineWidth,'Color',red); 
yline(q_limit(5,1),'--','LineWidth',lineWidth,'Color',red); 
yline(q_limit(5,2),'-.','LineWidth',lineWidth,'Color',red);

xlim([0 plots_end_time])

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(rad)$','interpreter','latex','FontSize',labelFontSize)
legend('$q_{a1}$','$q_{a1}^-$','$q_{a1}^+$', ...
    '$q_{a2}$','$q_{a2}^-$','$q_{a2}^+$', ...
    'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
grid on
set(gcf, 'Position',  [500, 500, 490, 310])

% Joint 3 and 4
figure()
plot(time_jstates,q(7,:),'LineWidth',lineWidth,'Color',yellow); hold on
yline(q_limit(6,1),'--','LineWidth',lineWidth,'Color',yellow); 
yline(q_limit(6,2),'-.','LineWidth',lineWidth,'Color',yellow); 

plot(time_jstates,q(8,:),'LineWidth',lineWidth,'Color',purple); 
yline(q_limit(7,1),'--','LineWidth',lineWidth,'Color',purple); 
yline(q_limit(7,2),'-.','LineWidth',lineWidth,'Color',purple);

xlim([0 plots_end_time])

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(rad)$','interpreter','latex','FontSize',labelFontSize)
legend('$q_{a3}$','$q_{a3}^-$','$q_{a3}^+$', ...
    '$q_{a4}$','$q_{a4}^-$','$q_{a4}^+$', ...
    'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
grid on
set(gcf, 'Position',  [500, 500, 490, 310])

% Joint 5 and 6
figure()
plot(time_jstates,q(9,:),'LineWidth',lineWidth,'Color',green); hold on
yline(q_limit(8,1),'--','LineWidth',lineWidth,'Color',green); 
yline(q_limit(8,2),'-.','LineWidth',lineWidth,'Color',green); 

plot(time_jstates,q(10,:),'LineWidth',lineWidth,'Color',cyan); 
yline(q_limit(9,1),'--','LineWidth',lineWidth,'Color',cyan); 
yline(q_limit(9,2),'-.','LineWidth',lineWidth,'Color',cyan);

xlim([0 plots_end_time])

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(rad)$','interpreter','latex','FontSize',labelFontSize)
legend('$q_{a5}$','$q_{a5}^-$','$q_{a5}^+$', ...
    '$q_{a6}$','$q_{a6}^-$','$q_{a6}^+$', ...
    'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
grid on
set(gcf, 'Position',  [500, 500, 490, 310])

%% UR5 joints velocities
 
%%Same plot for all the joints
figure()
h1=plot(time_jstates,dq(5,:),'LineWidth',lineWidth,'Color',blue); hold on
h2=plot(time_jstates,dq(6,:),'LineWidth',lineWidth,'Color',red);  hold on
h3=plot(time_jstates,dq(7,:),'LineWidth',lineWidth,'Color',yellow); hold on
h4=plot(time_jstates,dq(8,:),'LineWidth',lineWidth,'Color',purple); hold on
h5=plot(time_jstates,dq(9,:),'LineWidth',lineWidth,'Color',green); hold on
h6=plot(time_jstates,dq(10,:),'LineWidth',lineWidth,'Color',cyan); hold off
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
xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(rad/s)$','interpreter','latex','FontSize',labelFontSize)
grid on
set(gcf, 'Position',  [500, 500, 490, 310])
