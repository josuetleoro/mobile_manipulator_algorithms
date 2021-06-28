function PlotJointVelComp(sim_data_file,mp_vel_exp_time,mp_vel_exp,dq_exp_time,dq_exp)

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


load(sim_data_file,'mp_vel','dq','time','dq_limit')
plots_end_time = time(end);

%% Mobile platform velocities
figure()

%Linear velocity
subplot(3,1,1)
plot(mp_vel_exp_time,mp_vel_exp(1,:),'LineWidth',lineWidth,'Color',blue); hold on
plot(time,mp_vel(1,:),'LineWidth',lineWidth,'Color',red); hold on
yline(-dq_limit(1),'--','LineWidth',lineWidth,'Color',blue); 
yline(dq_limit(1),'-.','LineWidth',lineWidth,'Color',blue);

xlim([0 plots_end_time])

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$v(m/s)$','Interpreter','latex')
legend('$v_{real}$', '$v_{cmd}$', '$v^-$','$v^+$', ...
       'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
title('Linear velocity')
grid on

%Angular velocity
subplot(3,1,2)
plot(mp_vel_exp_time,mp_vel_exp(2,:),'LineWidth',lineWidth,'Color',blue); hold on
plot(time,mp_vel(2,:),'LineWidth',lineWidth,'Color',red); hold on
yline(-dq_limit(2),'--','LineWidth',lineWidth,'Color',blue); 
yline(dq_limit(2),'-.','LineWidth',lineWidth,'Color',blue);

xlim([0 plots_end_time])

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$\omega(m/s)$','Interpreter','latex')
legend('$\omega_{real}$', '$\omega_{cmd}$', '$\omega^-$','$\omega^+$', ...
       'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
title('Angular velocity')
grid on

% Prismatic joint velocity
subplot(3,1,3)
plot(dq_exp_time,dq_exp(1,:),'LineWidth',lineWidth,'Color',blue); hold on
plot(time,dq(4,:),'LineWidth',lineWidth,'Color',red); hold on
yline(-dq_limit(3),'--','LineWidth',lineWidth,'Color',blue); 
yline(dq_limit(3),'-.','LineWidth',lineWidth,'Color',blue);

xlim([0 plots_end_time])

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$\dot{z}(m/s)$','Interpreter','latex')
legend('$\dot{z}_{real}$', '$\dot{z}_{cmd}$', '$\dot{z}^-$','$\dot{z}^+$', ...
       'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
title('Prismatic joint velocity')
grid on

%% UR5 joint velocities
figure()

% Joint 1
subplot(3,2,1)
plot(dq_exp_time,dq_exp(2,:),'LineWidth',lineWidth,'Color',blue); hold on
plot(time,dq(5,:),'LineWidth',lineWidth,'Color',red); hold on
yline(-dq_limit(4),'--','LineWidth',lineWidth,'Color',blue); 
yline(dq_limit(4),'-.','LineWidth',lineWidth,'Color',blue);

xlim([0 plots_end_time])

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$\dot{q}_1(m/s)$','Interpreter','latex')
legend('$\dot{q}_{1real}$', '$\dot{q}_{1cmd}$', '$\dot{q}_1^-$','$\dot{q}_1^+$', ...
       'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
title('Joint 1 velocity')
grid on

% Joint 2
subplot(3,2,2)
plot(dq_exp_time,dq_exp(3,:),'LineWidth',lineWidth,'Color',blue); hold on
plot(time,dq(6,:),'LineWidth',lineWidth,'Color',red); hold on
yline(-dq_limit(5),'--','LineWidth',lineWidth,'Color',blue); 
yline(dq_limit(5),'-.','LineWidth',lineWidth,'Color',blue);

xlim([0 plots_end_time])

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$\dot{q}_2(m/s)$','Interpreter','latex')
legend('$\dot{q}_{2real}$', '$\dot{q}_{2cmd}$', '$\dot{q}_2^-$','$\dot{q}_2^+$', ...
       'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
title('Joint 2 velocity')
grid on

% Joint 3
subplot(3,2,3)
plot(dq_exp_time,dq_exp(4,:),'LineWidth',lineWidth,'Color',blue); hold on
plot(time,dq(7,:),'LineWidth',lineWidth,'Color',red); hold on
yline(-dq_limit(6),'--','LineWidth',lineWidth,'Color',blue); 
yline(dq_limit(6),'-.','LineWidth',lineWidth,'Color',blue);

xlim([0 plots_end_time])

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$\dot{q}_3(m/s)$','Interpreter','latex')
legend('$\dot{q}_{3real}$', '$\dot{q}_{3cmd}$', '$\dot{q}_3^-$','$\dot{q}_3^+$', ...
       'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
title('Joint 3 velocity')
grid on

% Joint 4
subplot(3,2,4)
plot(dq_exp_time,dq_exp(5,:),'LineWidth',lineWidth,'Color',blue); hold on
plot(time,dq(8,:),'LineWidth',lineWidth,'Color',red); hold on
yline(-dq_limit(7),'--','LineWidth',lineWidth,'Color',blue); 
yline(dq_limit(7),'-.','LineWidth',lineWidth,'Color',blue);

xlim([0 plots_end_time])

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$\dot{q}_4(m/s)$','Interpreter','latex')
legend('$\dot{q}_{4real}$', '$\dot{q}_{4cmd}$', '$\dot{q}_4^-$','$\dot{q}_4^+$', ...
       'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
title('Joint 4 velocity')
grid on

% Joint 5
subplot(3,2,5)
plot(dq_exp_time,dq_exp(6,:),'LineWidth',lineWidth,'Color',blue); hold on
plot(time,dq(9,:),'LineWidth',lineWidth,'Color',red); hold on
yline(-dq_limit(8),'--','LineWidth',lineWidth,'Color',blue); 
yline(dq_limit(8),'-.','LineWidth',lineWidth,'Color',blue);

xlim([0 plots_end_time])

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$\dot{q}_5(m/s)$','Interpreter','latex')
legend('$\dot{q}_{5real}$', '$\dot{q}_{5cmd}$', '$\dot{q}_5^-$','$\dot{q}_5^+$', ...
       'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
title('Joint 5 velocity')
grid on

% Joint 6
subplot(3,2,6)
plot(dq_exp_time,dq_exp(7,:),'LineWidth',lineWidth,'Color',blue); hold on
plot(time,dq(10,:),'LineWidth',lineWidth,'Color',red); hold on
yline(-dq_limit(9),'--','LineWidth',lineWidth,'Color',blue); 
yline(dq_limit(9),'-.','LineWidth',lineWidth,'Color',blue);

xlim([0 plots_end_time])

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$\dot{q}_6(m/s)$','Interpreter','latex')
legend('$\dot{q}_{6real}$', '$\dot{q}_{6cmd}$', '$\dot{q}_6^-$','$\dot{q}_6^+$', ...
       'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
title('Joint 6 velocity')
grid on
end