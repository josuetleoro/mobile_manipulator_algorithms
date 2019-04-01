function PlotJointVelComp(sim_data_file,mp_vel_time_exp,mp_vel_exp,dq_vel_time_exp,dq_vel_exp)

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

%% Mobile platform velocities

%Linear velocity
figure()
plot(mp_vel_time_exp,mp_vel_exp(1,:),'LineWidth',lineWidth,'Color',blue); hold on
plot(time,mp_vel(1,:),'LineWidth',lineWidth,'Color',red); hold on
yline(-dq_limit(1),'--','LineWidth',lineWidth,'Color',blue); 
yline(dq_limit(1),'-.','LineWidth',lineWidth,'Color',blue);

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$v(m/s)$','Interpreter','latex')
legend('$v_{exp}$', '$v_{sim}$', '$v^-$','$v^+$', ...
       'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
title('Linear velocity')
grid on

%Angular velocity
figure()
plot(mp_vel_time_exp,mp_vel_exp(2,:),'LineWidth',lineWidth,'Color',blue); hold on
plot(time,mp_vel(2,:),'LineWidth',lineWidth,'Color',red); hold on
yline(-dq_limit(2),'--','LineWidth',lineWidth,'Color',blue); 
yline(dq_limit(2),'-.','LineWidth',lineWidth,'Color',blue);

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$\omega(m/s)$','Interpreter','latex')
legend('$\omega_{exp}$', '$\omega_{sim}$', '$\omega^-$','$\omega^+$', ...
       'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
title('Angular velocity')
grid on


end