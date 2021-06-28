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

plots_end_time = time(end);

%% Position and orientation error
figure()

subplot(1,3,1)
plot(time_tf,xi_exp(1,:),'LineWidth',lineWidth); hold on
plot(time,xi(1,:),'LineWidth',lineWidth); hold on
xlim([0 plots_end_time])
xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(m)$','interpreter','latex','FontSize',labelFontSize)
legend('$x_{real}$','$x_{sim}$','interpreter','latex','FontSize',labelFontSize)
grid on
title('X(m)')

subplot(1,3,2)
plot(time_tf,xi_exp(2,:),'LineWidth',lineWidth); hold on
plot(time,xi(2,:),'LineWidth',lineWidth); hold on
xlim([0 plots_end_time])
xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(m)$','interpreter','latex','FontSize',labelFontSize)
legend('$y_{real}$','$y_{sim}$','interpreter','latex','FontSize',labelFontSize)
grid on
title('Y(m)')

subplot(1,3,3)
plot(time_tf,xi_exp(3,:),'LineWidth',lineWidth); hold on
plot(time,xi(3,:),'LineWidth',lineWidth); hold on
xlim([0 plots_end_time])
xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(m)$','interpreter','latex','FontSize',labelFontSize)
legend('$z_{real}$','$z_{sim}$','interpreter','latex','FontSize',labelFontSize)
grid on
title('Z(m)')

