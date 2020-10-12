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

%% Pose plots
figure()
plot(time,x,'b','LineWidth',lineWidth);
ylabel('$x(m)$','interpreter','latex','FontSize',labelFontSize)
xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
grid on;

figure()
plot(time,y,'r','LineWidth',lineWidth);
ylabel('$y(m)$','interpreter','latex','FontSize',labelFontSize)
xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
grid on;

figure()
plot(time,theta,'g','LineWidth',lineWidth);
ylabel('$theta(rad)$','interpreter','latex','FontSize',labelFontSize)
xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
grid on;

%% Wheels velocities
figure()
plot(time,wr,'b','LineWidth',lineWidth);
ylabel('$\omega_r(m)$','interpreter','latex','FontSize',labelFontSize)
xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
grid on;

figure()
plot(time,wl,'r','LineWidth',lineWidth);
ylabel('$\omega_l(m)$','interpreter','latex','FontSize',labelFontSize)
xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
grid on;

