close all
% Show the final position
disp('Obtained final pose')
q(:,end)

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

%% UR5 joints position

% Joint 1 and 2
figure()
plot(time,q(5,:),'LineWidth',lineWidth,'Color',blue); hold on
yline(q_limit(4,1),'--','LineWidth',lineWidth,'Color',blue); 
yline(q_limit(4,2),'-.','LineWidth',lineWidth,'Color',blue); 

plot(time,q(6,:),'LineWidth',lineWidth,'Color',red); 
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
plot(time,q(7,:),'LineWidth',lineWidth,'Color',yellow); hold on
yline(q_limit(6,1),'--','LineWidth',lineWidth,'Color',yellow); 
yline(q_limit(6,2),'-.','LineWidth',lineWidth,'Color',yellow); 

plot(time,q(8,:),'LineWidth',lineWidth,'Color',purple); 
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
plot(time,q(9,:),'LineWidth',lineWidth,'Color',green); hold on
yline(q_limit(8,1),'--','LineWidth',lineWidth,'Color',green); 
yline(q_limit(8,2),'-.','LineWidth',lineWidth,'Color',green); 

plot(time,q(10,:),'LineWidth',lineWidth,'Color',cyan); 
yline(q_limit(9,1),'--','LineWidth',lineWidth,'Color',cyan); 
yline(q_limit(9,2),'-.','LineWidth',lineWidth,'Color',cyan);

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(rad)$','interpreter','latex','FontSize',labelFontSize)
legend('$q_{a5}$','$q_{a5}^-$','$q_{a5}^+$', ...
    '$q_{a6}$','$q_{a6}^-$','$q_{a6}^+$', ...
    'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
grid on

%% Position and orientation error
figure()
plot(time,xi_pos_error(1,:),'LineWidth',lineWidth); hold on
plot(time,xi_pos_error(2,:),'LineWidth',lineWidth); hold on
plot(time,xi_pos_error(3,:),'LineWidth',lineWidth);
xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(m)$','interpreter','latex','FontSize',labelFontSize)
legend('$e_{Px}$','$e_{Py}$','$e_{Pz}$','interpreter','latex','FontSize',labelFontSize)
%title('Position error(m)')
grid on

figure()
plot(time,xi_orient_error(1,:),'LineWidth',lineWidth); hold on
plot(time,xi_orient_error(2,:),'LineWidth',lineWidth); hold on
plot(time,xi_orient_error(3,:),'LineWidth',lineWidth);
xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylabel('$(rad)$','interpreter','latex','FontSize',labelFontSize)
legend('$e_{Ox}$','$e_{Oy}$','$e_{Oz}$','interpreter','latex','FontSize',labelFontSize)
grid on
%title('Orientation error(rad)')
