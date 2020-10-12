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
xlabel('$x(m)$','interpreter','latex','FontSize',labelFontSize);
ylabel('$y(m)$','interpreter','latex','FontSize',labelFontSize);

%% Mobile platform velocities
figure()
plot(time,mp_vel(1,:),'LineWidth',lineWidth,'Color',blue); hold on
yline(-dq_limit(1),'--','LineWidth',lineWidth,'Color',blue); 
yline(dq_limit(1),'-.','LineWidth',lineWidth,'Color',blue);

plot(time,mp_vel(2,:),'LineWidth',lineWidth,'Color',red);
yline(-dq_limit(2),'--','LineWidth',lineWidth,'Color',red);
yline(dq_limit(2),'-.','LineWidth',lineWidth,'Color',red);

xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
%legend('$v(m/s)$','$v^-$','$v^+$','$\omega(rad/s)$','$\omega^-$','$\omega^+$','interpreter','latex','FontSize',labelFontSize)
ylabel('$v(m/s)$   ,  $\omega$(rad/s)','Interpreter','latex')
legend('$v$','$v^-$','$v^+$', ...
       '$\omega$','$\omega^-$','$\omega^+$', ...
       'interpreter','latex','NumColumns',2,'FontSize',labelFontSize)
%legend('dv(m/s)','dv_{max}','dv_{min}''w(rad/s)')
%title('Mobile platform velocity commands')
grid on
