function plotEndEffectorMotion2(T6Traj,frame_size, step)

set(0,'defaulttextinterpreter','latex')
set(0,'defaulttextfontname', 'Times')
set(0,'defaulttextfontsize',16)

set(0, 'defaultAxesTickLabelInterpreter','latex');
set(0, 'defaultLegendInterpreter','latex');
set(0,'defaultaxesfontsize',16)
set(0,'defaultaxesfontname', 'Times')

labelFontSize=14;

% plot the position
pos=reshape(T6Traj(1:3,4,:),3,[]);
n=length(pos); 
step_points = floor(step/8);
for i=1:step_points:n
    P1=pos(:,i);
    %Plot the position of the end effector
    scatter3(P1(1),P1(2),P1(3),5,'filled','k'); hold on
end

% plot the orientation
for i=1:step:n
    %Plot the position of the end effector
    plotFrame(T6Traj(:,:,i),1,0.6); hold on
end

text(pos(1,1)-0.5,pos(2,1),pos(3,1)+0.2,['    ' '$r(0)$'],'HorizontalAlignment','left','interpreter','latex','FontSize',labelFontSize);
text(pos(1,end),pos(2,end),pos(3,end)+0.1,['    ' '$r(t_f)$'],'HorizontalAlignment','left','interpreter','latex','FontSize',labelFontSize);

axis equal
% xlim([-1, 2.5])
% ylim([-1.5, 1.5])
% zlim([0, 2])
xlabel('$x(m)$','interpreter','latex','FontSize',labelFontSize);
ylabel('$y(m)$','interpreter','latex','FontSize',labelFontSize);
zlabel('$z(m)$','interpreter','latex','FontSize',labelFontSize);


end