function plotMobileManipulatorMotion(qmp,T6Traj,endLength)
%Get the Position of the end effector and the approaching direction
pos=reshape(T6Traj(1:3,4,:),3,[]);
appro=reshape(T6Traj(1:3,3,:),3,[]);

%plot the path and orientation of the end effector
n=length(pos); 
for i=1:n
    P1=pos(:,i);
    P2=P1+appro(:,i)*endLength;
    grid on
    v=[P1';P2'];
    plot3(v(:,1),v(:,2),v(:,3)); hold on
    %Plot the position of the end effector
    scatter3(P1(1),P1(2),P1(3),5,'filled','k')
    scatter3(P2(1),P2(2),P2(3),5,'filled','b')
    
    %Plot the position of the mobile platform
    scatter3(qmp(1,i),qmp(2,i),0,5,'filled','k')
end
%Plot the inital and final frame
plotFrame(T6Traj(:,:,1),2,'T0')
plotFrame(T6Traj(:,:,end),2,'Tf')

%plot the mobile platform arrow
plotMobPlatArrow(qmp(1,1),qmp(2,1),qmp(3,1),2,'mp_0');
plotMobPlatArrow(qmp(1,end),qmp(2,end),qmp(3,end),2,'mp_f');

xlabel('x')
ylabel('y')
zlabel('z')
view(-133,31)
end