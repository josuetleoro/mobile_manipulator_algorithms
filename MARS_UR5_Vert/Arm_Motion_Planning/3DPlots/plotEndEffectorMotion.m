function plotEndEffectorMotion(T6Traj,endLength)
%Get the Position of the end effector and the approaching direction
pos=reshape(T6Traj(1:3,4,:),3,[]);
appro=reshape(T6Traj(1:3,3,:),3,[]);

%plot the path and orientation of the end effector
n=length(pos); 
for i=1:n
    P1=pos(:,i);
    P2=P1+appro(:,i)*endLength;
%     if i==1 %For the first point use a different line to notice it
%         line([P1(1) P2(1)],[P1(2) P2(2)],[P1(3) P2(3)],'Color','c','LineWidth',5); hold on
%     end
    grid on
    v=[P1';P2'];
    plot3(v(:,1),v(:,2),v(:,3)); hold on
    %Plot the position of the end effector
    scatter3(P1(1),P1(2),P1(3),5,'filled','k')
    scatter3(P2(1),P2(2),P2(3),5,'filled','b')
end
%line([P1(1) P2(1)],[P1(2) P2(2)],[P1(3) P2(3)],'Color','r','LineWidth',5);
plotFrames(T6Traj(:,:,1),T6Traj(:,:,end),2)

end