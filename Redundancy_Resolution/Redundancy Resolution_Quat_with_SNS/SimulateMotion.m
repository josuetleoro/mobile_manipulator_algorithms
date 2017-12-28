addpath SimulateMotion

%Open a dialog box to look for the motion data
uiopen();

%Determine the number of iterations
n=size(q,2);

%Define the drawing limits
%drawLimits=[-5 5 -4 4];

xlimit=abs(ceil(xi(1,1)-xi(1,end)));
ylimit=abs(ceil(xi(2,1)-xi(2,end)));

figure()

%Define the drawing limits
drawLimits=[-1*xlimit-2 xlimit+2 -1*ylimit-2 ylimit+2];

%The end effector path
endEffPos=zeros(2,n);
endEffPos(1,:)=xi_des(1,:);
endEffPos(2,:)=xi_des(2,:);

%% Plot the Initial Position
%Draw the mobile manipulator in the current position
drawMobilePlatform(q(1,1),q(2,1),q(3,1),xi(1,1),xi(2,1))
%Plot the final position
plot(xi_des(1,end),xi_des(2,end),'b.','MarkerSize',20);
%Plot the path
plot(endEffPos(1,:),endEffPos(2,:));
axis(drawLimits)
xlabel('x(m)')
ylabel('y(m)')
hold off

%% Start the simulation after pressing a key
disp('Press a key to start')
pause()
for i=1:n
    %Draw the mobile manipulator in the current position
    drawMobilePlatform(q(1,i),q(2,i),q(3,i),xi(1,i),xi(2,i))
    
    %Plot the final position
    plot(xi_des(1,end),xi_des(2,end),'b.','MarkerSize',20);
    
    %Plot the path
    plot(endEffPos(1,:),endEffPos(2,:));
    
    axis(drawLimits)   
    xlabel('x(m)')
    ylabel('y(m)')    
    
    hold off
    drawnow        
    pause(0.01)        
    
end