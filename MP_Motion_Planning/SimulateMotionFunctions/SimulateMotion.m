addpath SimulateMotionFunctions
figure()

%Determine the number of iterations
n=size(x,2);
%Define the drawing limits
%drawLimits=[-5 5 -4 4];

xlimit=abs(ceil(x(1)-x(end)));
ylimit=abs(ceil(y(1)-y(end)));

x_min=min(x);
x_max=max(x);
y_min=min(y);
y_max=max(y);

%Define the drawing limits
%drawLimits=[-1*xlimit-2 xlimit-1 -1*ylimit-2 ylimit-1];
drawLimits=[ceil(x_min)-2 x_max+1 y_min-2 y_max+1];

%% Plot the Initial Position
%Draw the mobile manipulator in the current position
drawMobilePlatform(x(1),y(1),theta(1))
%Plot the final position
plot(xd,yd,'b.','MarkerSize',20);
%Plot the path
plot(x,y,'r');
axis(drawLimits)
xlabel('x(m)')
ylabel('y(m)')
hold off
grid on

%% Start the simulation after pressing a key
disp('Press a key to start')
pause()
for i=1:n
    %Draw the mobile manipulator in the current position
    drawMobilePlatform(x(i),y(i),theta(i))
    
    %Plot the final position
    plot(xd,yd,'b.','MarkerSize',20);
    
    %Plot the path
    plot(x,y,'r');
    
    axis(drawLimits)   
    xlabel('x(m)')
    ylabel('y(m)')    
    
    hold off
    grid on
    drawnow        
    %pause(0.001)        
    
end