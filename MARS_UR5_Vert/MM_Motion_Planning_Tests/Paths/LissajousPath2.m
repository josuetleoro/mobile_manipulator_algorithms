function MotPlan=LissajousPath2(T0,tf,ts,tb)
pos0=T0(1:3,4);

A = 1.3;
B = 1.3;
C = 0.27;
deltax = 0;
deltay = pi/2;
wa = 1;
wb = 2;
%x = A*cos(wa*t + dx);
%y = B*cos(wb*t + dy);

sf=2*pi;
vel=(2*pi-0)/(tf-tb);
max_a=vel/tb;

%% Parameter s (velocity in the path)

i=1;    %Element position
%Initial parabolic segment
for t=0:ts:tb
    s(i)=max_a/2*t^2;
    ds(i)=max_a*t;
    
    x(i) = pos0(1) + A*cos(wa*(s(i)+pi/2) + deltax);
    y(i) = pos0(2) + B*cos(wb*(s(i)+pi/2) + deltay);
    z(i) = pos0(3) + C*cos(2*s(i))-C;    
    dx(i) = -A*sin(wa*(s(i)+pi/2) + deltax)*wa*ds(i);
    dy(i) = -B*sin(wb*(s(i)+pi/2) + deltay)*wb*ds(i);
    dz(i) = -2*C*sin(2*s(i))*ds(i);
    
    time(i)=t;
    i=i+1;
end

%Knot point
pa_x=s(i-1);

%Linear segment
for t=time(i-1)+ts:ts:tf-tb
    s(i)=pa_x+vel*(t-tb);
    ds(i)=vel;
    
    x(i) = pos0(1) + A*cos(wa*(s(i)+pi/2) + deltax);
    y(i) = pos0(2) + B*cos(wb*(s(i)+pi/2) + deltay);
    z(i) = pos0(3) + C*cos(2*s(i))-C;    
    dx(i) = -A*sin(wa*(s(i)+pi/2) + deltax)*wa*ds(i);
    dy(i) = -B*sin(wb*(s(i)+pi/2) + deltay)*wb*ds(i);
    dz(i) = -2*C*sin(2*s(i))*ds(i);
    
    time(i)=t;
    i=i+1;
end

%Final parabolic segment
for t=time(i-1)+ts:ts:tf
    %Initial parabolic segment
    s(i)=sf-max_a/2*(tf-t)^2;
    ds(i)=max_a*(tf-t);
    
    x(i) = pos0(1) + A*cos(wa*(s(i)+pi/2) + deltax);
    y(i) = pos0(2) + B*cos(wb*(s(i)+pi/2) + deltay);
    z(i) = pos0(3) + C*cos(2*s(i))-C;
    
    dx(i) = -A*sin(wa*(s(i)+pi/2) + deltax)*wa*ds(i);
    dy(i) = -B*sin(wb*(s(i)+pi/2) + deltay)*wb*ds(i);
    dz(i) = -2*C*sin(2*s(i))*ds(i);
    
    time(i)=t;
    i=i+1;
end

q0=cartToQuat(T0(1:3,1:3));
k=1;
for t=0:ts:tf
    quat(1:4,k)=q0;
    w(1:3,k)=[0;0;0];    
    k=k+1;
end

%% Plots
% figure()
% plot3(x, y, z);
% title('Trajectory 3D plot')
% grid on;
% xlabel('x(m)')
% ylabel('y(m)')
% zlabel('z(m)')
% 
% %Show the XY motion of the end-effector
% figure()
% plot(x,y);
% title('xy')
% grid on;
% xlabel('x(m)')
% ylabel('y(m)')
% %Show the Z motion of the end-effector
% figure()
% plot(time,z);
% title('z')
% grid on;
% xlabel('time(s)')
% ylabel('z(m)')
% pause()

% figure()
% subplot(3,1,1)
% plot(time,s)
% title('s')
% subplot(3,1,2)
% plot(time,ds)
% title('ds')
% 
% figure()
% subplot(3,1,1)
% plot(time,x)
% title('x')
% subplot(3,1,2)
% plot(time,y)
% title('y')
% subplot(3,1,3)
% plot(time,z)
% title('z')
% 
% figure()
% subplot(3,1,1)
% plot(time,dx)
% title('dx')
% subplot(3,1,2)
% plot(time,dy)
% title('dy')
% subplot(3,1,3)
% plot(time,dz)
% title('dz')
% 
% pause()

%% Return the motion planning data
MotPlan={};
MotPlan.x=x;
MotPlan.dx=dx;
MotPlan.y=y;
MotPlan.dy=dy;
MotPlan.z=z;
MotPlan.dz=dz;
MotPlan.quat=quat;
MotPlan.w=w;
MotPlan.time=time;


end