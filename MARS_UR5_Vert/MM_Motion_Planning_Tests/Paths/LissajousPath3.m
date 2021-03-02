function MotPlan=LissajousPath2(T0,tf,ts,tb)
%x = A*cos(wa*t + dx); %y = B*cos(wb*t + dy);
pos0=T0(1:3,4);

A = 0.8;
B = 0.8;
C = pos0(3)/4;
deltax = pi/2;
deltay = 0.0;
wa = 2;
wb = 1;


sf=2*pi;
vel=(2*pi-0)/(tf-tb);
max_a=vel/tb;

%% Parameter s (velocity in the path)

i=1;    %Element position
%Initial parabolic segment
tan_vel = zeros(3,tf/ts+1);
accel = zeros(3,tf/ts+1);
normal = zeros(3,tf/ts+1);
for t=0:ts:tb
    s(i)=max_a/2*t^2;
    ds(i)=max_a*t;
    dds(i)=max_a;
    
    x(i) = pos0(1) + A*cos(wa*(s(i)+pi/2) + deltax);
    y(i) = pos0(2) + B*cos(wb*(s(i)+pi/2) + deltay);
    z(i) = pos0(3) + C*cos(2*s(i))-C;    
    
    dx(i) = -A*wa*sin(wa*(s(i)+pi/2) + deltax)*ds(i);
    dy(i) = -B*wb*sin(wb*(s(i)+pi/2) + deltay)*ds(i);
    dz(i) = -2*C*sin(2*s(i))*ds(i);
    
    ddx(i) = -A*wa^2*cos(wa*(s(i)+pi/2) + deltax)*ds(i)^2+dx(i)/ds(i)*dds(i);
    ddy(i) = -B*wb^2*sin(wb*(s(i)+pi/2) + deltay)*ds(i)^2+dy(i)/ds(i)*dds(i);
    ddz(i) = -4*C*cos(2*s(i))*ds(i)^2+dz(i)/ds(i)*dds(i);
    
    tan_vel(:,i) = [dx(i);dy(i);dz(i)];
    tan_vel(:,i) = tan_vel(:,i)/norm(tan_vel(:,i));
    accel(:,i) = [ddx(i);ddy(i);ddz(i)];
%     approach(:,i) = accel(:,i)-dot(tan_vel(:,i),accel(:,i))*tan_vel(:,i);
%     approach(:,i) = approach(:,i)/norm(approach(:,i));
    
    approach(:,i) = cross(accel(:,i), tan_vel(:,i));
    approach(:,i) = approach(:,i)/norm(approach(:,i));    
    
    time(i)=t;
    i=i+1;
end

%Knot point
pa_x=s(i-1);

%Linear segment
for t=time(i-1)+ts:ts:tf-tb
    s(i)=pa_x+vel*(t-tb);
    ds(i)=vel;
    dds(i)=0;
    
    x(i) = pos0(1) + A*cos(wa*(s(i)+pi/2) + deltax);
    y(i) = pos0(2) + B*cos(wb*(s(i)+pi/2) + deltay);
    z(i) = pos0(3) + C*cos(2*s(i))-C;    
    
    dx(i) = -A*sin(wa*(s(i)+pi/2) + deltax)*wa*ds(i);
    dy(i) = -B*sin(wb*(s(i)+pi/2) + deltay)*wb*ds(i);
    dz(i) = -2*C*sin(2*s(i))*ds(i);
    
    ddx(i) = -A*wa^2*cos(wa*(s(i)+pi/2) + deltax)*ds(i)^2+dx(i)/ds(i)*dds(i);
    ddy(i) = -B*wb^2*sin(wb*(s(i)+pi/2) + deltay)*ds(i)^2+dy(i)/ds(i)*dds(i);
    ddz(i) = -4*C*cos(2*s(i))*ds(i)^2+dz(i)/ds(i)*dds(i);
    tan_vel(:,i) = [dx(i);dy(i);dz(i)];
    tan_vel(:,i) = tan_vel(:,i)/norm(tan_vel(:,i));
    accel(:,i) = [ddx(i);ddy(i);ddz(i)];
%     approach(:,i) = accel(:,i)-dot(tan_vel(:,i),accel(:,i))*tan_vel(:,i);
%     approach(:,i) = approach(:,i)/norm(approach(:,i));
    
    approach(:,i) = cross(accel(:,i), tan_vel(:,i));
    approach(:,i) = approach(:,i)/norm(approach(:,i));    
    
    time(i)=t;
    i=i+1;
end

%Final parabolic segment
for t=time(i-1)+ts:ts:tf
    %Initial parabolic segment
    s(i)=sf-max_a/2*(tf-t)^2;
    ds(i)=max_a*(tf-t);
    dds(i)=-max_a;
    
    x(i) = pos0(1) + A*cos(wa*(s(i)+pi/2) + deltax);
    y(i) = pos0(2) + B*cos(wb*(s(i)+pi/2) + deltay);
    z(i) = pos0(3) + C*cos(2*s(i))-C;
    
    dx(i) = -A*sin(wa*(s(i)+pi/2) + deltax)*wa*ds(i);
    dy(i) = -B*sin(wb*(s(i)+pi/2) + deltay)*wb*ds(i);
    dz(i) = -2*C*sin(2*s(i))*ds(i);
    
    ddx(i) = -A*wa^2*cos(wa*(s(i)+pi/2) + deltax)*ds(i)^2+dx(i)/ds(i)*dds(i);
    ddy(i) = -B*wb^2*sin(wb*(s(i)+pi/2) + deltay)*ds(i)^2+dy(i)/ds(i)*dds(i);
    ddz(i) = -4*C*cos(2*s(i))*ds(i)^2+dz(i)/ds(i)*dds(i);
    tan_vel(:,i) = [dx(i);dy(i);dz(i)];
    tan_vel(:,i) = tan_vel(:,i)/norm(tan_vel(:,i));
    accel(:,i) = [ddx(i);ddy(i);ddz(i)];
%     approach(:,i) = accel(:,i)-dot(tan_vel(:,i),accel(:,i))*tan_vel(:,i);
%     approach(:,i) = approach(:,i)/norm(approach(:,i)); 
    
    approach(:,i) = cross(accel(:,i), tan_vel(:,i));
    approach(:,i) = approach(:,i)/norm(approach(:,i));    
    
    time(i)=t;
    i=i+1;
end

% %% Lissajous path using parameter s
% for k=1:size(s,2)
%     x(k) = pos0(1) + A*cos(wa*(s(k)+pi/2) + deltax);
%     y(k) = pos0(2) + B*cos(wb*(s(k)+pi/2) + deltay);
%     z(k) = pos0(3) + C*cos(2*s(k))-C;
%     dx(k) = -A*wa*sin(wa*(s(k)+pi/2) + deltax);
%     dy(k) = -B*wb*sin(wb*(s(k)+pi/2) + deltay);
%     dz(k) = -2*C*sin(s(k));
% end

q0=cartToQuat(T0(1:3,1:3));

k=1;
for t=0:ts:tf
    quat(1:4,k)=q0;
    w(1:3,k)=[0;0;0];    
    k=k+1;
end

%% Plots
pos = [x;y;z];
figure()
plot3(x, y, z); hold on;
title('Trajectory 3D plot')
% xlim([-1,1.5])
% ylim([-1,1])
% zlim([0,1])
plotVectors(pos,accel,'r');
plotVectors(pos,tan_vel,'g');
plotVectors(pos,approach,'b');

%figure()
pause()

figure()
plot(y, z);
title('Trajectory YZ plane')

figure()
subplot(3,1,1)
plot(time,s)
title('s')
subplot(3,1,2)
plot(time,ds)
title('ds')

figure()
subplot(3,1,1)
plot(time,x)
title('x')
subplot(3,1,2)
plot(time,y)
title('y')
subplot(3,1,3)
plot(time,z)
title('z')

figure()
subplot(3,1,1)
plot(time,dx)
title('dx')
subplot(3,1,2)
plot(time,dy)
title('dy')
subplot(3,1,3)
plot(time,dz)
title('dz')

pause()

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