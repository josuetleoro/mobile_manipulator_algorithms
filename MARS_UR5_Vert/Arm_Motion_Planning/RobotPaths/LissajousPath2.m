function MotPlan=LissajousPath2(T0,tf,ts,tb)
pos0=T0(1:3,4);
R0=T0(1:3,1:3);
angleRange=deg2rad(60);
quat0_matlab = rotm2quat(R0);

A = 0.2;
B = 0.35;
C = pos0(3)/4;
deltax = pi/2;
deltay = 0.0;
wa = 2;
wb = 1;
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
    dz(i) = -4*C*sin(2*s(i))*ds(i);
    
    %Orientation
    rotAngle = angleRange*sin(s(i));
    eul = [rotAngle 0 0];
%     quat(:,i) = quatmultiply(quat0_matlab,eul2quat(eul, 'XYZ'))';
    quat(:,i) = quatmultiply(eul2quat(eul, 'XYZ'), quat0_matlab)';
    
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
    dz(i) = -4*C*sin(2*s(i))*ds(i);
    
    %Orientation
    rotAngle = angleRange*sin(s(i));
    eul = [rotAngle 0 0];
%     quat(:,i) = quatmultiply(quat0_matlab,eul2quat(eul, 'XYZ'))';
    quat(:,i) = quatmultiply(eul2quat(eul, 'XYZ'), quat0_matlab)';
    
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
    dz(i) = -4*C*sin(2*s(i))*ds(i);
    
    %Orientation
    rotAngle = angleRange*sin(s(i));
    eul = [rotAngle 0 0];
%     quat(:,i) = quatmultiply(quat0_matlab,eul2quat(eul, 'XYZ'))';
    quat(:,i) = quatmultiply(eul2quat(eul, 'XYZ'), quat0_matlab)';
    
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

% %% Plots
% figure()
% plot3(x, y, z);
% title('Trajectory 3D plot')
% 
% figure()
% plot(y, z);
% title('Trajectory YZ plane')
% 
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
% %Plot the evolution of quat
% figure()
% subplot(2,2,1)
% plot(time,quat(1,:),'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('quat_w')
% title('quat_w')
% 
% subplot(2,2,2)
% plot(time,quat(2,:),'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('quat_x')
% title('quat_x')
% 
% subplot(2,2,3)
% plot(time,quat(3,:),'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('quat_y')
% title('quat_y')
% 
% subplot(2,2,4)
% plot(time,quat(4,:),'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('quat_z')
% title('quat_z')
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
MotPlan.time=time;


end