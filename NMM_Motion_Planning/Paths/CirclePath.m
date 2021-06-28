function MotPlan=CirclePath(T0,tf,ts,tb,r)
pos0=T0(1:3,4);

syms u
assume(u, 'real')

height = 0.25;
freq = 4;

%% Curve definition (Circle path)

% Define the path function
d = 2*pi;
F = [r*cos(u)-r+pos0(1);
     r*sin(u)+pos0(2);
	 height*sin(freq*u)+pos0(3)];
 
% Derivative of the path
dF = [-r*sin(u);
       r*cos(u);
       height*freq*cos(freq*u)];

%% Transform to matlab function
F_mat = matlabFunction(F);
dF_mat = matlabFunction(dF);

%% Define the evolution of parameter s

% % Using jerk limited trajectory
% sf = d;
% vel_max = (sf-0)/(tf-tb);
% a_max=vel_max/tb;
% [traj, time] = GenTraj(ts, a_max, vel_max, sf, 1);
% s = traj(3,1:end);
% ds = traj(2,1:end);

% Using parabolic blending
sf=d;
max_vel=sf/(tf-tb);
max_a=max_vel/tb;
i=1;    %Element position
%Initial parabolic segment
for t=0:ts:tb
    s(i)=max_a/2*t^2;
    ds(i)=max_a*t;
    time(i)=t;
    i=i+1;
end
%Knot point
pa_x=s(i-1);
%Linear segment
for t=time(i-1)+ts:ts:tf-tb
    s(i)=pa_x+max_vel*(t-tb);
    ds(i)=max_vel;
    time(i)=t;
    i=i+1;
end
%Final parabolic segment
for t=time(i-1)+ts:ts:tf
    %Initial parabolic segment
    s(i)=sf-max_a/2*(tf-t)^2;
    ds(i)=max_a*(tf-t);
    time(i)=t;
    i=i+1;
end

%% Use the evolution of parameter s to generate the trajectory in each of the coordinates
for i=1:length(time)
    %Position
    pos = F_mat(s(i));
    x(i) = pos(1);
    y(i) = pos(2);
    z(i) = pos(3);
    
    %Velocity
    vel = dF_mat(s(i))*ds(i);
    dx(i) = vel(1);
    dy(i) = vel(2);
    dz(i) = vel(3);    
end

% Point the orientation direction to the center of the circle
target = [pos0(1)-r; pos0(2); 0];
approach = T0(1:3,3);
for k=1:length(x)
    orient = target - [x(k);y(k);0];
    orient = orient / norm(orient);
    normal = cross(orient, approach);
    R = [normal,orient,approach];
    quat(:,k)=cartToQuat(R);
    if k==1
        w(1:3,k)=[0;0;0];
    else
        p = quat(:,k);
        q = quat(:,k-1);
        w(1:3,k)=velFromQuats(quat(:,k-1),quat(:,k),ts);
    end    
end
w(1:3,end)=[0;0;0];

% % Rotate only around the z axis
% z_axis = [0;0;1];
% q0=cartToQuat(T0(1:3,1:3));
% Q0= Quat(q0');
% for i=1:length(s)
%     w(1:3,i)= z_axis*ds(i);
%     if (abs(ds(i)) < 1e-04)        
%         quat_vel = Quat([1,0,0,0]);        
%     else
%         quat_vel = quatVel(w(1:3,i));
%     end
%     
%     if (i == 1)
%         Qk = quat_vel*Q0;
%     else
%         Qk = quat_vel*Quat(quat(1:4,i-1)');
%     end
%     quat(1:4,i) = Qk.vecRep();
% end
% w(1:3,end)=[0;0;0];

%% Plots (Comment if not needed)
% %Show the end effector motion in 3D
% figure()
% Rd=zeros(4,4,length(time));
% %Form the T6Traj matrix
% for i=1:length(x)
%    Rd(:,4,i)=[x(i);y(i);z(i);1];
%    Rd(1:3,1:3,i)=quatToRotMat(quat(:,i));
% end
% plotEndEffectorMotion2(Rd,0.02)
% pause()

%Show the XY motion of the end-effector
figure()
plot(x,y);
grid on;
title('xy')
%Show the Z motion of the end-effector
figure()
plot(time,z);
title('z')

% figure()
% subplot(2,1,1)
% plot(time,s)
% title('s')
% subplot(2,1,2)
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
% 
% Plot the evolution of wx, wy and wz
figure()
subplot(1,3,1)
plot(time,w(1,:),'r','LineWidth',2); grid on
xlabel('time(s)')
ylabel('wx')
title('wx')

subplot(1,3,2)
plot(time,w(2,:),'r','LineWidth',2); grid on
xlabel('time(s)')
ylabel('wy')
title('wy')

subplot(1,3,3)
plot(time,w(3,:),'r','LineWidth',2); grid on
xlabel('time(s)')
ylabel('wz')
title('wz')

%Plot the evolution of quat
time=0:ts:tf;
figure()
subplot(2,2,1)
plot(time,quat(1,:),'r','LineWidth',2); grid on
xlabel('time(s)')
ylabel('quatw')
title('quatw')

subplot(2,2,2)
plot(time,quat(2,:),'r','LineWidth',2); grid on
xlabel('time(s)')
ylabel('quatx')
title('quatx')

subplot(2,2,3)
plot(time,quat(3,:),'r','LineWidth',2); grid on
xlabel('time(s)')
ylabel('quaty')
title('quaty')

subplot(2,2,4)
plot(time,quat(4,:),'r','LineWidth',2); grid on
xlabel('time(s)')
ylabel('quatz')
title('quatz')
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