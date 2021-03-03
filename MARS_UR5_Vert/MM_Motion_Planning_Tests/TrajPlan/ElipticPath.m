function MotPlan=ElipticPath(T0,Tf,tf,ts,tb)
p1=T0(1:3,4);
p2=Tf(1:3,4);

syms u
assume(u, 'real')

%% Curve definition

% %%%%%%%%%%%%% Eliptic path %%%%%%%%%%%%%%
d = pi/2; % Distance to move
% Major and minor axes
a = abs(p2(1)-p1(1));
b = abs(p2(2)-p1(2));

q = p2(1:2) - p1(1:2);
theta=atan2(q(2),q(1));
phi = floor((theta+3*pi/2)/(pi/2))*pi/2;

% Slope for z coordinate
m = (p2(3)-p1(3))/d;
Z = [a*cos(u+phi)+p2(1);
     b*sin(u+phi)+p1(2);
	 m*u+p1(3)];
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

sf=d;
max_vel=sf/(tf-tb);
max_a=max_vel/tb;

%% Make sure there is no problem with independent coordinates
% Position
Z_f_unfixed = matlabFunction(Z);
if numel( symvar(Z) ) > 0
    Z_f_fixed = @(x) Z_f_unfixed(x);
else
    Z_f_fixed = @(x) Z_f_unfixed();
end
Z_f = Z_f_fixed;

% Velocity
dZ_du = diff(Z, u);
dZ_du_f_unfixed = matlabFunction(dZ_du);
if numel( symvar(dZ_du) ) > 0
    dZ_du_f_fixed = @(x) dZ_du_f_unfixed(x);
else
    dZ_du_f_fixed = @(x) dZ_du_f_unfixed();
end
dZ_du_f = dZ_du_f_fixed;

%% Define the evolution of parameter s

[s,ds,~]=Traj5(0,d,ts,0,tf);
time=0:ts:tf;

% i=1;    %Element position
% %Initial parabolic segment
% for t=0:ts:tb
%     s(i)=max_a/2*t^2;
%     ds(i)=max_a*t;
%     time(i)=t;
%     i=i+1;
% end
% %Knot point
% pa_x=s(i-1);
% %Linear segment
% for t=time(i-1)+ts:ts:tf-tb
%     s(i)=pa_x+max_vel*(t-tb);
%     ds(i)=max_vel;
%     time(i)=t;
%     i=i+1;
% end
% %Final parabolic segment
% for t=time(i-1)+ts:ts:tf
%     %Initial parabolic segment
%     s(i)=sf-max_a/2*(tf-t)^2;
%     ds(i)=max_a*(tf-t);
%     time(i)=t;
%     i=i+1;
% end

%% Use the evolution of parameter s to generate the trajectory in each of the coordinates
for i=1:length(time)
    %Position
    pos = Z_f(s(i));
    x(i) = pos(1);
    y(i) = pos(2);
    z(i) = pos(3);    
    %Velocity
    vel = dZ_du_f(s(i))*ds(i);
    dx(i) = vel(1);
    dy(i) = vel(2);
    dz(i) = vel(3);    
end

%% Motion planning for orientation using quaternion polynomial
% Use a fixed orientation for the whole trajectory
q0=cartToQuat(T0(1:3,1:3));
qf=cartToQuat(Tf(1:3,1:3));

%Create the quaternion objects
Q0=Quat(q0');
Qf=Quat(qf');

%Desired initial and final angular velocities and accelerations
w0=[0;0;0];
dw0=[0;0;0];
wf=[0;0;0];
dwf=[0;0;0];
%Do the interpolation
[quat,w,~]=quatPolynomInterpolation(Q0,w0,dw0,Qf,wf,dwf,0,tf,ts,'fixed');

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
% % pause()
% 
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
% % Plot the evolution of wx, wy and wz
% figure()
% subplot(1,3,1)
% plot(time,w(1,:),'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('wx')
% title('wx')
% 
% subplot(1,3,2)
% plot(time,w(2,:),'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('wy')
% title('wy')
% 
% subplot(1,3,3)
% plot(time,w(3,:),'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('wz')
% title('wz')
% 
% %Plot the evolution of quat
% time=0:ts:tf;
% figure()
% subplot(2,2,1)
% plot(time,quat(1,:),'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('quatw')
% title('quatw')
% 
% subplot(2,2,2)
% plot(time,quat(2,:),'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('quatx')
% title('quatx')
% 
% subplot(2,2,3)
% plot(time,quat(3,:),'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('quaty')
% title('quaty')
% 
% subplot(2,2,4)
% plot(time,quat(4,:),'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('quatz')
% title('quatz')
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