function MotPlan=EllipticPath(T0,Tf,tf,ts,tb)
p1=T0(1:3,4);
p2=Tf(1:3,4);

syms u
assume(u, 'real')

%% Curve definition

% %%%%%%%%%%%%% Eliptic path %%%%%%%%%%%%%%
% Define the center as the closest point to the origin
q = p2(1:2)-p1(1:2);
c1 = p1(1:2) + [q(1); 0];
c2 = p1(1:2) + [0; q(2)];
cx = 0;
cy = 0;
if (norm(c1) < norm(c2))
    cx = c1(1);
    cy = c1(2);
else
    cx = c2(1);
    cy = c2(2);
end
center = [cx;cy];

% Find the angles of P1 and P2
v1 = p1(1:2)-center;
v2 = p2(1:2)-center;
u1 = atan2(v1(2),v1(1));
u2 = atan2(v2(2),v2(1));
% Make sure the shortest distance is being used
diff = mod(u2-u1+pi,2*pi)-pi;
u2 = u1 + diff;
% Find the radii
th_a = abs(cos(u1));
th_b = abs(sin(u1));
a = 0;
b = 0;

if (th_a > 0.001)
    a = (p1(1)-cx)/cos(u1);
else
    a = (p2(1)-cx)/cos(u2);
end

if (th_b > 0.001)
    b = (p1(2)-cy)/sin(u1);
else
    b = (p2(2)-cy)/sin(u2);
end

% Slope for z coordinate
m = (p2(3)-p1(3))/(u2-u1);

% Define the path function
F = [a*cos(u)+cx;
     b*sin(u)+cy;
	 m*(u-u1)+p1(3)];
 
% Derivative of the path
dF = [-a*sin(u);
       b*cos(u);
       m]; 

%% Transform to matlab function
F_mat = matlabFunction(F);
dF_mat = matlabFunction(dF);

% %% Test points
% p1_test = F_mat(u1)
% p2_test = F_mat(u2)

%% Define the evolution of parameter s

% Using fifth order polynomial
[s,ds,~]=Traj5(u1,u2,ts,0,tf);
time=0:ts:tf;

% %% Using parabolic blending
% % sf=d;
% % max_vel=sf/(tf-tb);
% % max_a=max_vel/tb;
% % i=1;    %Element position
% % %Initial parabolic segment
% % for t=0:ts:tb
% %     s(i)=max_a/2*t^2;
% %     ds(i)=max_a*t;
% %     time(i)=t;
% %     i=i+1;
% % end
% % %Knot point
% % pa_x=s(i-1);
% % %Linear segment
% % for t=time(i-1)+ts:ts:tf-tb
% %     s(i)=pa_x+max_vel*(t-tb);
% %     ds(i)=max_vel;
% %     time(i)=t;
% %     i=i+1;
% % end
% % %Final parabolic segment
% % for t=time(i-1)+ts:ts:tf
% %     %Initial parabolic segment
% %     s(i)=sf-max_a/2*(tf-t)^2;
% %     ds(i)=max_a*(tf-t);
% %     time(i)=t;
% %     i=i+1;
% % end

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

% %% Plots (Comment if not needed)
% % %Show the end effector motion in 3D
% % figure()
% % Rd=zeros(4,4,length(time));
% % %Form the T6Traj matrix
% % for i=1:length(x)
% %    Rd(:,4,i)=[x(i);y(i);z(i);1];
% %    Rd(1:3,1:3,i)=quatToRotMat(quat(:,i));
% % end
% % plotEndEffectorMotion2(Rd,0.02)
% % pause()
% 
% %Show the XY motion of the end-effector
% figure()
% plot(x,y); hold on
% plot(center(1),center(2),'r*'); hold on
% 
% %Plot P1 and P2
% plot(p1(1),p1(2),'b*');
% plot(p2(1),p2(2),'b*');
% % axis equal
% text(p1(1),p1(2),'P1d')
% text(p2(1),p2(2),'P2d')
%
% % figure()
% % subplot(2,1,1)
% % plot(time,s)
% % title('s')
% % subplot(2,1,2)
% % plot(time,ds)
% % title('ds')
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
% % 
% % figure()
% % subplot(3,1,1)
% % plot(time,dx)
% % title('dx')
% % subplot(3,1,2)
% % plot(time,dy)
% % title('dy')
% % subplot(3,1,3)
% % plot(time,dz)
% % title('dz')
% pause()
% 
% % % Plot the evolution of wx, wy and wz
% % figure()
% % subplot(1,3,1)
% % plot(time,w(1,:),'r','LineWidth',2); grid on
% % xlabel('time(s)')
% % ylabel('wx')
% % title('wx')
% % 
% % subplot(1,3,2)
% % plot(time,w(2,:),'r','LineWidth',2); grid on
% % xlabel('time(s)')
% % ylabel('wy')
% % title('wy')
% % 
% % subplot(1,3,3)
% % plot(time,w(3,:),'r','LineWidth',2); grid on
% % xlabel('time(s)')
% % ylabel('wz')
% % title('wz')
% % 
% % %Plot the evolution of quat
% % time=0:ts:tf;
% % figure()
% % subplot(2,2,1)
% % plot(time,quat(1,:),'r','LineWidth',2); grid on
% % xlabel('time(s)')
% % ylabel('quatw')
% % title('quatw')
% % 
% % subplot(2,2,2)
% % plot(time,quat(2,:),'r','LineWidth',2); grid on
% % xlabel('time(s)')
% % ylabel('quatx')
% % title('quatx')
% % 
% % subplot(2,2,3)
% % plot(time,quat(3,:),'r','LineWidth',2); grid on
% % xlabel('time(s)')
% % ylabel('quaty')
% % title('quaty')
% % 
% % subplot(2,2,4)
% % plot(time,quat(4,:),'r','LineWidth',2); grid on
% % xlabel('time(s)')
% % ylabel('quatz')
% % title('quatz')
% % pause()
% 
% %% Return the motion planning data
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