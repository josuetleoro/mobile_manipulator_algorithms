function MotPlan=SineWavePath(T0,tf,ts,tb,d,h,Nwaves)
pos0=T0(1:3,4);

syms u
assume(u, 'real')

%% Curve definition
%%%%%%%%%%%%% Sine wave path %%%%%%%%%%%%%%
% d = 4.0;
% h = 0.3;
% Nwaves = 4.0;
lambda = d / Nwaves;
Z = [u;           % r(u)
     0; % th(u)
	 h*sin(2*pi/lambda*u)]; % z(u)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %%%%%%%%%%%%% Circle path %%%%%%%%%%%%%%
% d = 2*pi;
% r = 0.35;
% Z = [0.0;           % r(u)
%      r*sin(u); % th(u)
% 	 r*cos(u)]; % z(u)
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
    pos = Z_f(s(i));
    x(i) = pos0(1) + pos(1);
    y(i) = pos0(2) + pos(2);
    z(i) = pos0(3) + pos(3);
    
    %Velocity
    vel = dZ_du_f(s(i))*ds(i);
    dx(i) = vel(1);
    dy(i) = vel(2);
    dz(i) = vel(3);    
end

% Use a fixed orientation for the whole trajectory
q0=cartToQuat(T0(1:3,1:3));
k=1;
for t=0:ts:tf
    quat(1:4,k)=q0;
    w(1:3,k)=[0;0;0];    
    k=k+1;
end

%% Plots (Comment if not needed)
% figure()
% plot3(x, y, z);
% title('Trajectory 3D plot')
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