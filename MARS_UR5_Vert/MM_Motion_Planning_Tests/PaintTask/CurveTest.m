clear all

syms u
assume(u, 'real')

%%%%%%%%%%%%% Curve definition %%%%%%%%%%%%%
d = 4.0;
h = 0.6;
Nwaves = 2.0;
lambda = d / Nwaves;

Z = [u;           % r(u)
     0; % th(u)
	 h*sin(2*pi/lambda*u)]; % z(u)
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 
%% Parameter values on curve's limits
u0 = 0;
uf = d;

%% Curve's coordinates derivatives wrt u
dZ_du = diff(Z, u);

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
dZ_du_f_unfixed = matlabFunction(dZ_du);
if numel( symvar(dZ_du) ) > 0
    dZ_du_f_fixed = @(x) dZ_du_f_unfixed(x);
else
    dZ_du_f_fixed = @(x) dZ_du_f_unfixed();
end
dZ_du_f = dZ_du_f_fixed;


%% Find the curve lenght using a line integral
dL_du = sqrt(dZ_du' * dZ_du);
L = double( vpaintegral(dL_du, u, [u0, uf]) );

%% Generate the trajectory
tf = 10;
tb = 2;
ts = 1/50;
max_vel=d/(tf-tb);
max_a=max_vel/tb;
pos0 = [0;0;0];

%% Parameter s (velocity in the path)

i=1;    %Element position
%Initial parabolic segment
for t=0:ts:tb
    s(i)=max_a/2*t^2;
    ds(i)=max_a*t;
    pos = Z_f(s(i));
    x(i) = pos0(1) + pos(1);
    y(i) = pos0(2) + pos(2);
    z(i) = pos0(3) + pos(3);
    vel = dZ_du_f(s(i))*ds(i);
    dx(i) = vel(1);
    dy(i) = vel(2);
    dz(i) = vel(3);
    
    time(i)=t;
    i=i+1;
end

%Knot point
pa_x=s(i-1);

%Linear segment
for t=time(i-1)+ts:ts:tf-tb
    s(i)=pa_x+max_vel*(t-tb);
    ds(i)=max_vel;
    
    pos = Z_f(s(i));
    x(i) = pos0(1) + pos(1);
    y(i) = pos0(2) + pos(2);
    z(i) = pos0(3) + pos(3);
    vel = dZ_du_f(s(i))*ds(i);
    dx(i) = vel(1);
    dy(i) = vel(2);
    dz(i) = vel(3);
    
    time(i)=t;
    i=i+1;
end

%Final parabolic segment
for t=time(i-1)+ts:ts:tf
    %Initial parabolic segment
    s(i)=d-max_a/2*(tf-t)^2;
    ds(i)=max_a*(tf-t);
    
    pos = Z_f(s(i));
    x(i) = pos0(1) + pos(1);
    y(i) = pos0(2) + pos(2);
    z(i) = pos0(3) + pos(3);
    vel = dZ_du_f(s(i))*ds(i);
    dx(i) = vel(1);
    dy(i) = vel(2);
    dz(i) = vel(3);
    
    time(i)=t;
    i=i+1;
end

% Parameter shape
figure()
subplot(2,1,1)
plot(time,s)
subplot(2,1,2)
plot(time,ds)

% Position and velocity
figure()
%position
subplot(2,3,1)
plot(time,x)
subplot(2,3,2)
plot(time,y)
subplot(2,3,3)
plot(time,z)
subplot(2,3,4)
%velocity
plot(time,dx)
subplot(2,3,5)
plot(time,dy)
subplot(2,3,6)
plot(time,dz)

% Curve
figure()
plot3(x, y, z);


