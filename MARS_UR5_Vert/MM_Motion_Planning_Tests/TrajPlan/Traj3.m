function [q,dq,ddq]=Traj3(q0,qf,ts,tf)
[a3,a2,a1,a0] = createTraj3(q0,qf,0,0,0,tf);
% make a polynomial
p = [a3,a2,a1,a0];
% Create time vector
time=0:ts:tf;
% Evaluate the polynomial : Position
q = polyval(p,time);
% calculate the first derivative : Velocity
pd = polyder(p);
% Evaluate the velocity
dq = polyval(pd,time);
% calculate the second derivative : Acceleration
pdd = polyder(pd);
% Evaluate the acceleration
ddq = polyval(pdd,time);
end

function [a3,a2,a1,a0] = createTraj3(theta0,thetaf,thetad0,thetadf,tstart,tfinal)
	% inputs : initial position, velocity + final position, velocity + initial and final times
	% output : a vector specifying the polynomial and can be used with poly functions such as : polyder, polyval, etc.
	% create a 3rd order trajectory
	% example:
	% createTraj3(10,30,0,0,0,1)
	%
	%
	% By: Reza Ahmadzadeh - Matlab/Octave - 2013
	T = tfinal - tstart;
	a0 = theta0;
	a1 = thetad0;
	a2 = (-3 * (theta0 - thetaf) - (2 * thetad0+thetadf )*T)/ T ^ 2;
	a3 = (2 * (theta0 - thetaf) + (thetad0+thetadf )*T)/ T ^ 3;
	
end
