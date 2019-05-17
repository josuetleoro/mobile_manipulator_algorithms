function trans = dCubicTrans(N,tf,blend_perc,ts)

time = 0:ts:tf;

tb1 = time(ceil(N*blend_perc))
tb2 = time(ceil(N*(1-blend_perc)))

[a5,a4,a3,a2,a1,a0] = createTraj5(0,1,0,0,0,0,0,tb1);
p1 = [a0,a1,a2,a3,a4,a5];
[a5,a4,a3,a2,a1,a0] = createTraj5(1,0,0,0,0,0,tb2,tf);
p2 = [a0,a1,a2,a3,a4,a5];


blend1 = floor(N*blend_perc)
blend2 = floor(N*(1-blend_perc))

trans = zeros(1,N);
t=0;
k=1;
while k <= N
%     k
%     t
    if (k < blend1)
       trans(k)=p1(1)+p1(2)*t+p1(3)*t^2+p1(4)*t^3+p1(5)*t^4+p1(6)*t^5;        
    end
    
    if (k >= blend1 && k <= blend2)
       trans(k)=1;
    end
    
    if (k > blend2)
       trans(k)=p2(1)+p2(2)*(t-tb2)+p2(3)*(t-tb2)^2+p2(4)*(t-tb2)^3+p2(5)*(t-tb2)^4+p2(6)*(t-tb2)^5;
    end
%     trans(k)
%     pause()
    k=k+1;
    t=t+ts;    
end
% size(trans)
% size(time)
% plot(time, trans)
% pause()

end

function [a5,a4,a3,a2,a1,a0] = createTraj5(theta0,thetaf,thetad0,thetadf,thetadd0,thetaddf,tstart,tfinal)
	% inputs : initial position, velocity, and acceleration + final position, velocity, and acceleration, + initial and final times
	% output : a vector specifying the polynomial and can be used with poly functions such as : polyder, polyval, etc.
	% create a 5th order trajectory
	% example:
	% createTraj5(10,30,0,0,0,0,0,1)
	%
	%
	% By: Reza Ahmadzadeh - Matlab/Octave - 2013
	T = tfinal - tstart;
	a0 = theta0;
	a1 = thetad0;
	a2 = 0.5 * thetadd0;
	a3 =(1/(2*T^3)) * (20 * (thetaf - theta0) - (8 * thetadf+ 12*thetad0 )*T - (3 * thetaddf - thetadd0 )*T^2 );
	a4 =(1/(2*T^4)) * (30 * (theta0 - thetaf) + (14 * thetadf+ 16*thetad0 )*T + (3 * thetaddf - 2*thetadd0 )*T^2 );
	a5 =(1/(2*T^5)) * (12 * (thetaf - theta0) - 6*(thetadf+ thetad0 )*T - (thetaddf - thetadd0 )*T^2 );		
end	
