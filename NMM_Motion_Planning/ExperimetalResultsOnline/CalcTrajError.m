function [pos_error,or_error]=CalcTrajError(ee_xi,xi_des,time,ts)
pos0 = xi_des(1:3,1);
posf = xi_des(1:3,end);
q0 = xi_des(4:7,1);
qf = xi_des(4:7,end);

%% Motion planning for positon (x,y,z)
x=Traj5(pos0(1),posf(1),time);
y=Traj5(pos0(2),posf(2),time);
z=Traj5(pos0(3),posf(3),time);

% plot(x)
% pause()

%% Motion planning for orientation using quaternion polynomial
%Create the quaternion objects
Q0=Quat(q0');
Qf=Quat(qf');

%Desired initial and final angular velocities and accelerations
w0=[0;0;0];
dw0=[0;0;0];
wf=[0;0;0];
dwf=[0;0;0];
%Do the interpolation
[quat,~,~]=quatPolynomInterpolation(Q0,w0,dw0,Qf,wf,dwf,time,ts,'fixed');

%Return the trajectory errors
pos(1,:) = x;
pos(2,:) = y;
pos(3,:) = z;
pos_error = pos - ee_xi(1:3,:);
or_error = zeros(3,length(ee_xi));
for i=1:length(ee_xi)    
    %temp_quat=quatmultiply(ee_xi(4:7,i)',orienOff)';
    %or_error(1:3,i)=errorFromQuats(quat(1:4,i),temp_quat);
    or_error(1:3,i)=errorFromQuats(quat(1:4,i),ee_xi(4:7,i));
end

% %Eliminate the difference at the start of the trajectory due to sensor drift
% posOff = ee_xi(1:3,1)-pos0(1:3,1);
% ee_xi(1:3,:) = ee_xi(1:3,:) - repmat(posOff,1,length(ee_xi));
% orienOff = quatmultiply(quatinv(ee_xi(4:7,1)'),quat(1:4,1)');
% pos(1,:) = x;
% pos(2,:) = y;
% pos(3,:) = z;
% pos_error = pos - ee_xi(1:3,:);
% or_error = zeros(3,length(ee_xi));
% for i=1:length(ee_xi)    
%     temp_quat=quatmultiply(ee_xi(4:7,i)',orienOff)';
%     or_error(1:3,i)=errorFromQuats(quat(1:4,i),temp_quat);
% end

end

function q=Traj5(q0,qf,time)
[a5,a4,a3,a2,a1,a0] = createTraj5(q0,qf,0,0,0,0,time(1), time(end));
% make a polynomial
p = [a5,a4,a3,a2,a1,a0];

% Evaluate the polynomial : Position
q = polyval(p,time);
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

function [quat,w,dw]=quatPolynomInterpolation(Qi,wi,dwi,Qf,wf,dwf,time,ts,method)
%Method succesive is not working
if (strcmp(method,'succesive'))
   error('Succesive method currently not working, use "fixed" instead')
end
%% Method validation
if ~(strcmp(method,'succesive') || strcmp(method,'fixed'))
   error('Quaterion interpolation method does not exist. Available methods are fixed or succesive')
end
%% Initialization
%Pure quaternion objects for initial and final angular velocity
%and acceleration
Qwi=Quat(0,wi);
dQwi=Quat(0,dwi);
Qwf=Quat(0,wf);
dQwf=Quat(0,dwf);
ti = time(1);
tf = time(end);

%To find the shortest path change the sign of one of the quaternions,
% if the dot product is less than zero
if dot(Qi.vecRep(),Qf.vecRep())<0
    Qf=-1*Qf;
end

%Assign the derivatives of the norm
dNf=0;
ddNf=0;

%% Calculate the polynomial coefficients
dQf=quatDerNorm(Qwf,dNf,Qf);
ddQf=quatSecDerNorm(Qwf,dQwf,dNf,ddNf,Qf);

%% Perform the interpolation
m=length(time)-1;
h=ts;
[p0,p1,p2,p3,p4,p5]=polinomialCoefficients(ti,tf,h,Qi,Qwi,dQwi,Qf,dQf,ddQf,'fixed');
%initialize arrays to accelerate calculations
w=zeros(3,m-1);
dw=zeros(3,m-1);
quat=zeros(4,m-1);
q=Quat.empty(0,m-1);
dq=Quat.empty(0,m-1);
ddq=Quat.empty(0,m-1);
ti_hat = ti;
for k=0:m-1    
    if strcmp(method,'succesive') && k>0
        %tk=ti+k*h;
        tk = time(k+1);
        h = time(k+2)-time(k+1);
        [p0,p1,p2,p3,p4,p5]=polinomialCoefficients(tk,tf,h,qN,Qwk_p_1,dQwk_p_1,Qf,dQf,ddQf,'succesive',p0,p1,p2,p3,p4,p5);
        ti_hat = tk;
    end
    %tk_plus_1=ti+(k+1)*h;
    tk_plus_1=time(k+1);
    tau=(tk_plus_1-ti_hat)/(tf-ti_hat);
    q(k+1)=quatPol(tau,p0,p1,p2,p3,p4,p5);
    dq(k+1)=dquatPol(tk_plus_1,ti_hat,tf,p0,p1,p2,p3,p4,p5);
    ddq(k+1)=ddquatPol(tk_plus_1,ti_hat,tf,p0,p1,p2,p3,p4,p5);
    
    %Compute w(k+1) and dw(k+1)
    Qwk_p_1=2*dq(k+1)*q(k+1).inv();
    dQwk_p_1=2*ddq(k+1)*q(k+1).inv()-2*(dq(k+1)*q(k+1).inv())*(dq(k+1)*q(k+1).inv());
    w(:,k+1)=Qwk_p_1.getV();
    dw(:,k+1)=dQwk_p_1.getV();

    %Compute quat(k+1)
    quat(:,k+1)=q(k+1).vecRep();
    % Since by convention the quaternion scalar part must be kept positive,
    % multiply the quaternion by -1 when the scalar part is negative.
    if quat(1,k+1) < 0
        quat(:,k+1) = -1 * quat(:,k+1);
    end
    quat(:,k+1)=quat(:,k+1)/norm(quat(:,k+1));
end
%Add the initial condition
quat=[Qi.vecRep(),quat];
w=[wi,w];
dw=[dwi,dw];
end

function [p0,p1,p2,p3,p4,p5] = polinomialCoefficients(tk,tf,h,Qk,Qwk,dQwk,Qf,dQf,ddQf,method,p0_prev,p1_prev,p2_prev,p3_prev,p4_prev,p5_prev)
Tk=tf-tk;
%persistent ti;
if strcmp(method,'succesive')
%     tau=(tk-h-ti)/(tf-ti);    
%     qaux=quatPol(tau,p0_prev,p1_prev,p2_prev,p3_prev,p4_prev,p5_prev);
%     dQk=dquatPol(tk-h,ti,tf,p0_prev,p1_prev,p2_prev,p3_prev,p4_prev,p5_prev);
%     ddQk=ddquatPol(tk-h,ti,tf,p0_prev,p1_prev,p2_prev,p3_prev,p4_prev,p5_prev);    
    
    qaux=quatPol(2*h/(Tk+h),p0_prev,p1_prev,p2_prev,p3_prev,p4_prev,p5_prev);       
    dQk=dquatPolDelayed(tk,h,tf,p0_prev,p1_prev,p2_prev,p3_prev,p4_prev,p5_prev);
    ddQk=ddquatPolDelayed(tk,h,tf,p0_prev,p1_prev,p2_prev,p3_prev,p4_prev,p5_prev);    
    
    qk=qaux.vecRep();
    dqk=dQk.vecRep();
    ddqk=ddQk.vecRep();
    N=norm(qk);
    dN=1/N*dot(qk,dqk);
    ddN=1/N*(dot(qk,ddqk)+dot(dqk,dqk)-dN^2);
    
    %Compute the new polynomial coefficients
    dQk=quatDerNorm(Qwk,0,Qk);
    ddQk=quatSecDerNorm(Qwk,dQwk,0,ddN,Qk);
    p0=Qk;
    p1=3*Qk+dQk*Tk;
    p2=1/2*ddQk*Tk^2+3*dQk*Tk+6*Qk;
    p3=Qf;
    p4=3*Qf-dQf*Tk;
    p5=1/2*ddQf*Tk^2-3*dQf*Tk+6*Qf;
else
    %ti=tk;
    dQk=quatDerNorm(Qwk,0,Qk);
    ddQk=quatSecDerNorm(Qwk,dQwk,0,0,Qk);
    p0=Qk;
    p1=3*Qk+dQk*Tk;
    p2=1/2*ddQk*Tk^2+3*dQk*Tk+6*Qk;
    p3=Qf;
    p4=3*Qf-dQf*Tk;
    p5=1/2*ddQf*Tk^2-3*dQf*Tk+6*Qf;
end

end

function q=quatPol(tau,p0,p1,p2,p3,p4,p5)
q=(1-tau)^3*(p0+p1*tau+p2*tau^2)+tau^3*(p3+p4*(1-tau)+p5*(1-tau)^2);
end

function dq=dquatPolDelayed(t,h,tf,p0,p1,p2,p3,p4,p5)
dq = -6*(1-2*h/(tf-t+h))^2*(p0+2*p1*h/(tf-t+h)+4*p2*h^2/(tf-t+h)^2)*h/(tf-t+h)^2+(1-2*h/(tf-t+h))^3*(2*p1*h/(tf-t+h)^2+8*p2*h^2/(tf-t+h)^3)+24*h^3*(p3+p4*(1-2*h/(tf-t+h))+p5*(1-2*h/(tf-t+h))^2)/(tf-t+h)^4+8*h^3*(-2*p4*h/(tf-t+h)^2-4*p5*(1-2*h/(tf-t+h))*h/(tf-t+h)^2)/(tf-t+h)^3;
end

function ddq=ddquatPolDelayed(t,h,tf,p0,p1,p2,p3,p4,p5)
ddq = (24*(1-2*h/(tf-t+h)))*(p0+2*p1*h/(tf-t+h)+4*p2*h^2/(tf-t+h)^2)*h^2/(tf-t+h)^4-12*(1-2*h/(tf-t+h))^2*(2*p1*h/(tf-t+h)^2+8*p2*h^2/(tf-t+h)^3)*h/(tf-t+h)^2-12*(1-2*h/(tf-t+h))^2*(p0+2*p1*h/(tf-t+h)+4*p2*h^2/(tf-t+h)^2)*h/(tf-t+h)^3+(1-2*h/(tf-t+h))^3*(4*p1*h/(tf-t+h)^3+24*p2*h^2/(tf-t+h)^4)+96*h^3*(p3+p4*(1-2*h/(tf-t+h))+p5*(1-2*h/(tf-t+h))^2)/(tf-t+h)^5+48*h^3*(-2*p4*h/(tf-t+h)^2-4*p5*(1-2*h/(tf-t+h))*h/(tf-t+h)^2)/(tf-t+h)^4+8*h^3*(-4*p4*h/(tf-t+h)^3+8*p5*h^2/(tf-t+h)^4-8*p5*(1-2*h/(tf-t+h))*h/(tf-t+h)^3)/(tf-t+h)^3;
end

function eO=errorFromQuats(qd,qe)
%Calculate the orientation error
sO=qd(1)*qe(1)+qd(2:4)'*qe(2:4);
eO=qe(1)*qd(2:4)-qd(1)*qe(2:4)-cross(qd(2:4),qe(2:4));
% Since by convention the quaternion scalar part must be kept positive,
% multiply the quaternion by -1 when the scalar part is negative.
if sO < 0
    eO = -1*eO;
end
end

function error=errorFromQuatsComplete(qd,qe)

%Calculate the orientation error
sO=qd(1)*qe(1)+qd(2:4)'*qe(2:4);
eO=qe(1)*qd(2:4)-qd(1)*qe(2:4)-cross(qd(2:4),qe(2:4));
error = [sO;eO];

end
