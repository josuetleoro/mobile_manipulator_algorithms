function [quat,w,dw]=quatInterpolation(Qi,wi,dwi,Qf,wf,dwf,ti,tf,ts)
%% Initialization
%Pure quaternion objects for initial and final angular velocity
%and acceleration
Qwi=Quat(0,wi);
Qdwi=Quat(0,dwi);
Qwf=Quat(0,wf);
Qdwf=Quat(0,dwf);
T=tf-ti;

%To find the shortest path change the sign of one of the quaternions,
% if the dot product is less than zero
if dot(Qi.vecRep(),Qf.vecRep())<0
    Qf=-1*Qf;
end

%Assign the derivatives of the norm
dNi=0;
ddNi=0;
dNf=0;
ddNf=0;

%% Calculate the polynomial coefficients
dQi=quatDerNorm(Qwi,dNi,Qi);
ddQi=quatSecDerNorm(Qwi,Qdwi,dNi,ddNi,Qi);
dQf=quatDerNorm(Qwf,dNf,Qf);
ddQf=quatSecDerNorm(Qwf,Qdwf,dNf,ddNf,Qf);

p0=Qi;
p1=3*Qi+dQi*T;
p2=1/2*ddQi*T^2+3*dQi*T+6*Qi;
p3=Qf;
p4=3*Qf-dQf*T;
p5=1/2*ddQf*T^2-3*dQf*T+6*Qf;

%% Perform the interpolation
m=T/ts;
h=ts;
%initialize arrays to accelerate calculations
w=zeros(3,m-1);
dw=zeros(3,m-1);
quat=zeros(4,m-1);
q=Quat.empty(0,m-1);
dq=Quat.empty(0,m-1);
ddq=Quat.empty(0,m-1);
for k=0:m-1
    t=ti+(k+1)*h;
    tau=(t-ti)/(tf-ti);
    q(k+1)=quatPol(tau,p0,p1,p2,p3,p4,p5);
    dq(k+1)=dquatPol(t,ti,tf,p0,p1,p2,p3,p4,p5);
    ddq(k+1)=ddquatPol(t,ti,tf,p0,p1,p2,p3,p4,p5);
    
    %Compute w(k+1) and dw(k+1)
    Qwk_p_1=2*dq(k+1)*q(k+1).inv();
    Qdwk_p_1=2*ddq(k+1)*q(k+1).inv()-2*(dq(k+1)*q(k+1).inv())*(dq(k+1)*q(k+1).inv());
    w(:,k+1)=Qwk_p_1.getV();
    dw(:,k+1)=Qdwk_p_1.getV();

    %Compute quat(k+1)
    qN=q(k+1)/norm(q(k+1));    
    quat(:,k+1)=qN.vecRep();
end
%Add the initial condition
quat=[Qi.vecRep(),quat];
w=[wi,w];
dw=[dwi,dw];
end





