function [quat,w,dw]=quatPolynomInterpolation(Qi,wi,dwi,Qf,wf,dwf,ti,tf,ts,method)
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
T=tf-ti;

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
m=T/ts;
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
        tk=ti+k*h;
        [p0,p1,p2,p3,p4,p5]=polinomialCoefficients(tk,tf,h,qN,Qwk_p_1,dQwk_p_1,Qf,dQf,ddQf,'succesive',p0,p1,p2,p3,p4,p5);
        ti_hat = tk;
    end
    tk_plus_1=ti+(k+1)*h;
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
    qN=q(k+1)/norm(q(k+1));    
    quat(:,k+1)=qN.vecRep();
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





