%clear all
close all

%Add the MMUR5 path to use the class MMUR5
addpath MARS_UR5

%Create a MMUR5 object
MARS=MARS_UR5();

%Load the test point
testN=3;
TestPointsJLimColAvoid

%Load the joints constraints
JointConstraints

%Set the step size for the gradient descent method and error weight. A
%higher error weight might decrease the manipulability because of its
%influence on the motion.

%With Fs=20Hz
ts=0.05;  %Overwrite ts
alpha=20;  %Best alpha=20
lambda=20.0; %Overwrite  lambda best=20.0

% % %With Fs=100Hz
% ts=0.005;  %Overwrite ts
% alpha=20;   %Best alpha=20
% lambda=20.0; %Overwrite lambda best=2.0

% %For individual manipulabilities
% MM_manip_sel = 1;

%% Initial values of the generalized coordinates of the MM
q0=[tx;ty;phi_mp;tz;qa];
%Find the initial position of the end effector
T0=MARS.forwardKin(q0);

%% Get the desired pose transformation matrix %%%
%RF=eul2rotm([roll, pitch, yaw],'XYZ')
RF=eul2rotm([yaw pitch roll],'ZYX')
Tf=zeros(4,4);
Tf(4,4)=1;
Tf(1:3,1:3)=RF;
Tf(1:3,4)=Pos;

%Euler0=rotm2eul(T0(1:3,1:3),'XYZ')*180/pi
Euler0=rotm2eul(T0(1:3,1:3),'ZYX')*180/pi
T0
Tf

%%%%%%%%%% Show the MM frames in 3D %%%%%%%%%%
% figure()
% %Mobile platform initial pos
% plotMobPlatArrow(tx,ty,phi_mp,2,'mp_0');
% plotFrame(T0,2,'T0'); hold on;
% plotFrame(Tf,2,'Tf')
% xlabel('x')
% ylabel('y')
% zlabel('z')
% view(-133,31)
% pause()

%% Trajectory planning
tic
disp('Calculating the trajectory...')
%Use the trajectory planning function
MotPlan = struct([]);
MotPlan=TrajPlanQuatPolynomials(T0,Tf,ts,tb,tf);
%Set the number of iterations from the motion planning data
N=size(MotPlan.x,2);

%% Initialize variables
q=zeros(10,N);
xi=zeros(7,N);
J = zeros(6,9);
eta=zeros(9,N);
dq=zeros(10,N);
MM_man_measure=zeros(1,N);
ur5_man_measure=zeros(1,N);

%The error weighting matrix Werror
Werror=lambda*eye(6);
Werror(4:6,4:6)=0.1*eye(3);
%Werror(4:6,4:6)=Werror(4:6,4:6)/20;

%The Wjlim weight matrix
Wjlim=eye(9,9);
invWjlim=zeros(9,9);
sqrtInvWjlim=zeros(9,9);
Wmatrix=zeros(9,9);
maxAlpha=zeros(1,N);
minAlpha=zeros(1,N);
clear jLimitGrad;
elbowz=zeros(1,N);

%Identity matrix of size delta=9, delta=M-1 =>10DOF-1
Id=eye(9);

%Create non-holonomic constrains matrix
S=zeros(10,9);
S(3:end,2:end)=eye(8);

%Load the desired position and velocity from the motion planning data
xi_des=zeros(6,N);
xi_des(1,:)=[MotPlan.x];
xi_des(2,:)=[MotPlan.y];
xi_des(3,:)=[MotPlan.z];
xi_des(4:7,:)=[MotPlan.quat];

dxi_des=zeros(6,N);    
dxi_des(1,:)=[MotPlan.dx];
dxi_des(2,:)=[MotPlan.dy];
dxi_des(3,:)=[MotPlan.dz];
dxi_des(4:6,:)=[MotPlan.w];

%Copy the initial values of the motion planning
q(:,1)=q0;
xi(:,1)=xi_des(:,1);
quat_e=xi(4:7,1);

%% Calculate the maximum velocity normalization matrix
Tq=zeros(9,9);
dqLimProd = 1;
for i=1:9
    Tq(i,i)=1/dq_limit(i);
    dqLimProd=dqLimProd*dq_limit(i);
end
invTq=inv(Tq);

%% Get the maximum desired velocity for each coordinate
maxdxi=max(abs(dxi_des),[],2);
for i=1:6
   if maxdxi(i)==0
      maxdxi(i)=1; 
   end
end

%%
disp('Calculating the inverse velocity kinematics solution')
k=1;
while(k<=N)
    %% Redundancy resolution using manipulability gradient
    fprintf('Step %d of %d\n',k,N);
    
    %Replace the elements cos(phi) and sin (phi)
    S(1,1)=cos(q(3,k)); S(2,1)=sin(q(3,k));
    
    %Calculate the Jacobian
    JBar=evaluateJBar(q(3,k),q(5,k),q(6,k),q(7,k),q(8,k),q(9,k));
        
    %% Manipulability gradient
    [MM_dP,MM_manip, ur5_dP, ur5_manip]=manGrad(q(:,k),JBar);   
    MM_man_measure(k)=MM_manip;
    ur5_man_measure(k)=ur5_manip;    
    dP=MM_dP*ur5_manip+ur5_dP*MM_manip;
    %dP=MM_dP;
    %dP=ur5_dP;
       
    %% Joint limit cost function gradient
    Wjlim=jLimitGrad(q(:,k),q_limit);
    invWjlim=inv(Wjlim);
    sqrtInvWjlim=sqrt(invWjlim);
    %prod(diag(sqrtInvWjlim))
    
    %% Collision avoidance gradient
    [Wcol, elbowz(k),dist(k)]=elbowColMat(q(:,k),1,70,1);
    %Wcol=eye(9,9);    
    invWcol=inv(Wcol);
    invWcol=sqrt(invWcol);   
    
    %% Man gradient weightMatrix
    Wman=manipWeight(ur5_dP,ur5_manip,1,25,1);
    %Wman=eye(9,9);
    invWman=inv(Wman);
    invWman=sqrt(invWman); 
    
    %% Inverse differential kinematics         
    %%%%%%%%%%%%%Calculate the position and orientation error%%%%%%%%%%%%
    %Position error
    eP=xi_des(1:3,k)-xi(1:3,k);
    
    %Orientation error
    quat_d=xi_des(4:7,k);    
    eO=errorFromQuats(quat_d,quat_e);  
        
    errorRate(1:3,1)=eP;
    errorRate(4:6,1)=eO;
    error_cont=Werror*errorRate;
    %%%%%%%%%%Calculate the control input and internal motion%%%%%%%%%%%%%
    Wmatrix=invWman*invWcol*sqrtInvWjlim*invTq;
    JBarWeighted=JBar*Wmatrix;
    inv_JBar=pinv(JBarWeighted);
    cont_input=inv_JBar*(dxi_des(:,k)+error_cont);    
    
    %Calculate the weighting by task velocity
    dP=invWman*invWcol*sqrtInvWjlim*Tq*dP;  
    taskNorm=abs(max(dxi_des(1:3,k)./maxdxi(1:3)));   
    dP=taskNorm*dP;
    
    %Calculate the internal motion
    int_motion=(Id-inv_JBar*JBarWeighted)*dP;    
    
    %Calculate the maximum and minimum step size
    [maxAlpha(k),minAlpha(k)] = calcMaxMinAlpha(cont_input,int_motion,dq_limit);
    if maxAlpha(k) < minAlpha(k)
       %error('Could not achieve task that complies with joint velocities limits')
       disp('Could not achieve task that complies with joint velocities limits')
       break
    end
    
    %Saturate alpha in case is out of bounds
    if alpha > maxAlpha(k)
       alpha = maxAlpha(k);
    end
    if alpha < minAlpha(k)
        alpha = minAlpha(k);
    end
    int_motion = alpha*int_motion;    
        
    %Mobility control vector
    eta(:,k)=cont_input+int_motion;    
    eta(:,k)=invWman*invWcol*sqrtInvWjlim*invTq*eta(:,k);
    
    %Calculate the joints velocities
    dq(:,k)=S*eta(:,k);
    
    %% update variables for next iteration       
    if k < N
        %Calculate the joint values
        q(:,k+1)=q(:,k)+dq(:,k)*ts;
        
        %Calculate the position of the end effector
        T=MARS.forwardKin(q(:,k+1));
        xi(1:3,k+1)=T(1:3,4);
        Re=T(1:3,1:3);
        quat_e=cartToQuat(Re);
        %quat_e=rotm2quat(Re)';
        xi(4:7,k+1)=quat_e;
    end
    
    %increment iteration step
    k=k+1;  
end
toc
k=k-1;
if k==N
    disp('Task completed')
else
    disp('Task cannot be executed')
    k
    N
end

time=MotPlan.time(1:k);

%Error of the end effector position
fprintf('Desired Final Position\n');
xi_des(:,k)
fprintf('Obtained Final Position\n');
xi(:,k)

xi_pos_error=xi_des(1:3,:)-xi(1:3,:);
fprintf('Final Position Error\n');
xi_pos_error(:,end)
fprintf('Norm error: %fmm\n',norm(xi_pos_error(:,end))*1000);

fprintf('Desired Final Transformation Matrix\n');
Tf

fprintf('Obtained Final Transformation Matrix\n');
TfObtained(:,4)=[xi(1,end);xi(2,end);xi(3,end);1];
TfObtained(1:3,1:3)=quatToRotMat(xi(4:7,end)');
TfObtained

if k < N
    MM_man_measure = MM_man_measure(1:k);
    ur5_man_measure = ur5_man_measure(1:k);
    maxAlpha = maxAlpha(1:k);
end

%% Plot elbow position
figure()
title('Elbow z pos')
plot(time(1:k),elbowz(1:k),'b','LineWidth',1.5); hold on;
xlabel('time(s)')
grid on

%% Plot all the variables
% Adjust the manipulability measures
MM_man_measure(1)=MM_man_measure(2);
MM_man_measure(end)=MM_man_measure(end-1);
ur5_man_measure(1)=ur5_man_measure(2);
ur5_man_measure(end)=ur5_man_measure(end-1);
% Store the mobile platform velocities
mp_vel=eta(1:3,:);

%Plot all the variables
PlotEvolution

%% Plot end effector path
% figure()
% plot3(xi(1,:),xi(2,:),xi(3,:));
% xlabel('x[m]')
% ylabel('y[m]')
% zlabel('z[m]')
% zlim([0,1.5])
% grid on;

%% Plot the evolution of quat
% quat=xi(4:7,:);   
% figure()
% subplot(2,2,1)
% plot(time,quat(1,:),'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('quat_w')
% title('quat_w')
% 
% subplot(2,2,2)
% plot(time,quat(2,:),'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('quat_x')
% title('quat_x')
% 
% subplot(2,2,3)
% plot(time,quat(3,:),'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('quat_y')
% title('quat_y')
% 
% subplot(2,2,4)
% plot(time,quat(4,:),'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('quat_z')
% title('quat_z')

%% Save the redundancy resolution position and velocities
% JointMotion.q_des=q;
% JointMotion.dq_des=dq;
% JointMotion.ts=ts;
% JointMotion.tf=tf;
% uisave('JointMotion','JointMotion.mat');

