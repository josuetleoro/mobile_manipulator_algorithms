clear all
close all

%Add the MMUR5 path to use the class MMUR5
addpath MARS_UR5

%Create a MMUR5 object
MARS=MARS_UR5();

%Load the test point
testN=15;
TestPoints

%Load the joints constraints
JointConstraints

%% Set the step size for the gradient descent method and error weight.
%A higher error weight might decrease the manipulability because of its
%influence on the motion.

ts=1/20;    %Sampling time
alpha=12; 
kappa=0.92;
if kappa > 0.92
    error('The manipulability of the entire system should also be considered.');
end

Kp_pos=10;
Ki_pos=50;
Kp_or=20;

%% Initial values of the generalized coordinates of the MM
q0=[tx;ty;phi_mp;tz;qa];
%Find the initial position of the end effector
T0=MARS.forwardKin(q0);
Pos_0=T0(1:3,4);
quat_0=cartToQuat(T0(1:3,1:3));

%% Get the desired pose transformation matrix %%%
Rf=quat2rotm(quat_f');
Tf=zeros(4,4);
Tf(4,4)=1;
Tf(1:3,1:3)=Rf;
Tf(1:3,4)=Pos_f;

% Tf2=zeros(4,4);
% Tf2(4,4)=1;
% Tf2(1:3,1:3)=Rf2;
% Tf2(1:3,4)=Pos_f;
% Tf2(2,4)=Tf2(2,4)+2;
% Tf2
T0
Tf

%%%%%%%%%% Show the MM frames in 3D %%%%%%%%%%
% % figure()
% %Mobile platform initial pos
% plotFrame(eye(4),2,'mp_0'); hold on;
% %plotMobPlatArrow(tx,ty,phi_mp,2,'mp_0');
% plotFrame(T0,2,'T0'); hold on;
% plotFrame(Tf,2,'Tf'); 
% plotFrame(Tf2,2,'Tf2');
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
MotPlan=TrajPlanQuatPolynomials(Pos_0,quat_0,Pos_f,quat_f,ts,tb,tf);
%Set the number of iterations from the motion planning data
N=size(MotPlan.x,2);

%% Initialize variables
q=zeros(10,N);
xi=zeros(7,N);
J = zeros(6,9);
eta=zeros(9,N);
dq=zeros(10,N);
W_measure=zeros(1,N);
MM_man_measure=zeros(1,N);
ur5_man_measure=zeros(1,N);

%The error weighting matrices
Werror=zeros(6,6);
Werror(1:3,1:3)=Kp_pos*eye(3);
Werror(4:6,4:6)=Kp_or*eye(3);

Wierror=zeros(6,6);
Wierror(1:3,1:3)=Ki_pos*eye(3);

%The Wjlim weight matrix
maxAlpha=zeros(1,N);
minAlpha=zeros(1,N);
clear prevGradH prevGradPElbow prevGradPWrist;
dist_elbow=zeros(1,N);
dist_wrist=zeros(1,N);
wrist_pos=zeros(3,N);

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
for i=1:9
    Tq(i,i)=1/dq_limit(i);
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
trans=sigmoid(MotPlan.time,2*tf/3,2);
errorPrev=zeros(6,1);
ierror=zeros(6,1);
error_cont=zeros(6,1);
Rk=zeros(3,3,N);
while(k<=N)
    %% Redundancy resolution using manipulability gradient
    fprintf('Step %d of %d\n',k,N);
    
    %Replace the elements cos(phi) and sin (phi)
    S(1,1)=cos(q(3,k)); S(2,1)=sin(q(3,k));
    
    %Calculate the Jacobian
    JBar=evaluateJBar(q(3,k),q(5,k),q(6,k),q(7,k),q(8,k),q(9,k));    
   
    %% Manipulability gradient
    %[MM_dP,MM_manip, ur5_dP, ur5_manip]=manGradTaskDir(q(:,k),JBar,Wtaskdir);   
    [MM_dP,MM_manip, ur5_dP, ur5_manip]=manGrad2(q(:,k),JBar);   
    MM_man_measure(k)=MM_manip;
    ur5_man_measure(k)=ur5_manip;

%     dP=ur5_manip*MM_dP+MM_manip*ur5_dP;                                   %Combined Mobile manipulator and robot arm
%     W_measure(k)=MM_manip*ur5_manip;
    
    dP=(1-kappa)*(ur5_manip*MM_dP+MM_manip*ur5_dP)+(kappa)*ur5_dP;                              %Combined Mobile manipulator and robot arm
    W_measure(k)=(1-kappa)*MM_manip*ur5_manip+(kappa)*ur5_manip;
    
%     dP=(min_mm+1-kappa)*MM_manip+(kappa-min_mm)*ur5_dP;                              %Combined Mobile manipulator and robot arm
%     W_measure(k)=(min_mm+1-kappa)*MM_manip+(kappa-min_mm)*ur5_manip;
    
%     dP=(1-trans(k))*MM_dP+trans(k)*ur5_dP;                                %Combined Mobile manipulator and robot arm
%     W_measure(k)=(1-trans(k))*MM_manip+trans(k)*ur5_manip;                %With transition

    %dP=MM_dP;                                                              %Mobile manipulator system alone
    %dP=ur5_dP;                                                             %Robot arm alone
    dP=S'*dP;    
    %% Joint limit cost function gradient
    Wjlim=jLimitGrad(q(:,k),q_limit);
    %Wjlim(2,2)=0;
    %Wjlim=eye(9,9);    
    
    %% Collision avoidance weighting matrices
    [Wcol_elbow, dist_elbow(k)]=elbowColMat(q(:,k),0.001,50,1);
    [Wcol_wrist, dist_wrist(k), wrist_pos(:,k)]=wristColMat(q(:,k),0.001,50,1);
    Wcol=Wcol_elbow*Wcol_wrist;
    %Wcol=eye(9,9);
    
    %% Inverse differential kinematics         
    %%%%%%%%%%%%%Calculate the position and orientation error%%%%%%%%%%%%
    %Position error
    eP=xi_des(1:3,k)-xi(1:3,k);
    xi_pos_error(1:3,k)=eP;
    
    %Orientation error
    quat_d=xi_des(4:7,k);    
    %eO=errorFromQuats(quat_d,quat_e);
    eO=errorFromQuatsR(quat2rotm(quat_d'),quat2rotm(quat_e'));
    xi_orient_error(1:3,k)=eO;
        
    errorRate(1:3,1)=eP;
    errorRate(4:6,1)=eO;
    ierror(1:3)=ierror(1:3)+eP*ts;
    ierror(4:6)=zeros(3,1);
    errorPrev=errorRate;
    error_cont=Werror*errorRate+Wierror*ierror;
    %%%%%%%%%%Calculate the control input and internal motion%%%%%%%%%%%%%
    Wmatrix=Wcol*Wjlim*invTq;
    JBarWeighted=JBar*Wmatrix;
    inv_JBar=pinv(JBarWeighted);
    cont_input=inv_JBar*(dxi_des(:,k)+error_cont);    
    
    %Calculate the weighting by task velocity
    taskNorm=abs(max(dxi_des(1:3,k)./maxdxi(1:3)));   
    dP=taskNorm*dP;    
    %Calculate the internal motion
    dP=Wcol*Wjlim*dP;
    int_motion=(Id-inv_JBar*JBarWeighted)*dP;
   
    %Calculate the maximum and minimum step size
    [maxAlpha(k),minAlpha(k)] = calcMaxMinAlpha(cont_input,int_motion,dq_limit);
    if maxAlpha(k) < minAlpha(k)
       diag(Wcol)
       diag(Wjlim)
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
    
    %Calculate the joints velocities
    dq(:,k)=S*Wmatrix*eta(:,k);
    
    %% update variables for next iteration       
    if k < N
        %Calculate the joint values
        q(:,k+1)=q(:,k)+dq(:,k)*ts;
        
        %Calculate the position of the end effector
        T=MARS.forwardKin(q(:,k+1));
        xi(1:3,k+1)=T(1:3,4);
        Re=T(1:3,1:3);
        quat_e=cartToQuat(Re);
        xi(4:7,k+1)=quat_e;
        
        Rk(:,:,k)=T(1:3,1:3);
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
fprintf('\nDesired Final Pose');
xi_des(:,k)'
fprintf('\nObtained Final Pose');
xi(:,k)'

%xi_pos_error=xi_des(1:3,:)-xi(1:3,:);
fprintf('\nFinal Position Error:');
xi_pos_error(:,end)'
fprintf('Pos norm error: %fmm\n',norm(xi_pos_error(:,end))*1000');

fprintf('\nFinal Orientation Error');
%xi_orient_error=errorFromQuats(xi_des(4:7,:),xi(4:7,:));
xi_orient_error(:,end)'
fprintf('Orientation norm error: %f\n',norm(xi_orient_error(:,end))');

fprintf('\nDesired Final Transformation Matrix\n');
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

%% Plot elbow collision distance
figure()
plot(time(1:k),dist_elbow(1:k),'b','LineWidth',1.5); hold on;
xlabel('time(s)')
title('Distance elbow to mob plat [m]')
grid on

%% Plot wrist collision distance
figure()
plot(time(1:k),dist_wrist(1:k),'b','LineWidth',1.5); hold on;
xlabel('time(s)')
title('Distance wrist to front of mob plat [m]')
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

%% Save the redundancy resolution position and velocities
% JointMotion.q_des=q;
% JointMotion.dq_des=dq;
% JointMotion.ts=ts;
% JointMotion.tf=tf;
% uisave('JointMotion','JointMotion.mat');