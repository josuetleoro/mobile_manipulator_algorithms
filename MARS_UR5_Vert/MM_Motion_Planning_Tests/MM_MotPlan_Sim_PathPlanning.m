clear all
close all

%Additional paths
addpath MARS_UR5
addpath TrajPlan
addpath JacobiansFolder
addpath 3DPlots

%Create a MMUR5 object
MARS=MARS_UR5();

%Load the test point
testN=12;
TestPointsMaxLinVel

%Joints' angles with maximum manipulability for UR5
%qa=[0.0;-0.40;1.06;5*pi/4;-pi/2;0.0];

%Load the joints constraints
JointConstraints

%% Set the step size for the gradient descent method and error weight.
%A higher error weight might decrease the manipulability because of its
%influence on the motion.

ts=1/20;    %Sampling time
alpha=12;
beta=1;
trans_step=0.005;
epsilon=5e-4;

Kp_pos=1;
Ki_pos=0;   %Ki=50 for traj plan parabolic blending Ki=0 for FifthOrder
Kp_or=2;

%% Initial values of the generalized coordinates of the MM
q0=[tx;ty;phi_mp;tz;qa];
%Find the initial position of the end effector
T0=MARS.forwardKin(q0);
%pause()
Pos_0=T0(1:3,4);
quat_0=cartToQuat(T0(1:3,1:3));
xi_0=[Pos_0;quat_0];

%% Get the desired pose transformation matrix %%%
Rf=quat2rotm(quat_f');
Tf=zeros(4,4);
Tf(4,4)=1;
Tf(1:3,1:3)=Rf;
Tf(1:3,4)=Pos_f;

xi_des(1:3,1)=Tf(1:3,4);
xi_des(4:7,1)=cartToQuat(Tf(1:3,1:3));

T0
Tf

%%%%%%%%%% Show the MM frames in 3D %%%%%%%%%%
% figure()
% %Mobile platform initial pos
% %plotFrame(eye(4),1,'mp_0'); hold on;
% plotMobPlatArrow(tx,ty,phi_mp,2,'mp0');
% plotFrame(T0,1,'T0'); hold on;
% plotFrame(Tf,1,'Tf'); 
% %plotFrame(Tf2,2,'Tf2');
% xlabel('x')
% ylabel('y')
% zlabel('z')
% view(-133,31)
% pause()

%% Initialize variables
tic
% q=zeros(10,N);
% xi=zeros(7,N);
% J = zeros(6,9);
% eta=zeros(9,N);
% dq=zeros(10,N);
% W_measure=zeros(1,N);
% MM_man_measure=zeros(1,N);
% ur5_man_measure=zeros(1,N);

%The error weighting matrices
Werror=zeros(6,6);
Werror(1:3,1:3)=Kp_pos*eye(3);
Werror(4:6,4:6)=Kp_or*eye(3);

Wierror=zeros(6,6);
Wierror(1:3,1:3)=Ki_pos*eye(3);

clear prevGradH prevGradPElbow prevGradPWrist;

%Identity matrix of size delta=9, delta=M-1 =>10DOF-1
Id=eye(9);

%Create non-holonomic constrains matrix
S=zeros(10,9);
S(3:end,2:end)=eye(8);

%Copy the initial values of the motion planning
q(:,1)=q0;
xi(:,1)=xi_0;
quat_e=xi_0(4:7,1);

%% Calculate the maximum velocity normalization matrix
Tq=zeros(9,9);
for i=1:9
    Tq(i,i)=1/sqrt(dq_limit(i));
end
invTq=inv(Tq);

disp('Calculating the inverse velocity kinematics solution')
k=1;
errorPrev=zeros(6,1);
ierror=zeros(6,1);
derror=zeros(6,1);
error_cont=zeros(6,1);
xi_pos_error(1:3,k)=xi_des(1:3)-xi(1:3,k);
error = norm(xi_pos_error);
time(1)=0;
trans=0;
trans2 = 1;
while(error>epsilon)
    %% Redundancy resolution using manipulability gradient
    fprintf('Step %d\n',k);
    
    %Replace the elements cos(phi) and sin (phi)
    S(1,1)=cos(q(3,k)); S(2,1)=sin(q(3,k));
    
    %Calculate the Jacobian
    JBar=evaluateJBar(q(3,k),q(5,k),q(6,k),q(7,k),q(8,k),q(9,k));    
        
    %% Manipulability gradient
    [MM_dP,MM_manip, ur5_dP, ur5_manip]=manGradJBar2(q(:,k),JBar);   
    MM_man_measure(k)=MM_manip;
    ur5_man_measure(k)=ur5_manip;
    
    dP=ur5_manip*MM_dP+MM_manip*ur5_dP;                                     %Combined Mobile manipulator and robot arm
    %dP=ur5_dP;
    W_measure(k)=MM_manip*ur5_manip;
    dP=S'*dP;
    
    %% Joint limit cost function gradient
    Wjlim=jLimitGrad(q(:,k),q_limit);
    %Wjlim=eye(9,9);
    
    %% Collision avoidance weighting matrices
    [Wcol_elbow, dist_elbow(k)]=elbowColMat(q(:,k),0.001,50,1);
    [Wcol_wrist, dist_wrist(k), wrist_pos(:,k)]=wristColMat(q(:,k),0.001,50,1);
    Wcol=Wcol_elbow*Wcol_wrist;
    %Wcol_elbow
    %Wcol=eye(9,9);
    
    %% Inverse differential kinematics         
    %%%%%%%%%%%%%Calculate the position and orientation error%%%%%%%%%%%%
    %Position error
    eP=xi_des(1:3)-xi(1:3,k);
    xi_pos_error(1:3,k)=eP;
    error = norm(eP)
    
    %Orientation error
    quat_d=xi_des(4:7);    
    eO=errorFromQuats(quat_d,quat_e);
    %eO=errorFromQuatsR(quat2rotm(quat_d'),quat2rotm(quat_e'));
    xi_orient_error(1:3,k)=eO;
        
    errorRate(1:3,1)=eP;
    errorRate(4:6,1)=eO;
    ierror(1:3)=ierror(1:3)+eP*ts;
    ierror(4:6)=zeros(3,1);
    derror(1:3)=(eP-errorPrev(1:3))/ts;
    errorPrev=errorRate;
    error_cont=Werror*errorRate+Wierror*ierror;
    %%%%%%%%%%Calculate the control input and internal motion%%%%%%%%%%%%%
    Wmatrix=Wcol*Wjlim*invTq;
    JBar_w=JBar*Wmatrix;
    inv_JBar_w=pinv(JBar_w);
    cont_input=inv_JBar_w*(error_cont);    
    
    %Calculate the weighting by task velocity
    %taskNorm=abs(max(dxi_des(1:3,k)./maxdxi(1:3)));
    %Use sigmoid trapezoidal for manipulability
    %taskNorm=trans(k);
    %if (k> N/2)
    %    taskNorm=trans(k);
    %    %taskNorm=abs(max(dxi_des(1:3,k)./maxdxi(1:3)));
    %else
    %    taskNorm=abs(max(dxi_des(1:3,k)./maxdxi(1:3)));
    %end
    %dP=taskNorm*dP;    
    %dP=trans(k)*dP;

    %Calculate the internal motion
    dP=Wmatrix*dP;
    int_motion=(Id-inv_JBar_w*JBar_w)*dP;
    
    cont_input=Wmatrix*cont_input;
    int_motion=Wmatrix*int_motion;
    
    aux = JBar*JBar'*errorRate;
    beta = dot(errorRate,aux)/(aux'*aux);
    
    if error < 0.1
       trans2 = trans2 - 0.01;
    end
    if trans2 < 0
        trans2 = 0;
    end            
    
    %Mobility control vector
    eta(:,k)=trans*(beta*cont_input+trans2*alpha*int_motion);    
    % Find scale factor to avoid exceeding maximum velocities
    velScale = jointVelLimScale(eta(:,k), dq_limit);
    eta(:,k)= velScale * eta(:,k);    
    
    if trans < 1
        trans = trans + trans_step;
    end   
    
    %Calculate the joints velocities
    dq(:,k)=S*eta(:,k);
    
    %% update variables for next iteration       
    if error > epsilon
        %Calculate the joint values
        q(:,k+1)=q(:,k)+dq(:,k)*ts;
        
        %Calculate the position of the end effector
        T=MARS.forwardKin(q(:,k+1));
        xi(1:3,k+1)=T(1:3,4);
        Re=T(1:3,1:3);
        quat_e=cartToQuat(Re);
        xi(4:7,k+1)=quat_e;        
        time(k+1)=time(k)+ts;
    end
    %increment iteration step
    k=k+1;
    
end
toc
k=k-1;
% if k==N
%     disp('Task completed')
% else
%     disp('Task cannot be executed')
%     k
%     N
% end

%Error of the end effector position
fprintf('\nDesired Final Pose');
xi_des(:,end)'
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

%Trajectory time
fprintf('\nTrajectory time');
time(end)

fprintf('\nDesired Final Transformation Matrix\n');
Tf

fprintf('Obtained Final Transformation Matrix\n');
TfObtained(:,4)=[xi(1,k);xi(2,k);xi(3,k);1];
TfObtained(1:3,1:3)=quatToRotMat(xi(4:7,k)');
TfObtained

% if k < N
%     MM_man_measure = MM_man_measure(1:k);
%     ur5_man_measure = ur5_man_measure(1:k);
%     maxAlpha = maxAlpha(1:k);
% end

% Adjust the manipulability measures
MM_man_measure(1)=MM_man_measure(2);
MM_man_measure(end)=MM_man_measure(end-1);
ur5_man_measure(1)=ur5_man_measure(2);
ur5_man_measure(end)=ur5_man_measure(end-1);
% Store the mobile platform velocities
mp_vel=eta(1:3,:);

%% Plot variables to analyze manipulability
% figure()
% plot(time,maxAlpha,'b','LineWidth',1.5);
% grid on
% xlabel('time(s)')
% title('Max step size')
% 
% 
% figure()
% subplot(3,1,1)
% plot(time,MM_man_measure,'b','LineWidth',1.5); hold on;
% plot(time,ur5_man_measure,'r','LineWidth',1.5);
% plot(time,W_measure,'g','LineWidth',1.5); hold off
% grid on
% legend('MM_{manip}','UR5_{manip}','W_{manip}')
% xlabel('time(s)')
% title('Manipulability measure')
%% Plot all the variables
PlotEvolutionPretty
