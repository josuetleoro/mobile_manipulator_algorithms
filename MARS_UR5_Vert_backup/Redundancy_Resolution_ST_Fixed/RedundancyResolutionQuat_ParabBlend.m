%clear all
close all

%Add the MMUR5 path to use the class MMUR5
addpath MARS_UR5
%Add path for the UR5 manipulability
addpath UR5_manip

%Create a MMUR5 object
MARS=MARS_UR5();

%Load the test point
testN=3;
TestPoints

%Set the step size for the gradient descent method and error weight. The 
%best results with mobile manipulator manipulability use a low error weight

%With Fs=20Hz
ts=0.05;  %Overwrite ts
alpha=2.5;  %Best alpha=3.5
lambda=1.0; %Overwrite lambda best=0.05

% %With Fs=100Hz
% ts=0.01;  %Overwrite ts
% alpha=3.5;
% lambda=0.05; %Overwrite lambda best=0.05

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
MotPlan=TrajPlanQuatParabBlend(T0,Tf,ts,tb,tf);
%MotPlan=TrajPlanQuat(T0,Tf,ts,tb,tf);
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

%The weight matrix W
Werror=lambda*eye(6);
Werror(4:6,4:6)=1*Werror(4:6,4:6);

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

disp('Calculating the inverse velocity kinematics solution')
k=1;
while(k<N)
    %% Redundancy resolution using manipulability gradient
    fprintf('Step %d of %d\n',k,N);
    
    %Replace the elements cos(phi) and sin (phi)
    S(1,1)=cos(q(3,k)); S(2,1)=sin(q(3,k));
    
    %Calculate the Jacobian
    JBar=evaluateJBar(q(3,k),q(5,k),q(6,k),q(7,k),q(8,k),q(9,k));
        
    %Manipulability gradient
    [MM_dP,MM_manip, ur5_dP, ur5_manip]=manGrad(q(:,k),JBar);   
    MM_man_measure(k)=MM_manip;
    ur5_man_measure(k)=ur5_manip;
    
    dP=MM_dP*ur5_manip+ur5_dP*MM_manip;
    %dP=0.2*MM_manip+0.8*ur5_dP;
    
%     %Select the manipulability to use
%     if MM_manip_sel == 1
%         %Use MM manipulability
%         dP=MM_dP;
%     else
%         %Use the ur5 manipulability only
%         dP=ur5_dP;
%     end
            
    %%%%%%%%%%%%%Calculate the position and orientation error%%%%%%%%%%%%
    %Position error
    eP=xi_des(1:3,k)-xi(1:3,k);
    
    %Orientation error
    quat_d=xi_des(4:7,k);    
    eO=errorFromQuats(quat_d,quat_e);  
        
    errorRate(1:3,1)=eP;
    errorRate(4:6,1)=eO;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
    
    %Calculate the control input and internal motion
    error_cont=Werror*errorRate;
    inv_JBar=pinv(JBar);
    cont_input=inv_JBar*(dxi_des(:,k)+error_cont);
    projM=(Id-inv_JBar*JBar);
    dq_N = alpha*dP;         %(De Luca A., et al., Kin and Modeling and Redun. of NMM)
    int_motion=projM*dq_N;
        
    %Mobility control vector
    eta(:,k)=cont_input+int_motion;    
    
    %% update variables for next iteration
        
    %Calculate the joints velocities
    dq(:,k)=S*eta(:,k);
   
    %Calculate the joint values
    q(:,k+1)=q(:,k)+dq(:,k)*ts; 
        
    %Calculate the position of the end effector
    T=MARS.forwardKin(q(:,k+1));
    xi(1:3,k+1)=T(1:3,4);
    Re=T(1:3,1:3);
    quat_e=cartToQuat(Re);
%     if(quat_e(1)<0)
%         quat_e=quat_e*-1;
%     end
    %quat_e=rotm2quat(Re)';
    xi(4:7,k+1)=quat_e;
    
    %increment the step
    k=k+1; 
    %pause()   
end 
dq(:,k)=dq(:,k-1);
toc
%time=MotPlan.time;
k
N
if k==N
    disp('Task completed')
else
    disp('Task cannot be executed')
end

time=MotPlan.time(1:k);

%Error of the end effector position
%fprintf('Desired Final Position\n');
xi_des(:,k)
%fprintf('Obtained Final Position\n');
xi(:,k)

xi_pos_error=xi_des(1:3,k)-xi(1:3,k);
fprintf('Final Position Error\n');
xi_pos_error(:,end)
fprintf('Norm error: %f\n',norm(xi_pos_error(:,end)));

fprintf('Desired Final Transformation Matrix\n');
Tf

fprintf('Obtained Final Transformation Matrix\n');
TfObtained(:,4)=[xi(1,end);xi(2,end);xi(3,end);1];
TfObtained(1:3,1:3)=quatToRotMat(xi(4:7,end)');
TfObtained

%Adjust manipulability measure values
MM_man_measure(1)=MM_man_measure(2);
MM_man_measure(end)=MM_man_measure(end-1);
ur5_man_measure(1)=ur5_man_measure(2);
ur5_man_measure(end)=ur5_man_measure(end-1);

%Store the mobile platform velocities
mp_vel=eta(1:3,:);

PlotEvolution

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

%% Show the mobile platform and end effector motion in 3D
% Rd=zeros(4,4,length(time));
% %Form the T6Traj matrix
% for k=1:length(time)
%    Rd(:,4,k)=[xi(1,k);xi(2,k);xi(3,k);1];
%    Rd(1:3,1:3,k)=quatToRotMat(quat(:,k));  
%    %Rd(1:3,1:3,k)=quat2rotm(quat(:,k)');
% end
% figure()
% plotMobileManipulatorMotion(q(1:3,:),Rd,1);

%% Save the redundancy resolution position and velocities
% JointMotion.q_des=q;
% JointMotion.dq_des=dq;
% JointMotion.ts=ts;
% JointMotion.tf=tf;
% uisave('JointMotion','JointMotion.mat');

