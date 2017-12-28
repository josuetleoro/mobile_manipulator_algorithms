clear all
close all

%Add the MMUR5 path to use the class MMUR5
addpath MARS_UR5 sns
%Create a MMUR5 object
MARS=MARS_UR5();

%% Define joints constrains
q_limit=[-Inf, Inf;
         -Inf, Inf;
           0.0, 0.8;
          -2*pi, 2*pi;
          -22*pi/12, 22*pi/12;
          -22*pi/12, 22*pi/12;
          -22*pi/12, 22*pi/12;
          -22*pi/12, 22*pi/12;
          -22*pi/12, 22*pi/12];

dq_limit=[0.8;
          pi/3;
          0.1;
          pi;
          pi;
          pi;
          pi;
          pi;
          pi];
      
ddq_limit=[1.0;
           4.0; %4.0; 
           0.1;
           pi/6;
           pi/6;
           pi/6;
           pi/6;
           pi/6;
           pi/6];

%% Choose the initial joint values, final poistion and times
%%%%%%%%Best final position and orientation%%%%%%%%%
%Initial joints values
% tx=0;
% ty=0;
% phi_mp=0;
% tz=0;
% qa=[0;-pi/4;pi/4;0;pi/2;0];
% %initial values of the generalized coordinates of the MM
% q(:,1)=[tx;ty;phi_mp;tz;qa];
% %Find the initial position of the end effector
% T0=MM.forwardKin(q(:,1));
% 
% %Final joints values
% Pos=[2;3;1.0];
% phi=90*pi/180;
% theta=-90*pi/180;
% psi=-90*pi/180;
% tf=12;          %Desired final time
% ts=0.01;       %time step
% tb=2;          %Blending time
% lambda = 5;    %error weight  

%%%%%%%%%%%%%%%%%% Test 1 (working) %%%%%%%%%%%%%%%%%%%%%%%
% %Initial joints values
% tx=0;
% ty=0;
% phi_mp=0;
% tz=0;
% qa=[0;-pi/4;pi/4;0;pi/2;0];
% %qa=[pi;-pi/4;pi/4;0;pi/2;0]; 
% 
% %initial values of the generalized coordinates of the MM
% q(:,1)=[tx;ty;phi_mp;tz;qa];
% %Find the initial position of the end effector
% T0=MM.forwardKin(q(:,1));
% 
% %Final joints values
% Pos=[-3;3;1.3];
% phi=60*pi/180;
% theta=-30*pi/180;
% psi=60*pi/180;
% tf=10;          %Desired final time
% ts=0.05;       %time step
% tb=2;          %Blending time
% lambda = 5;    %error weight  

%%%%%%%%%%%%%%%%%% Test 2 (working) %%%%%%%%%%%%%%%%%%%%%%%
% %Initial joints values
% tx=0;
% ty=0;
% phi_mp=0;
% tz=0;
% qa=[0;-pi/4;pi/4;0;-pi/2;0]; %theta1=pi/2 is problematic
% %initial values of the generalized coordinates of the MM
% q(:,1)=[tx;ty;phi_mp;tz;qa];
% %Find the initial position of the end effector
% T0=MM.forwardKin(q(:,1));
% 
% %Final joints values
% Pos=[3;3;0.8];
% phi=0*pi/180;
% theta=0*pi/180;
% psi=0*pi/180;
% tf=15;          %Desired final time
% ts=0.05;       %time step
% tb=3;          %Blending time
% lambda = 5;    %error weight  

%%%%%%%%%%%%%%%%%% Test 3 %%%%%%%%%%%%%%%%%%%%%%%
%Initial joints values
tx=0;
ty=0;
phi_mp=0;
tz=0;
%qa=[0;-pi/4;pi/4;0;-pi/2;0];
qa=[pi;-pi/4;pi/4;0;pi/2;0];
%initial values of the generalized coordinates of the MM
q(:,1)=[tx;ty;phi_mp;tz;qa];
%Find the initial position of the end effector
T0=MARS.forwardKin(q(:,1));

Pos=[2;3;1.2];
phi=120*pi/180;
theta=-90*pi/180;
psi=-45*pi/180;
tf=35;          %Desired final time
ts=0.05;         %time step
tb=5;           %Blending time
lambda = 5;    %error weight

%%%%%%%%%% Test 4 (Problematic orientation) %%%%%%%% 
% %Initial joints values
% tx=0;
% ty=0;
% phi_mp=0;
% tz=0;
% qa=[0;-pi/4;pi/4;0;-pi/2;0];% %initial values of the generalized coordinates of the MM
% q(:,1)=[tx;ty;phi_mp;tz;qa];
% %Find the initial position of the end effector
% T0=MM.forwardKin(q(:,1));
% 
% Pos=[2;3;1.5];
% phi=120*pi/180;
% theta=-90*pi/180;
% psi=-45*pi/180;
% tf=30;          %Desired final time
% ts=0.05;         %time step
% tb=5;           %Blending time
% lambda = 0;    %error weight

%%%%%%%% Get the desired transformation matrix %%%%%%
%RF=eulerToRotMat(phi,theta,psi,'ZYZ')
RF=eul2rotm([phi theta psi],'ZYZ')
Tf=zeros(4,4);
Tf(4,4)=1;
Tf(1:3,1:3)=RF;
Tf(1:3,4)=Pos;

Euler0=rotm2eul(T0(1:3,1:3),'ZYZ')*180/pi
T0
Tf

%%%%%%%%%% Show the MM frames in 3D %%%%%%%%%%
figure()
%Mobile platform initial pos
plotMobPlatArrow(tx,ty,phi_mp,2,'mp_0');
plotFrame(T0,2,'T0'); hold on;
plotFrame(Tf,2,'Tf')
xlabel('x')
ylabel('y')
zlabel('z')
view(-133,31)

pause()
%% Trajectory planning
tic
disp('Calculating the trajectory...')
%Use the trajectory planning function
MotPlan=TrajPlanQuat(T0,Tf,ts,tb,tf);


%% Projected gradient method

dq(:,1)=zeros(10,1);
mp_vel(:,1)=zeros(3,1);
man_measure(1)=0;

%Identity matrix of size delta=9, delta=M-1 =>10DOF-1
Id=eye(9);

%Create non-holonomic constrains matrix
S=zeros(10,9);
S(3:end,2:end)=eye(8);

%The weight matrix W
Werror=lambda*eye(6);
% Werror(1:3,1:3)=5*Werror(1:3,1:3);
% Werror(4:6,4:6)=5*Werror(4:6,4:6);

%Set the step size
alpha=0.05;  % Best: k=0.05;

%Set the number of iterations from the motion planning data
N=size(MotPlan.x,2);

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

k=1;
%Copy the initial position and orientation
xi(:,1)=xi_des(:,1);
qe=xi(4:7,1);
quat(:,1)=qe;

%Get the number of degrees of freedom and the task dimension
n=9;
m=length(xi_des);
JBar = zeros(6,9);
dp=zeros(10,1);
scale=1.0;
disp('Calculating the inverse velocity kinematics solution')
while(k<N)
    %% Redundancy resolution using manipulability gradient
    %fprintf('Step %d of %d\N',k,N);
    
    %Replace the elements cos(phi) and sin (phi)
    S(1,1)=cos(q(3,k)); S(2,1)=sin(q(3,k));
    
    %Pseudoinverse of JBar
    JBar=evaluateJBar(q(3,k),q(5,k),q(6,k),q(7,k),q(8,k),q(9,k));
        
    %%%%%%%%%%%%%%Calculate the position and orientation error%%%%%%%%%%%%
    %Position error
    eP=xi_des(1:3,k)-xi(1:3,k);
    %Orientation error
    qd=xi_des(4:7,k);    
    eO=errorFromQuats(qd,qe);  
    
    errorRate(1:3,1)=eP;
    errorRate(4:6,1)=eO;
    %errorRate(1:6,1)=zeros(6,1);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
    %Manipulability gradient
    [dP,manip]=manGrad(q(:,k),JBar);   
    inv_JBar=pinv(JBar);
    man_measure(k)=manip;
    
    %Calculate the control input and internal motion
    error_cont=Werror*errorRate;
    cont_input=inv_JBar*(scale*dxi_des(:,k)+error_cont);
    projM=(Id-inv_JBar*JBar);
    dq_N = alpha*S'*dP;
    %dq_N = zeros(9,1);
    int_motion=projM*dq_N;
    
    %Mobility control vector
    eta(:,k)=cont_input+int_motion;
    
    %% SNS Algorithm
    k
    %Check if the control vector satisfy the joints box contraints
    %Mobile platform
    if k == 1
        [dQminMobPlat, dQmaxMobPlat]=shapeJointVelBoundMobPlat(eta(1:2,k),zeros(2,1),dq_limit(1:2),ddq_limit(1:2),ts);
    else
        [dQminMobPlat, dQmaxMobPlat]=shapeJointVelBoundMobPlat(eta(1:2,k),eta(1:2,k-1),dq_limit(1:2),ddq_limit(1:2),ts);
    end
    dQmin(1:2,k)=dQminMobPlat;
    dQmax(1:2,k)=dQmaxMobPlat;
        
    %Robot arm
    [dQminArm, dQmaxArm, idxMin, idxMax]=shapeJointVelBoundArm(q(4:10,k),q_limit(3:9,:),dq_limit(3:9),ddq_limit(3:9),ts);
    dQmin(3:9,k)=dQminArm;
    dQmax(3:9,k)=dQmaxArm;

%     min=dQmin(:,k);
%     max=dQmax(:,k);
%     eta
%     pause()
    limit_exceeded = false;
    for i=1:9
        if (eta(i,k)<dQmin(i,k)) || (eta(i,k)>dQmax(i,k))
            limit_exceeded = true;
%             dQmin(:,k)
%             dQmax(:,k)
%             eta(:,k)            
%             disp('Joint bound exceeded');
%             i
%             pause()
        end
    end
    
    if limit_exceeded
%         dQmin(:,k)
%         idxMin
%         dQmax(:,k)
%         idxMax
%         q(:,k)
%         eta(:,k)
%         pause()
        
        [eta_sns,scale,W]=sns_mm(dQmin(:,k),dQmax(:,k),dq_N,JBar,dxi_des(:,k),error_cont); 
%         dQmin(:,k)
%         dQmax(:,k)
%         min=dQmin(:,k)
%         max=dQmax(:,k)
%         W
%         eta(:,k)
        if(scale<1.0)
            scale
            disp('scaling applied');
        end
        %pause
        eta(:,k)=eta_sns;
    end  
    if(scale <= 0)
        break;
    end
    %% update variables for next iteration
        
    %Calculate the joints velocities
    dq(:,k+1)=S*eta(:,k);
   
    %Calculate the joint values
    q(:,k+1)=q(:,k)+dq(:,k+1)*ts; 
        
    %Calculate the position of the end effector
    T=MARS.forwardKin(q(:,k+1));
    xi(1:3,k+1)=T(1:3,4);
    Re=T(1:3,1:3);
    qe=cartToQuat(Re);
    %qe=qe/norm(qe);
    xi(4:7,k+1)=qe;
    quat(:,k+1)=qe;   
    
    currentPos=xi(:,k+1);
    
    %Store the mobile platform velocities
    mp_vel(:,k+1)=eta(1:3,k);
    
    %increment the step
    k=k+1; 
    %pause()   
end 
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

xi_error=xi_des(:,k)-xi(:,k);
fprintf('Final Position Error\n');
xi_error(:,end)

fprintf('Desired Final Transformation Matrix\n');
Tf

fprintf('Obtained Final Transformation Matrix\n');
TfObtained(:,4)=[xi(1,end);xi(2,end);xi(3,end);1];
TfObtained(1:3,1:3)=quatToRotMat(xi(4:7,end)');
TfObtained

figure()
man_measure(1)=man_measure(2);
man_measure(k)=man_measure(end);
plot(time,man_measure,'LineWidth',1.5); grid on
%title('Manipulability');
xlabel('time(s)')
ylabel('Manipulability')

PlotEvolution

%% Plot the evolution of quat
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


%% Show the end effector motion in 3D
% Rd=zeros(4,4,length(time));
% %Form the T6Traj matrix
% for k=1:length(time)
%    Rd(:,4,k)=[xi(1,k);xi(2,k);xi(3,k);1];
%    Rd(1:3,1:3,k)=quatToRotMat(quat(:,k));  
%    %Rd(1:3,1:3,k)=quat2rotm(quat(:,k)');
% end
% figure()
% plotEndEffectorMotion(Rd,1)

%% Show the mobile platform and end effector motion in 3D
Rd=zeros(4,4,length(time));
%Form the T6Traj matrix
for k=1:length(time)
   Rd(:,4,k)=[xi(1,k);xi(2,k);xi(3,k);1];
   Rd(1:3,1:3,k)=quatToRotMat(quat(:,k));  
   %Rd(1:3,1:3,k)=quat2rotm(quat(:,k)');
end
figure()
plotMobileManipulatorMotion(q(1:3,:),Rd,1);

%% Save the redundancy resolution position and velocities
% JointMotion.q_des=q;
% JointMotion.dq_des=dq;
% JointMotion.ts=ts;
% JointMotion.tf=tf;
% uisave('JointMotion','JointMotion.mat');

