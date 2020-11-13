clear all
addpath UR5Kin
addpath TrajPlan
addpath 3DPlots

robot = UR5Robot();

% T0e = robot.forwardKin(q);
% pos = T0e(1:3,4)'
% quat = rotmat2quatROS(T0e(1:3,1:3))
% 
% % Calculate the inverse kinematics
% sols = robot.inverseKin(T0e)

ts=1/20;
tf=12;

%% Initial joints values
q0 = [0.5818229;4.2901589;1.6468228;-1.3006193;-1.7442122; 0];
T0 = robot.forwardKin(q0);
Pos_0=T0(1:3,4);
quat_0=cartToQuat(T0(1:3,1:3));

%% Get the desired pose transformation matrix %%%
Pos_f = [-0.5; -0.5; 0.5];
quat_f = [0.0160; 0.160; -0.7920; 0.6102];

Rf=quat2rotm(quat_f');
Tf=zeros(4,4);
Tf(4,4)=1;
Tf(1:3,1:3)=Rf;
Tf(1:3,4)=Pos_f;
T0
Tf

% plotFrame(T0,1,'T0'); hold on;
% plotFrame(Tf,1,'Tf'); 
% xlabel('x')
% ylabel('y')
% zlabel('z')
% view(-133,31)
% pause()

%% Trajectory planning
tic
disp('Calculating the trajectory...')

MotPlan = struct([]);
%Use the trajectory planning function
MotPlan=TrajPlanPol(Pos_0,quat_0,Pos_f,quat_f,ts,tf);

%% Initializae variables
%Set the number of iterations from the motion planning data
N=size(MotPlan.x,2);
xi_des = zeros(7,N);
xi_des(1,:)=[MotPlan.x];
xi_des(2,:)=[MotPlan.y];
xi_des(3,:)=[MotPlan.z];
xi_des(4:7,:)=[MotPlan.quat];
q=zeros(6,N);
q(:,1) = q0;

%% Use the inverse kinematics for each pose in the motion planning

for i=1:N
    T = posQuat2RotMat(xi_des(:,i));
     % Find the IK closest to the last joint positions
    sol = robot.closestIK(T, q(:,i))
    pause()
    q(:,i+1) = sol;
end























