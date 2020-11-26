clear all
addpath UR5Kin
addpath RobotPaths
addpath TrajPlan
addpath 3DPlots

robot = UR5Robot();

%Name of the test
testN='LissajousPath';

ts=1/20;

%% Initial joints values
% Add mobile platform joint values too (Starting point of the painting task)
tx=0;
ty=0;
phi_mp=0;
tz=0.2;
mp_pos = [tx; ty; phi_mp; tz];

JointConstraints

qa = deg2rad([0;-80;110;-120;-90;0.0]);
T0 = robot.forwardKin(qa);
Pos_0=T0(1:3,4)
quat_0=cartToQuat(T0(1:3,1:3))


%% Trajectory planning
tic
disp('Calculating the trajectory...')

MotPlan = struct([]);
%Use the trajectory planning function
MotPlan=LissajousPath2(T0,30,ts,5);
time=MotPlan.time;

%% Initialize variables
%Set the number of iterations from the motion planning data
N=size(MotPlan.x,2);
xi_des = zeros(7,N);
xi_des(1,:)=[MotPlan.x];
xi_des(2,:)=[MotPlan.y];
xi_des(3,:)=[MotPlan.z];
xi_des(4:7,:)=[MotPlan.quat];
xi_pos_error(1:3,1) = zeros(3,1);
xi_orient_error(1:3,1) = zeros(3,1);

% Store the starting joint angles
q=zeros(10,N);
q(1:4) = mp_pos;
q(5:10,1) = qa;

%% Use the inverse kinematics for each pose in the motion planning
disp('Finding the IK of each point in the trajectory...')
for i=1:N-1
    T = posQuat2RotMat(xi_des(:,i));
     % Find the IK closest to the last joint positions
    sol = robot.closestIK(T, q(5:10,i));
    
    %%%%%%%%%%%%%Calculate the position and orientation error%%%%%%%%%%%%
    % Get the current transformation matrix
    T_i = robot.forwardKin(sol);
    pos_i = T_i(1:3,4);
    quat_i = rotm2quat(T_i(1:3,1:3))';
    
    %Position error
    xi_pos_error(1:3,i+1)=xi_des(1:3,i)-pos_i;    
    
    %Orientation error    
    xi_orient_error(1:3,i+1)=errorFromQuats(xi_des(4:7,i),quat_i);
    
    q(:,i+1) = [mp_pos;sol];
end

%% Show the obtained final pose
TfObtained = robot.forwardKin(q(5:10,end));

PlotEvolutionPretty
