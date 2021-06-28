close all
clear all
clc

%Open a dialog box to look for the motion data
uiopen('*.mat');

%% Mobile platform

%Determine the number of iterations
n=size(q_mp,2);
 
joints_names = {'right_wheel','left_wheel','prismatic_joint','ur5_shoulder_pan_joint','ur5_shoulder_lift_joint'...
    'ur5_elbow_joint','ur5_wrist_1_joint','ur5_wrist_2_joint','ur5_wrist_3_joint'};

%rosinit

%Get the transformation tree
tftree = rostf;
pause(1);
%Create the transformation message
odom_trans = rosmessage('geometry_msgs/TransformStamped');
odom_trans.ChildFrameId = 'base_footprint';
odom_trans.Header.FrameId = 'odom';

%Create the path message
[path_pub, path_msg] = rospublisher('/curr_path','nav_msgs/Path');
path_msg.Header.FrameId = 'odom';

% Find the dividend to use so that at least 100 points are always used for the path
pathDiv = 2;
pathNPoints = 100;
if n > pathNPoints
    pathDiv = floor(n/pathNPoints);
end

path_msg.Poses = arrayfun(@(~) rosmessage('geometry_msgs/PoseStamped'),zeros(1,ceil(n/pathDiv)));        

k=1;
for i=1:n
    if(mod(i,pathDiv)==0)
        path_msg.Poses(k).Pose.Position.X = q_mp(1,i);
        path_msg.Poses(k).Pose.Position.Y = q_mp(2,i);
        path_msg.Poses(k).Pose.Position.Z = 0;
        quatrot = eul2quat([q_mp(3,i) 0 0],'ZYZ');
        path_msg.Poses(k).Pose.Orientation.W = quatrot(1);
        path_msg.Poses(k).Pose.Orientation.X = quatrot(2);
        path_msg.Poses(k).Pose.Orientation.Y = quatrot(3);
        path_msg.Poses(k).Pose.Orientation.Z = quatrot(4);
        k=k+1;
    end
end
%Make sure the last pose is also stored
path_msg.Poses(end).Pose.Position.X = q_mp(1,end);
path_msg.Poses(end).Pose.Position.Y = q_mp(2,end);
path_msg.Poses(end).Pose.Position.Z = 0;
quatrot = eul2quat([q_mp(3,i) 0 0],'ZYZ');
path_msg.Poses(end).Pose.Orientation.W = quatrot(1);
path_msg.Poses(end).Pose.Orientation.X = quatrot(2);
path_msg.Poses(end).Pose.Orientation.Y = quatrot(3);
path_msg.Poses(end).Pose.Orientation.Z = quatrot(4);

%Create the transformation of the last pose to draw it
desired_trans = rosmessage('geometry_msgs/TransformStamped');
desired_trans.ChildFrameId = 'Desired_pos';
desired_trans.Header.FrameId = 'odom';
desired_trans.Transform.Translation.X = mp_des(1);
desired_trans.Transform.Translation.Y = mp_des(2);
desired_trans.Transform.Translation.Z = 0;
quatrot = eul2quat([mp_des(3) 0 0],'ZYZ');
desired_trans.Transform.Rotation.W = quatrot(1);
desired_trans.Transform.Rotation.X = quatrot(2);
desired_trans.Transform.Rotation.Y = quatrot(3);
desired_trans.Transform.Rotation.Z = quatrot(4);

[state_pub, state_msg] = rospublisher('/mars_joint_states','sensor_msgs/JointState');

%Use stoploop function to wait for ok button
FS = stoploop({'Press Ok to start the simulation'}); 

%Initial Joints values
state_msg.Name = joints_names;
state_msg.Position = [0.0; 0.0; q_mp(4,1); q_mp(5,1); q_mp(6,1); q_mp(7,1); q_mp(8,1); q_mp(9,1); q_mp(10,1)];
state_msg.Velocity = zeros(9,1);
state_msg.Effort = zeros(9,1);
rate = rosrate(20);
reset(rate);
%Initial odom transformation
odom_trans.Transform.Translation.X = q_mp(1,1);
odom_trans.Transform.Translation.Y = q_mp(2,1);
odom_trans.Transform.Translation.Z = 0.0;
quatrot = eul2quat([q_mp(3,1) 0 0],'ZYZ');
odom_trans.Transform.Rotation.W = quatrot(1);
odom_trans.Transform.Rotation.X = quatrot(2);
odom_trans.Transform.Rotation.Y = quatrot(3);
odom_trans.Transform.Rotation.Z = quatrot(4);
while (~FS.Stop())   
    %Publish the joint states and transformations
    now = rostime('now');
    state_msg.Header.Stamp = now;
    odom_trans.Header.Stamp = now;
    desired_trans.Header.Stamp = now;
    path_msg.Header.Stamp = now;    
    send(state_pub,state_msg);
    send(path_pub,path_msg);
    sendTransform(tftree, odom_trans);
    sendTransform(tftree, desired_trans);    
    waitfor(rate);    
end
FS.Clear()
clear FS;
clear rate

rate = rosrate(20);
reset(rate);
for i=1:n    
    time = rate.TotalElapsedTime;
    fprintf('Time Elapsed: %f\n',time)
    
    %Joints values
    state_msg.Name = joints_names;
    state_msg.Position = [0.0; 0.0; q_mp(4,i); q_mp(5,i); q_mp(6,i); q_mp(7,i); q_mp(8,i); q_mp(9,i); q_mp(10,i)];
    state_msg.Velocity = zeros(9,1);
    state_msg.Effort = zeros(9,1);
    
    %Mobile platform movement using the odom transformation
    now = rostime('now');
    odom_trans.Transform.Translation.X = q_mp(1,i);
    odom_trans.Transform.Translation.Y = q_mp(2,i);
    odom_trans.Transform.Translation.Z = 0.0;
    quatrot = eul2quat([q_mp(3,i) 0 0],'ZYZ');
    odom_trans.Transform.Rotation.W = quatrot(1);
    odom_trans.Transform.Rotation.X = quatrot(2);
    odom_trans.Transform.Rotation.Y = quatrot(3);
    odom_trans.Transform.Rotation.Z = quatrot(4);
    
    %Publish the joint states and transformations
    state_msg.Header.Stamp = now;
    odom_trans.Header.Stamp = now;
    desired_trans.Header.Stamp = now;
    send(state_pub,state_msg);
    sendTransform(tftree, odom_trans);
    sendTransform(tftree, desired_trans);    
    waitfor(rate);    
end

%% Robot arm
%Determine the number of iterations
n=size(q_arm,2);

% Find the dividend to use so that at least 100 points are always used for the path
pathDiv = 2;
pathNPoints = 100;
if n > pathNPoints
    pathDiv = floor(n/pathNPoints);
end

path_msg.Header.FrameId = 'ur5_base';
path_msg.Poses = arrayfun(@(~) rosmessage('geometry_msgs/PoseStamped'),zeros(1,ceil(n/pathDiv)));        

k=1;
for i=1:n
    if(mod(i,pathDiv)==0)
        path_msg.Poses(k).Pose.Position.X = xi_arm(1,i);
        path_msg.Poses(k).Pose.Position.Y = xi_arm(2,i);
        path_msg.Poses(k).Pose.Position.Z = xi_arm(3,i);
        path_msg.Poses(k).Pose.Orientation.W = xi_arm(4,i);
        path_msg.Poses(k).Pose.Orientation.X = xi_arm(5,i);
        path_msg.Poses(k).Pose.Orientation.Y = xi_arm(6,i);
        path_msg.Poses(k).Pose.Orientation.Z = xi_arm(7,i);
        path_msg.Poses(k).Header.FrameId = 'ur5_base';
        k=k+1;
    end
end
%Make sure the last pose is also stored
path_msg.Poses(end).Pose.Position.X = xi_arm(1,end);
path_msg.Poses(end).Pose.Position.Y = xi_arm(2,end);
path_msg.Poses(end).Pose.Position.Z = xi_arm(3,end);
path_msg.Poses(end).Pose.Orientation.W = xi_arm(4,end);
path_msg.Poses(end).Pose.Orientation.X = xi_arm(5,end);
path_msg.Poses(end).Pose.Orientation.Y = xi_arm(6,end);
path_msg.Poses(end).Pose.Orientation.Z = xi_arm(7,end);
path_msg.Poses(k).Header.FrameId = 'ur5_base';

%% Create the transformation of the last arm tool0 pose to draw it
desired_trans = rosmessage('geometry_msgs/TransformStamped');
desired_trans.ChildFrameId = 'Desired_pos';
desired_trans.Header.FrameId = 'ur5_base';
desired_trans.Transform.Translation.X = xi_arm(1,end);
desired_trans.Transform.Translation.Y = xi_arm(2,end);
desired_trans.Transform.Translation.Z = xi_arm(3,end);
desired_trans.Transform.Rotation.W = xi_arm(4,end);
desired_trans.Transform.Rotation.X = xi_arm(5,end);
desired_trans.Transform.Rotation.Y = xi_arm(6,end);
desired_trans.Transform.Rotation.Z = xi_arm(7,end);

[state_pub, state_msg] = rospublisher('/mars_joint_states','sensor_msgs/JointState');

%Initial Joints values
state_msg.Name = joints_names;
state_msg.Position = [0.0; 0.0; q_arm(4,1); q_arm(5,1); q_arm(6,1); q_arm(7,1); q_arm(8,1); q_arm(9,1); q_arm(10,1)];
state_msg.Velocity = zeros(9,1);
state_msg.Effort = zeros(9,1);

% Publish the path and tf of the starting position
path_msg.Header.Stamp = now;    
send(path_pub,path_msg);
desired_trans.Header.Stamp = now;
path_msg.Header.Stamp = now;    
send(path_pub,path_msg);
sendTransform(tftree, odom_trans);
sendTransform(tftree, desired_trans);

reset(rate);
for i=1:n    
    time = rate.TotalElapsedTime;
    fprintf('Time Elapsed: %f\n',time)
    
    %Joints values
    state_msg.Name = joints_names;
    state_msg.Position = [0.0; 0.0; q_arm(4,i); q_arm(5,i); q_arm(6,i); q_arm(7,i); q_arm(8,i); q_arm(9,i); q_arm(10,i)];
    state_msg.Velocity = zeros(9,1);
    state_msg.Effort = zeros(9,1);
    
    %Mobile platform movement using the odom transformation
    now = rostime('now');    
    %Publish the joint states and transformations
    state_msg.Header.Stamp = now;
    odom_trans.Header.Stamp = now;
    desired_trans.Header.Stamp = now;
    send(state_pub,state_msg);
    % The odom trans is same as the last mobile platform odom trans, no need to update
    sendTransform(tftree, odom_trans);
    sendTransform(tftree, desired_trans);    
    waitfor(rate);    
end
%% Mobile manipulator
%Determine the number of iterations
n=size(q_mm,2);

% Find the dividend to use so that at least 100 points are always used for the path
pathDiv = 2;
pathNPoints = 100;
if n > pathNPoints
    pathDiv = floor(n/pathNPoints);
end

path_msg.Header.FrameId = 'odom';
path_msg.Poses = arrayfun(@(~) rosmessage('geometry_msgs/PoseStamped'),zeros(1,ceil(n/pathDiv)));

% Initialize the path message poses
k=1;
for i=1:n
    if(mod(i,pathDiv)==0)
        path_msg.Poses(k).Pose.Position.X = xi_mm(1,i);
        path_msg.Poses(k).Pose.Position.Y = xi_mm(2,i);
        path_msg.Poses(k).Pose.Position.Z = xi_mm(3,i);
        path_msg.Poses(k).Pose.Orientation.W = xi_mm(4,i);
        path_msg.Poses(k).Pose.Orientation.X = xi_mm(5,i);
        path_msg.Poses(k).Pose.Orientation.Y = xi_mm(6,i);
        path_msg.Poses(k).Pose.Orientation.Z = xi_mm(7,i);
        k=k+1;
    end
end
%Make sure the last pose is also stored
path_msg.Poses(end).Pose.Position.X = xi_mm(1,end);
path_msg.Poses(end).Pose.Position.Y = xi_mm(2,end);
path_msg.Poses(end).Pose.Position.Z = xi_mm(3,end);
path_msg.Poses(end).Pose.Orientation.W = xi_mm(4,end);
path_msg.Poses(end).Pose.Orientation.X = xi_mm(5,end);
path_msg.Poses(end).Pose.Orientation.Y = xi_mm(6,end);
path_msg.Poses(end).Pose.Orientation.Z = xi_mm(7,end);

%Create the transformation of the last pose to draw it
desired_trans = rosmessage('geometry_msgs/TransformStamped');
desired_trans.ChildFrameId = 'Desired_pos';
desired_trans.Header.FrameId = 'odom';
desired_trans.Transform.Translation.X = xi_mm(1,end);
desired_trans.Transform.Translation.Y = xi_mm(2,end);
desired_trans.Transform.Translation.Z = xi_mm(3,end);
desired_trans.Transform.Rotation.W = xi_mm(4,end);
desired_trans.Transform.Rotation.X = xi_mm(5,end);
desired_trans.Transform.Rotation.Y = xi_mm(6,end);
desired_trans.Transform.Rotation.Z = xi_mm(7,end);

%Initial Joints values
state_msg.Name = joints_names;
state_msg.Position = [0.0; 0.0; q_mm(4,1); q_mm(5,1); q_mm(6,1); q_mm(7,1); q_mm(8,1); q_mm(9,1); q_mm(10,1)];
state_msg.Velocity = zeros(9,1);
state_msg.Effort = zeros(9,1);

% Publish the path and tf of the starting position
path_msg.Header.Stamp = now;    
send(path_pub,path_msg);
desired_trans.Header.Stamp = now;
path_msg.Header.Stamp = now;    
send(path_pub,path_msg);
sendTransform(tftree, odom_trans);
sendTransform(tftree, desired_trans);

reset(rate);
for i=1:n    
    time = rate.TotalElapsedTime;
    fprintf('Time Elapsed: %f\n',time)
    
    %Joints values
    state_msg.Name = joints_names;
    state_msg.Position = [0.0; 0.0; q_mm(4,i); q_mm(5,i); q_mm(6,i); q_mm(7,i); q_mm(8,i); q_mm(9,i); q_mm(10,i)];
    state_msg.Velocity = zeros(9,1);
    state_msg.Effort = zeros(9,1);
    
    %Mobile platform movement using the odom transformation
    now = rostime('now');
    odom_trans.Transform.Translation.X = q_mm(1,i);
    odom_trans.Transform.Translation.Y = q_mm(2,i);
    odom_trans.Transform.Translation.Z = 0.0;
    quatrot = eul2quat([q_mm(3,i) 0 0],'ZYZ');
    odom_trans.Transform.Rotation.W = quatrot(1);
    odom_trans.Transform.Rotation.X = quatrot(2);
    odom_trans.Transform.Rotation.Y = quatrot(3);
    odom_trans.Transform.Rotation.Z = quatrot(4);
    
    %Publish the joint states and transformations
    state_msg.Header.Stamp = now;
    odom_trans.Header.Stamp = now;
    desired_trans.Header.Stamp = now;
    send(state_pub,state_msg);
    sendTransform(tftree, odom_trans);
    sendTransform(tftree, desired_trans);    
    waitfor(rate);    
end
