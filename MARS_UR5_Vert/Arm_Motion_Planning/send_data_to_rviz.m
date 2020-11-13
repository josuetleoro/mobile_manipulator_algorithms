close all
clear all
clc

%Open a dialog box to look for the motion data
uiopen('*.mat');

%Determine the number of iterations
n=size(q,2);
 
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

%% Create the mob_plat path message
[mob_plat_path_pub, mob_plat_path_msg] = rospublisher('/mob_plat_path','nav_msgs/Path');
mob_plat_path_msg.Header.FrameId = 'odom';

% Find the dividend to use so that at least 100 points are always used for the path
pathDiv = 2;
pathNPoints = 100;
if n > pathNPoints
    pathDiv = floor(n/pathNPoints);
end

mob_plat_path_msg.Poses = arrayfun(@(~) rosmessage('geometry_msgs/PoseStamped'),zeros(1,ceil(n/pathDiv)));        

k=1;
for i=1:n
    if(mod(i,pathDiv)==0)
        mob_plat_path_msg.Poses(k).Pose.Position.X = q(1,i);
        mob_plat_path_msg.Poses(k).Pose.Position.Y = q(2,i);
        mob_plat_path_msg.Poses(k).Pose.Position.Z = 0;
        quatrot = eul2quat([q(3,i) 0 0],'ZYZ');
        mob_plat_path_msg.Poses(k).Pose.Orientation.W = quatrot(1);
        mob_plat_path_msg.Poses(k).Pose.Orientation.X = quatrot(2);
        mob_plat_path_msg.Poses(k).Pose.Orientation.Y = quatrot(3);
        mob_plat_path_msg.Poses(k).Pose.Orientation.Z = quatrot(4);
        k=k+1;
    end
end
%Make sure the last pose is also stored
mob_plat_path_msg.Poses(end).Pose.Position.X = q(1,end);
mob_plat_path_msg.Poses(end).Pose.Position.Y = q(2,end);
mob_plat_path_msg.Poses(end).Pose.Position.Z = 0;
quatrot = eul2quat([q(3,i) 0 0],'ZYZ');
mob_plat_path_msg.Poses(end).Pose.Orientation.W = quatrot(1);
mob_plat_path_msg.Poses(end).Pose.Orientation.X = quatrot(2);
mob_plat_path_msg.Poses(end).Pose.Orientation.Y = quatrot(3);
mob_plat_path_msg.Poses(end).Pose.Orientation.Z = quatrot(4);

%% Create the transformation of the last mob_plat pose to draw it
desired_trans = rosmessage('geometry_msgs/TransformStamped');
desired_trans.ChildFrameId = 'Desired_pos';
desired_trans.Header.FrameId = 'odom';
desired_trans.Transform.Translation.X = q(1,end);
desired_trans.Transform.Translation.Y = q(2,end);
desired_trans.Transform.Translation.Z = 0;
desired_trans.Transform.Rotation.W = quatrot(1);
desired_trans.Transform.Rotation.X = quatrot(2);
desired_trans.Transform.Rotation.Y = quatrot(3);
desired_trans.Transform.Rotation.Z = quatrot(4);

[state_pub, state_msg] = rospublisher('/mars_joint_states','sensor_msgs/JointState');

%Use stoploop function to wait for ok button
FS = stoploop({'Press Ok to start the simulation'}); 

%Initial Joints values
state_msg.Name = joints_names;
state_msg.Position = [0.0; 0.0; q(4,1); q(5,1); q(6,1); q(7,1); q(8,1); q(9,1); q(10,1)];
state_msg.Velocity = zeros(9,1);
state_msg.Effort = zeros(9,1);
rate = rosrate(20);
reset(rate);
%Initial odom transformation
odom_trans.Transform.Translation.X = q(1,1);
odom_trans.Transform.Translation.Y = q(2,1);
odom_trans.Transform.Translation.Z = 0.0;
quatrot = eul2quat([q(3,1) 0 0],'ZYZ');
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
    mob_plat_path_msg.Header.Stamp = now;   
    send(state_pub,state_msg);
    send(mob_plat_path_pub,mob_plat_path_msg);
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
    state_msg.Position = [0.0; 0.0; q(4,i); q(5,i); q(6,i); q(7,i); q(8,i); q(9,i); q(10,i)];
    state_msg.Velocity = zeros(9,1);
    state_msg.Effort = zeros(9,1);
    
    %Mobile platform movement using the odom transformation
    now = rostime('now');
    odom_trans.Transform.Translation.X = q(1,i);
    odom_trans.Transform.Translation.Y = q(2,i);
    odom_trans.Transform.Translation.Z = 0.0;
    quatrot = eul2quat([q(3,i) 0 0],'ZYZ');
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