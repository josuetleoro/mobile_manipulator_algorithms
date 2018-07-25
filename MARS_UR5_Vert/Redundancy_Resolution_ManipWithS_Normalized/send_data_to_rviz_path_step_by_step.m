clear all
clc

%Open a dialog box to look for the motion data
uiopen();

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

%Create the path message
[path_pub, path_msg] = rospublisher('/ur5_tool0_path','nav_msgs/Path');
path_msg.Header.FrameId = 'odom';

[state_pub, state_msg] = rospublisher('/mars_joint_states','sensor_msgs/JointState');

%Use stoploop function to wait for ok button
FS = stoploop({'Press Ok to start the simulation'}); 

%Initial Joints values
state_msg.Name = joints_names;
state_msg.Position = [0.0; 0.0; q(4,1); q(5,1); q(6,1); q(7,1); q(8,1); q(9,1); q(10,1)];
state_msg.Velocity = zeros(9,1);
state_msg.Effort = zeros(9,1);
rate = rosrate(10);
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
    send(state_pub,state_msg);
    sendTransform(tftree, odom_trans);
    waitfor(rate);    
end
FS.Clear()
clear FS;
clear rate

k=1;
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
    send(state_pub,state_msg);
    sendTransform(tftree, odom_trans);
    
    %Add point to the path and publish it
    if(mod(i,10)==0)
        path_msg_temp = rosmessage(path_pub);
        if k > 1
            path_msg_temp = arrayfun(@(~) rosmessage('geometry_msgs/PoseStamped'),zeros(1,k-1));
            path_msg_temp(1:k-1)=path_msg.Poses(1:k-1);
            path_msg.Poses = arrayfun(@(~) rosmessage('geometry_msgs/PoseStamped'),zeros(1,k));
            path_msg.Poses(1:k-1)=path_msg_temp;
        else
            path_msg.Poses = arrayfun(@(~) rosmessage('geometry_msgs/PoseStamped'),zeros(1,k));
        end
        path_msg.Poses(k).Pose.Position.X = xi_des(1,i);    
        path_msg.Poses(k).Pose.Position.Y = xi_des(2,i);
        path_msg.Poses(k).Pose.Position.Z = xi_des(3,i);
        path_msg.Poses(k).Pose.Orientation.W = xi_des(4,i);
        path_msg.Poses(k).Pose.Orientation.X = xi_des(5,i);
        path_msg.Poses(k).Pose.Orientation.Y = xi_des(6,i);
        path_msg.Poses(k).Pose.Orientation.Z = xi_des(7,i);    
        path_msg.Header.Stamp = now;    
        send(path_pub,path_msg);
        k=k+1;
    end    
    waitfor(rate);    
end

%rosshutdown