close all
clear all
clc

% %Create the path message
% [path_pub, path_msg] = rospublisher('/ur5_tool0_path','nav_msgs/Path');
% path_msg.Header.FrameId = 'odom';

path_start_pos = [1;-2;0.869];
path_length = 4;
n_waves = 2;
wave_length = path_length/n_waves;
x0 = path_start_pos(1);
y0 = path_start_pos(2);
z0 = path_start_pos(3);
pos(1:3,1) = path_start_pos;
ts = 1/50;
k = 1;
for i=0:ts:path_length
    x = x0 + i;
    y = y0;
    z = z0 + 0.4*sin(2*pi/wave_length*i);
    pos(1:3,k)=[x;y;z];
    k=k+1;
end
plot(pos(1,:), pos(3,:))

% n = length(pos);
% path_msg.Poses = arrayfun(@(~) rosmessage('geometry_msgs/PoseStamped'),zeros(1,n));
% for i=1:n
%     path_msg.Poses(i).Pose.Position.X = pos(1,i);
%     path_msg.Poses(i).Pose.Position.Y = pos(2,i);
%     path_msg.Poses(i).Pose.Position.Z = pos(3,i);
%     path_msg.Poses(i).Pose.Orientation.W = 1;
%     path_msg.Poses(i).Pose.Orientation.X = 0;
%     path_msg.Poses(i).Pose.Orientation.Y = 0;
%     path_msg.Poses(i).Pose.Orientation.Z = 0;
% end
% 
% %Use stoploop function to wait for ok button
% FS = stoploop({'Press Ok to stop publishing path'});
% rate = rosrate(10);
% reset(rate);
% while (~FS.Stop())   
%     %Publish the joint states and transformations
%     now = rostime('now');
% %     state_msg.Header.Stamp = now;
% %     odom_trans.Header.Stamp = now;
% %     desired_trans.Header.Stamp = now;
%     path_msg.Header.Stamp = now;
%     send(path_pub,path_msg);
% %     send(state_pub,state_msg);    
% %     sendTransform(tftree, odom_trans);
% %     sendTransform(tftree, desired_trans);    
%     waitfor(rate);
% end
