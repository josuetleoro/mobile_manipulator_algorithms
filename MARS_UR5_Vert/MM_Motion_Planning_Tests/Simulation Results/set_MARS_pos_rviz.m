joints_names = {'right_wheel','left_wheel','prismatic_joint','ur5_shoulder_pan_joint','ur5_shoulder_lift_joint'...
    'ur5_elbow_joint','ur5_wrist_1_joint','ur5_wrist_2_joint','ur5_wrist_3_joint'};

%rosinit

%Get the transformation tree
tftree = rostf;
%Create the transformation message
odom_trans = rosmessage('geometry_msgs/TransformStamped');
odom_trans.ChildFrameId = 'base_footprint';
odom_trans.Header.FrameId = 'odom';

[state_pub, state_msg] = rospublisher('/mars_joint_states','sensor_msgs/JointState');

mob_plat_pos = [0.0;0.0;0.0]; %[x,y,phi]
prism_pos = 0.0;
%ur5_pos = [0.0;-0.061261057;-1.329242758;2.393719068;-2.646791811;-pi/2;pi];
ur5_pos = [0.0;0.0;0.0;0.0;0.0;0.0];

%Joints values
state_msg.Name = joints_names;
state_msg.Position = [0; 0; prism_pos; ur5_pos];
state_msg.Velocity = zeros(9,1);
state_msg.Effort = zeros(9,1);

rate = rosrate(20);
reset(rate);
while (rate.TotalElapsedTime <= 5)
    time = rate.TotalElapsedTime;
    fprintf('Time Elapsed: %f\n',time)
    now = rostime('now');
    
    %Mobile platform movement
    odom_trans.Transform.Translation.X = mob_plat_pos(1);
    odom_trans.Transform.Translation.Y = mob_plat_pos(2);
    odom_trans.Transform.Translation.Z = 0.0;
    
    quatrot = axang2quat([0 0 1 mob_plat_pos(3)]);
    %quatrot = eul2quat([pi/2 0 0],'ZYZ');
    odom_trans.Transform.Rotation.W = quatrot(1);
    odom_trans.Transform.Rotation.X = quatrot(2);
    odom_trans.Transform.Rotation.Y = quatrot(3);
    odom_trans.Transform.Rotation.Z = quatrot(4);
    
    state_msg.Header.Stamp = now;
    odom_trans.Header.Stamp = now;
    send(state_pub,state_msg);
    sendTransform(tftree, odom_trans);
    
    waitfor(rate);
end

%rosshutdown