joints_names = {'right_wheel','left_wheel','prismatic_joint','ur5_shoulder_pan_joint','ur5_shoulder_lift_joint'...
    'ur5_elbow_joint','ur5_wrist_1_joint','ur5_wrist_2_joint','ur5_wrist_3_joint'};

rosinit

%Get the transformation tree
tftree = rostf;
pause(1);
%Create the transformation message
odom_trans = rosmessage('geometry_msgs/TransformStamped');
odom_trans.ChildFrameId = 'base_footprint';
odom_trans.Header.FrameId = 'odom';

%[state_pub, state_msg] = rospublisher('/joint_states','sensor_msgs/JointState');
[state_pub, state_msg] = rospublisher('/mars_joint_states','sensor_msgs/JointState');
count = 0;
prism_pos = 0.0;
prism_inc = 0.002;
x_pos = 0.0;
angle = 0.0;
rate = rosrate(20);
reset(rate);
while (rate.TotalElapsedTime <= 10)
    time = rate.TotalElapsedTime;
    fprintf('Time Elapsed: %f\n',time)
    
    %Joints values
    state_msg.Name = joints_names;
    state_msg.Position = [0.0; 0.0; prism_pos; 0.0; -pi; pi/2; 0.0; 0.0; pi/2];
    state_msg.Velocity = zeros(9,1);
    state_msg.Effort = zeros(9,1);
    now = rostime('now');
    
    %Mobile platform movement
    odom_trans.Transform.Translation.X = 1*cos(angle);
    odom_trans.Transform.Translation.Y = 1*sin(angle);
    odom_trans.Transform.Translation.Z = 0.0;
    
    %quatrot = axang2quat([0 0 1 deg2rad(x_pos)]);
    quatrot = eul2quat([angle+pi/2 0 0],'ZYZ');
    odom_trans.Transform.Rotation.W = quatrot(1);
    odom_trans.Transform.Rotation.X = quatrot(2);
    odom_trans.Transform.Rotation.Y = quatrot(3);
    odom_trans.Transform.Rotation.Z = quatrot(4);

    state_msg.Header.Stamp = now;    
    odom_trans.Header.Stamp = now;
    send(state_pub,state_msg);
    sendTransform(tftree, odom_trans);
    
    %Update joints values
    x_pos = x_pos + 0.01;    
    angle = angle + 0.01;    
    prism_pos = prism_pos + prism_inc;
    if (prism_pos < 0.0 || prism_pos > 0.4)
        prism_inc = prism_inc*-1;
    end
    
    waitfor(rate);
end

rosshutdown