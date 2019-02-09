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


% %Test1
% mob_plat_pos = [0.0;0.0;0.0]; %[x,y,phi]
% prism_pos = 0.2;
% ur5_pos = [0.0;0.0;0.0;0.0;0.0;0.0];


% %Test2
mob_plat_pos = [0.0;0.0;-0.26]; %[x,y,phi]
prism_pos = 0.1234;
ur5_pos = [-0.8597;-0.6589;0.6589;-0.1246;-0.5897;pi/2];

%Joints values
state_msg.Name = joints_names;
state_msg.Position = [0; 0; prism_pos; ur5_pos];
state_msg.Velocity = zeros(9,1);
state_msg.Effort = zeros(9,1);

rate = rosrate(2);
reset(rate);
while (rate.TotalElapsedTime <= 10)
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
    
    base_foot_print_To_ur5_tool0 = getTransform(tftree, 'odom', 'ur5_tool0');
    translation = base_foot_print_To_ur5_tool0.Transform.Translation;
    rotation = base_foot_print_To_ur5_tool0.Transform.Rotation;
    
    pos = [translation.X, translation.Y, translation.Z];
    quat = [rotation.W,rotation.X,rotation.Y,rotation.Z];
    rotm=quat2rotm(quat);
    
    T=zeros(4,4);
    T(4,4)=1;
    T(1:3,1:3)=rotm;
    T(1:3,4)=pos
    
    
end

%rosshutdown