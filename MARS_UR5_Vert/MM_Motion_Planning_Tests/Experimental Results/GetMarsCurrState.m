disp('Reading data from MARS state')
rosinit('172.16.10.1')

clear tftree map_to_base_footprint mars_pos quat mars_orient mars_orient_euler

pause(1);
%Get the transformation tree
tftree = rostf;
pause(1);

%Get the mobile platform position and orientation
%map_to_base_footprint = getTransform(tftree, 'map', 'vive_mars');
map_to_base_footprint = getTransform(tftree, 'map', 'base_footprint');
mars_pos = map_to_base_footprint.Transform.Translation;
quat = map_to_base_footprint.Transform.Rotation;
mars_orient = [quat.W quat.X quat.Y quat.Z];
mars_orient_euler = quat2eul(mars_orient); %ZYX order

tx = mars_pos.X
ty = mars_pos.Y
phi_mp = mars_orient_euler(1)

%Get the joint angles
joint_states_sub = rossubscriber('/joint_states');
joint_states = receive(joint_states_sub, 10);
tz = joint_states.Position(3)
qa=[joint_states.Position(4);joint_states.Position(5);joint_states.Position(6);...
    joint_states.Position(7);joint_states.Position(8);joint_states.Position(9)]
%qa_angle = qa*180/pi

clear quat

rosshutdown