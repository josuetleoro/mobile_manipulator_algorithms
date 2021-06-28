function quat_ros=rotmat2quatROS(R)

% In matlab the order is w,x,y,z
% In ROS the order is x,y,z,w
quat_matlab=rotm2quat(R);
quat_ros=[quat_matlab(2:4), quat_matlab(1)];

end