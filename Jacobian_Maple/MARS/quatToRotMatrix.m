% Testing quat to rotation matrix to validate Maple transformation matrices
quat1 = [-0.0, 0.707, 0.707, 0.0];
quat2 = [0.0, 0.707, 0.707, 0.0];
quat3 = [-0.053, -0.419, 0.893, 0.155];
quat4 = [0.108, 0.729, -0.665, -0.123];
quat5 = [0.059, 0.402, -0.092, 0.909];

%ROS order x,y,z,w
%Matlab order w,x,y,z
quat = quat1;
temp = quat(1);
quat(1:3)=quat(2:4);
quat(4)=temp;

rotm = quat2rotm(quat)