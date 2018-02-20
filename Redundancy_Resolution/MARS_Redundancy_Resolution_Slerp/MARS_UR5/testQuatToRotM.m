%pos1 = [0.484;0.109;0.091];
%quat1 = [0,0.707,0.707,0];
% pos2 = [-0.392,0.109,1.132];
% quat2 = [0.5,0.5,0.5,0.5];
% pos3 = [0.139,0.046,1.246];
% quat3 = [-0.406,0.669,-0.187,0.594];
% pos4 = [0.065,0.131,1.246];
% quat4 = [-0.631,0.462,0.055,0.621];
pos5 = [-0.042,0.447,1.358];
quat5 = [0.514,0.537,0.584,0.327];

%ROS x,y,z,w
%Matlab w,x,y,z
pos=pos5;
quat = quat5;
temp = quat(4);
quat(2:4)=quat(1:3);
quat(1)=temp;
rotm=quat2rotm(quat);

T=zeros(4,4);
T(4,4)=1;
T(1:3,1:3)=rotm;
T(1:3,4)=pos
% roll=-pi/2;
% pitch=pi/2;
% yaw=0.0;
% eul=[yaw,pitch,roll];
% rotm=eul2rotm(eul,'ZYX')