function MotPlan=TrajPlanPol(pos0,q0,posf,qf,ts,tf)

%% Motion planning for positon (x,y,z)
[x,dx,~]=Traj5(pos0(1),posf(1),ts,0,tf);
[y,dy,~]=Traj5(pos0(2),posf(2),ts,0,tf);
[z,dz,~]=Traj5(pos0(3),posf(3),ts,0,tf);

%% Motion planning for orientation using quaternion polynomial
%Create the quaternion objects
Q0=Quat(q0');
Qf=Quat(qf');

%Desired initial and final angular velocities and accelerations
w0=[0;0;0];
dw0=[0;0;0];
wf=[0;0;0];
dwf=[0;0;0];
%Do the interpolation
%[quat,w,~]=quatInterpolation(Q0,w0,dw0,Qf,wf,dwf,0,tf,ts);
[quat,w,~]=quatPolynomInterpolation(Q0,w0,dw0,Qf,wf,dwf,0,tf,ts,'fixed');

time=0:ts:tf;

% %Show the end effector motion in 3D
% figure()
% Rd=zeros(4,4,length(time));
% %Form the T6Traj matrix
% for i=1:length(x)
%    Rd(:,4,i)=[x(i);y(i);z(i);1];
%    Rd(1:3,1:3,i)=quatToRotMat(quat(:,i));
%    %Rd(1:3,1:3,i)=quat2rotm(quat(:,i)');
% end
% plotEndEffectorMotion(Rd,5)
% pause()

% %Plot the evolution of dx, dy and dz
% figure()
% subplot(1,3,1)
% plot(time,dx,'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('dx')
% title('dx')
% 
% subplot(1,3,2)
% plot(time,dy,'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('dy')
% title('dy')
% 
% subplot(1,3,3)
% plot(time,dz,'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('dz')
% title('dz')
% 
% %Plot the evolution of wx, wy and wz
% figure()
% subplot(1,3,1)
% plot(time,w(1,:),'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('wx')
% title('wx')
% 
% subplot(1,3,2)
% plot(time,w(2,:),'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('wy')
% title('wy')
% 
% subplot(1,3,3)
% plot(time,w(3,:),'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('wz')
% title('wz')
% 
% 
% %Plot the evolution of quat
% time=0:ts:tf;
% figure()
% subplot(2,2,1)
% plot(time,quat(1,:),'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('quat_w')
% title('quat_w')
% 
% subplot(2,2,2)
% plot(time,quat(2,:),'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('quat_x')
% title('quat_x')
% 
% subplot(2,2,3)
% plot(time,quat(3,:),'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('quat_y')
% title('quat_y')
% 
% subplot(2,2,4)
% plot(time,quat(4,:),'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('quat_z')
% title('quat_z')
% pause()

%Return the motion planning data
MotPlan={};
MotPlan.x=x;
MotPlan.dx=dx;
MotPlan.y=y;
MotPlan.dy=dy;
MotPlan.z=z;
MotPlan.dz=dz;
MotPlan.quat=quat;
MotPlan.w=w;
MotPlan.time=time;
end

