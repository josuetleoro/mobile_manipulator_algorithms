function MotPlan=TrajPlanQuat(T0,Tf,ts,tb,tf)

pos0=T0(1:3,4);
posf=Tf(1:3,4);

%% Motion planning for positon (x,y,z)
[x,dx,~]=ParabBlend(pos0(1),posf(1),ts,tb,tf);
[y,dy,~]=ParabBlend(pos0(2),posf(2),ts,tb,tf);
[z,dz,~]=ParabBlend(pos0(3),posf(3),ts,tb,tf);

%% Motion planning for orientation
%Initial Orientation
%Transform the rotation matrix to a quaternion
%q0=cartToQuat(T0);
q0=rotm2quat(T0(1:3,1:3))';
q0=q0/norm(q0);
%T0Rec=quatToRotMat(q0)

%Final Orientation
%Transform the rotation matrix to a quaternion
%qf=cartToQuat(Tf);
qf=rotm2quat(Tf(1:3,1:3))';
qf=qf/norm(qf);
%TfRec=quatToRotMat(qf)

trajLength=tf/ts+1;
quat=zeros(4,trajLength);
dquat=zeros(4,trajLength);
w=zeros(3,trajLength);
quat_step=ts/tf;
n=1;

%Create the quaternion objects
Q0=Quat(q0');
Qf=Quat(qf');
%Change sign one of the quaternions to find the shortest path if needed
if dot(Q0.vecRep(),Qf.vecRep())<0
   Qf=-1*Qf; 
end

logq=Quat.log(Q0.conj()*Qf);
for i=0:quat_step:1
    quat(:,n)=slerp(q0,qf,i,0.001);    
    quat(:,n)=quat(:,n)/norm(quat(:,n));    
    q=Quat(quat(:,n)');
    dq=q*logq;
    w4=2*dq*q.inv()/tf;
    w(:,n)=w4.getV();    
    
%     tempQuat=Quat(quat(:,n)');
%     tempQuat=tempQuat*logq;
%     dquat(:,n)=tempQuat.vecRep();
%     w(:,n)=quatRatesToAxesRates(quat(:,n),dquat(:,n))/tf;        
    n=n+1;    
    %w(:,n)=2*logq.v/norm(logq.v)*acos(logq.s);
    %w(:,n)=2*logq.v;
end
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
% ylabel('x')
% title('x')
% 
% subplot(1,3,2)
% plot(time,dy,'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('y')
% title('y')
% 
% subplot(1,3,3)
% plot(time,dz,'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('z')
% title('z')
% 
%Plot the evolution of wx, wy and wz
figure()
subplot(1,3,1)
plot(time,w(1,:),'r','LineWidth',2); grid on
xlabel('time(s)')
ylabel('wx')
title('wx')

subplot(1,3,2)
plot(time,w(2,:),'r','LineWidth',2); grid on
xlabel('time(s)')
ylabel('wy')
title('wy')

subplot(1,3,3)
plot(time,w(3,:),'r','LineWidth',2); grid on
xlabel('time(s)')
ylabel('wz')
title('wz')

%Plot the evolution of quat
time=0:ts:tf;
figure()
subplot(2,2,1)
plot(time,quat(1,:),'r','LineWidth',2); grid on
xlabel('time(s)')
ylabel('quat_w')
title('quat_w')

subplot(2,2,2)
plot(time,quat(2,:),'r','LineWidth',2); grid on
xlabel('time(s)')
ylabel('quat_x')
title('quat_x')

subplot(2,2,3)
plot(time,quat(3,:),'r','LineWidth',2); grid on
xlabel('time(s)')
ylabel('quat_y')
title('quat_y')

subplot(2,2,4)
plot(time,quat(4,:),'r','LineWidth',2); grid on
xlabel('time(s)')
ylabel('quat_z')
title('quat_z')
pause()

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

