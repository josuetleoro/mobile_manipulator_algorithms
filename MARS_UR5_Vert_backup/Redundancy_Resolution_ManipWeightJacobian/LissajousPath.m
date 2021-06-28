function MotPlan=LissajousPath(T0,ts,period,l)
T=period;
pos0=T0(1:3,4);
q0=cartToQuat(T0(1:3,1:3));

k=1;
x(1)=pos0(1);
y(1)=pos0(2);
z(1)=pos0(3);
for t=0:ts:period
    dx(k)=-2*pi^2*l/T*sin(4*pi*sin(pi*t/(2*T))^2+pi/6)*sin(pi*t/T);
    dy(k)=pi^2*l/T*cos(2*pi*sin(pi*t/(2*T))^2+pi/6)*sin(pi*t/T);
    dz(k)=0;
    z(k)=pos0(3);
    
    quat(1:4,k)=q0;
    w(1:3,k)=[0;0;0];    
    k=k+1;
    x(k)=x(k-1)+dx(k-1)*ts;
    y(k)=y(k-1)+dy(k-1)*ts;        
end
x=x(2:end);
y=y(2:end);
%plot(x,y);
time=0:ts:period;

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