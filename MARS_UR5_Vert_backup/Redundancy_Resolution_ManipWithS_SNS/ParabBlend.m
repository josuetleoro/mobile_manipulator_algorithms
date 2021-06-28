function [q,dq,ddq]=ParabBlend(q0,qf,ts,tb,tf) 
%Initial Position, Final Position, Acceleration, Final Time, Blending
%Time, sample time

%Calculate velocity and acceleration
%disp('Task space linear velocity');
vel=(qf-q0)/(tf-tb);
max_a=vel/tb;

i=1;    %Element position
%Initial parabolic segment
for t=0:ts:tb
    q(i)=q0+max_a/2*t^2;
    dq(i)=max_a*t;
    ddq(i)=max_a;
    time(i)=t;
    i=i+1;
end

%Knot point
pa_x=q(i-1);

%Linear segment
for t=time(i-1)+ts:ts:tf-tb
    q(i)=pa_x+vel*(t-tb);
    dq(i)=vel;
    ddq(i)=0;
    time(i)=t;
    i=i+1;
end

%Final parabolic segment
for t=time(i-1)+ts:ts:tf
    %Initial parabolic segment
    q(i)=qf-max_a/2*(tf-t)^2;
    dq(i)=max_a*(tf-t);
    ddq(i)=-1*max_a;
    time(i)=t;
    i=i+1;
end

% %Plot of the X Pos and Vel
% figure(1)
% subplot(1,3,1)
% plot(time,q,'b','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('q')
% title('Position q')
% 
% subplot(1,3,2)
% plot(time,dq,'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('dq')
% title('Velocity dq')
% 
% subplot(1,3,3)
% plot(time,ddq,'r','LineWidth',2); grid on
% xlabel('time(s)')
% ylabel('ddq')
% title('Acceleration ddq')
% 
% pause()

end