function [dQmin, dQmax, idxMin, idxMax]=shapeJointVelBoundArm(qk,q_limit,dq_limit,ddq_limit,ts)
n=length(q_limit);
dQmax=zeros(n,1);
dQmin=dQmax;

% %Mobile platform
% for i=1:2
%     dQmin(i)=-dq_limit(i);
%     dQmax(i)=dq_limit(i);    
% end

% for i=1:2
%     dqc(1)=mp_vel(i)+ddq_limit(i);
%     dqc(2)=mp_vel(i)-ddq_limit(i);
%     dqc=abs(dqc);
%     dQmin(i)=max(-dq_limit(i),-1*min(dqc));
%     dQmax(i)=min(dq_limit(i),min(dqc));    
% end

%Robot arm and prismatic joint
for i=1:n
    %minium bound
    [dQmin(i) idxMin(i)]=max([(q_limit(i,1)-qk(i))/ts,-dq_limit(i),-sqrt(2*ddq_limit(i)*(qk(i)-q_limit(i,1)))]);
    
    %maximum bound
    [dQmax(i) idxMax(i)]=min([(q_limit(i,2)-qk(i))/ts,dq_limit(i),sqrt(2*ddq_limit(i)*(q_limit(i,2)-qk(i)))]);
end

% (q_limit(1,1)-qk(1))/ts
% (q_limit(1,2)-qk(1))/ts

end