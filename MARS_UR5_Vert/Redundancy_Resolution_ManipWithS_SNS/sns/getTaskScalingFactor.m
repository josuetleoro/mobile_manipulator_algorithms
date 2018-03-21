function [scalingFactor,mostCriticalJoint]=getTaskScalingFactor(a,b,W,dQmax,dQmin)
n=length(a);
Smin=(dQmin-b)./a;
Smax=(dQmax-b)./a;

for i=1:n
    %Switch min and max if necessary
    if Smin(i)>Smax(i)
       temp=Smin(i);
       Smin(i)=Smax(i);
       Smax(i)=temp;
    end
    %Remove saturated joints
    if(W(i,i)~=1)
        Smin(i) = -Inf;
        Smax(i) = Inf;
    end
end

%Find most critical joint
[smax,idx]=min(Smax);
mostCriticalJoint = idx;
smin=max(Smin);

%if (smin>smax) || (smax < 0.0) || (smin > 1.0) || (smax == Inf)
if (smin>smax) || (smax < 0.0) || (smin > 1.0) || (smax == Inf)
%     Smin(1:8)
%     Smax(1:8)
%     smin
%     smax
    %scalingFactor = -1.0;
    scalingFactor = 0.0;
else
    scalingFactor = smax;
end
end