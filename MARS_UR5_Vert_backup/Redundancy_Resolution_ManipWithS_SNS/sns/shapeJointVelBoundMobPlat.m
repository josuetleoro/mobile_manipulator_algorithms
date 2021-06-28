function [dQmin, dQmax]=shapeJointVelBoundMobPlat(dqk,dqk_minus_1,dq_limit,ddq_limit,ts)
dQmax=zeros(2,1);
dQmin=dQmax;

%Mobile platform
for i=1:2
    if (dqk(i) - dqk_minus_1(i)) >= 0
        %minium bound
        [dQmin(i)]=-dq_limit(i);
    
        %maximum bound
        [dQmax(i)]=min([dq_limit(i),dqk_minus_1(i)+ddq_limit(i)*ts]);        
    else
        %minium bound
        [dQmin(i)]=max([-dq_limit(i),dqk_minus_1(i)-ddq_limit(i)*ts]);        
    
        %maximum bound
        [dQmax(i)]=dq_limit(i);        
    end    
end

end