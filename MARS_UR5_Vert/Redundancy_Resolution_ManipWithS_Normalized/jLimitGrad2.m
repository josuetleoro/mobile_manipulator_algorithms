function Wjlim=jLimitGrad2(qk,limits)
persistent prevGradH;
if isempty(prevGradH)
   prevGradH=zeros(9,1) ;
end
q=qk(2:end);
Wjlim=eye(9,9);

rho=0.1;
qmax=limits(:,2);
qmin=limits(:,1);
range=limits(:,2)-limits(:,1);
ql=limits(:,1)+rho*range();
qh=limits(:,2)-rho*range();
beta=10;
for i=3:9
    if q(i) > qh(i)
        gradH = abs(beta*(q(i)-qh(i))/range(i));
    elseif q(i) < ql(i)
        gradH = abs(beta*(q(i)-ql(i))/range(i));
    else
        gradH = 0;
    end

%     if q(i) > qh(i)
%         gradH = atanh((q(i)-qh(i))/(qmax(i)-qh(i)));
%     elseif q(i) < ql(i)
%         gradH = atanh((ql(i)-q(i))/(ql(i)-qmin(i)));
%     else
%         gradH = 0;
%     end

%     if q(i) > qh(i)
%         gradH = (exp(((q(i)-qh(i))/(qmax(i)-qh(i))))-1)/(qmax(i)-q(i));
%     elseif q(i) < ql(i)
%         gradH = (exp(((ql(i)-q(i))/(ql(i)-qmin(i))))-1)/(q(i)-qmin(i));
%     else
%         gradH = 0;
%     end
    

    gradH = abs(gradH);

    gradHDif = gradH - prevGradH(i);
    prevGradH(i) = gradH;   
    
    if gradHDif > 0
        Wjlim(i,i) = 1 + gradH;
    else
        Wjlim(i,i) = 1;
    end
    
    if q(i) >= qmax(i)
        Wjlim(i,i) = inf;
    end
    if q(i) <= qmin(i)
        Wjlim(i,i) = inf;
    end 
end
% Wjlim(3,3)
% pause()
end