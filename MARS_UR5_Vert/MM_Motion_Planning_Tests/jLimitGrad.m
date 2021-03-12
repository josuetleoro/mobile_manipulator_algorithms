function Wjlim=jLimitGrad(qk,limits)
persistent prevGradH;
if isempty(prevGradH)
   prevGradH=zeros(9,1) ;
end
q=qk(2:end);
Wjlim=eye(9,9);
beta=5; %default 50
for i=3:9
    if (i == 3) % Prismatic joint
        beta = 50;
    else
        beta = 5; % All the other joints
    end
    
    gradH = abs((limits(i,2)-limits(i,1))^2*(2*q(i)-limits(i,2)-limits(i,1))/(4*(limits(i,2)-q(i))^2*(q(i)-limits(i,1))^2));
    gradH = gradH/beta;
    gradHDif = gradH - prevGradH(i);
    
%    %Future movement might be out of bounds
%     if (q(i) < limits(i,1) || q(i) > limits(i,2))
%         gradH = Inf;
%         gradHDif = -1;
%     else
%         gradHDif = gradH - prevGradH(i);
%     end

    
    prevGradH(i) = gradH;
    if gradHDif >= 0
        Wjlim(i,i) = 1/sqrt(1 + gradH);
    else
        Wjlim(i,i) = 1;
    end
end
end