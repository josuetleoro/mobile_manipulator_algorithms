function Wjlim=jLimitGrad(qk,limits)
persistent prevGradH;
if isempty(prevGradH)
   prevGradH=zeros(9,1) ;
end
q=qk(2:end);
Wjlim=eye(9,9);
beta=50;
for i=3:9
    gradH = abs((limits(i,2)-limits(i,1))^2*(2*q(i)-limits(i,2)-limits(i,1))/(4*(limits(i,2)-q(i))^2*(q(i)-limits(i,1))^2));
    gradH = gradH/beta;
    gradHDif = gradH - prevGradH(i);
    prevGradH(i) = gradH;
    if gradHDif >= 0
%         range=abs(limits(i,2)-limits(i,1));
%         if i == 3
%             perc=range*10/100;
%             low = limits(i,1) + perc;
%             high = limits(i,2) - perc;
%         else
%             perc=range*30/100;
%             low = limits(i,1) + perc;
%             high = limits(i,2) - perc;
%         end
%         
%         if q(i) > low && q(i) < high
%             Wjlim(i,i) = 1;
%         else
%             Wjlim(i,i) = 1 + gradH;
%         end
        Wjlim(i,i) = 1 + gradH;
    else
        Wjlim(i,i) = 1;
    end
end
% Wjlim(3,3)
% pause()
end