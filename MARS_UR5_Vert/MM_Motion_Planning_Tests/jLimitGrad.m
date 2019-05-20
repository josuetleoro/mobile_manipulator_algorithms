function Wjlim=jLimitGrad(qk,limits)
persistent prevGradH;
if isempty(prevGradH)
   prevGradH=zeros(9,1) ;
end
q=qk(2:end);
Wjlim=eye(9,9);
beta=50; %default 50
for i=3:9
    gradH = abs((limits(i,2)-limits(i,1))^2*(2*q(i)-limits(i,2)-limits(i,1))/(4*(limits(i,2)-q(i))^2*(q(i)-limits(i,1))^2));
    gradH = gradH/beta;
    gradHDif = gradH - prevGradH(i);
    prevGradH(i) = gradH;
    if gradHDif >= 0
        Wjlim(i,i) = 1/sqrt(1 + gradH);
    else
        Wjlim(i,i) = 1;
    end
end
end