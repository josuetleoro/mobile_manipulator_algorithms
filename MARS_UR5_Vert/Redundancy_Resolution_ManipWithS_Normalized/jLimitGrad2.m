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
beta=1000;
for i=3:9
    x = pi/range(i)*(q(i)-qmin(i))-pi/2;
    gradH = abs(2*pi/(beta*range(i))*tan(x)*sec(x)^2);
    gradHDif = gradH - prevGradH(i);
    prevGradH(i) = gradH;        
    if gradHDif > 0
        Wjlim(i,i) = 1 + gradH;
    else
        Wjlim(i,i) = 1;
    end
end
% Wjlim
% pause()
end