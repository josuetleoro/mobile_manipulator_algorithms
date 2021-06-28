function Wman = manipWeight(dP,w,rho,alpha,beta)
persistent prevGradPmanip;
if isempty(prevGradPmanip)
   prevGradPmanip=zeros(9,1) ;
end

%% Calculate P function gradient
dist=w;
deltad_deltaq=dP;
deltaP_deltad=-rho*exp(-alpha*dist)*dist^(-beta)*(beta/dist+alpha);
gradP = abs(deltaP_deltad*deltad_deltaq);
% dist
%P=exp(-alpha*dist)*dist^(-beta);
% gradP
gradPDif = gradP - prevGradPmanip;
%pause()
Wman=eye(9,9);
for i=1:9
    if gradPDif(i) >= 0
        Wman(i,i) = 1 + gradP(i);
    else
        Wman(i,i) = 1;
    end
end
prevGradPmanip = gradP;
end