function [Wcol, elbowz, dist] = elbowColMatWithP1(q,rho,alpha,beta)
x=q(1);
y=q(2);
phi=q(3);
zmp=q(4);
q1=q(5);
q2=q(6);
persistent prevGradP;
if isempty(prevGradP)
   prevGradP=zeros(9,1) ;
end

%% Returns the Weight matrix for elbow collision
%Calculate elbow position and Jacobian
pos_elbow = [-0.425e0 * (-cos(phi) * cos(q1) + sin(phi) * sin(q1)) * cos(q2) - 0.49e-1 * cos(phi) + 0.1e1 * x -0.425e0 * (-cos(phi) * sin(q1) - sin(phi) * cos(q1)) * cos(q2) - 0.49e-1 * sin(phi) + 0.1e1 * y 0.645359e0 - 0.425e0 * sin(q2) + zmp];
Jelbow = [cos(phi) -0.2125000000e0 * sin(phi + q1 + q2) - 0.2125000000e0 * sin(phi + q1 - q2) + 0.49e-1 * sin(phi) 0 -0.2125000000e0 * sin(phi + q1 + q2) - 0.2125000000e0 * sin(phi + q1 - q2) -0.2125000000e0 * sin(phi + q1 + q2) + 0.2125000000e0 * sin(phi + q1 - q2) 0 0 0 0; sin(phi) 0.2125000000e0 * cos(phi + q1 - q2) + 0.2125000000e0 * cos(phi + q1 + q2) - 0.49e-1 * cos(phi) 0 0.2125000000e0 * cos(phi + q1 - q2) + 0.2125000000e0 * cos(phi + q1 + q2) -0.2125000000e0 * cos(phi + q1 - q2) + 0.2125000000e0 * cos(phi + q1 + q2) 0 0 0 0; 0 0 0.1e1 0 -0.4250000000e0 * cos(q2) 0 0 0 0;];
%Calculate safety point and Jacobian
pos_p1 = [0.23e0 * cos(phi) + x 0.23e0 * sin(phi) + y 0.5];
Jp1 = [cos(phi) 0 0 0 0 0 0 0 0; sin(phi) 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0];
pb=pos_p1;
%% Calculate the collision gradient
pa_pb=(pos_elbow-pb)';
pb_pa=(pb-pos_elbow)';
dist=norm(pa_pb);

% size(Jelbow)
% size(pa_pb)
% size(Jp1)
% size(pb_pa)
% pause()

deltad_deltaq=1/dist*(Jelbow'*pa_pb+Jp1'*pb_pa);
deltaP_deltad=-rho*exp(-alpha*dist)*dist^(-beta)*(beta/dist+alpha);
gradP = abs(deltaP_deltad*deltad_deltaq);
%dist
% P=exp(-alpha*dist)*dist^(-beta)
% pause()
%gradP
Wcol=eye(9,9);
gradPDif = gradP - prevGradP;
for i=3:5
    if gradPDif(i) >= 0
        Wcol(i,i) = 1 + gradP(i);
    else
        Wcol(i,i) = 1;
    end
end
%Wcol(3,3)=1;
elbowz=pos_elbow(3);
prevGradP = gradP;
end