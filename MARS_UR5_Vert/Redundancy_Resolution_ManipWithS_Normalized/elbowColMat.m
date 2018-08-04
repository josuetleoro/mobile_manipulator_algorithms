function [Wcol, elbowz] = elbowColMat(q,rho,alpha,beta)
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
pos_elbow = [-0.425e0 * (-cos(phi) * cos(q1) + sin(phi) * sin(q1)) * cos(q2) - 0.49e-1 * cos(phi) -0.425e0 * (-cos(phi) * sin(q1) - sin(phi) * cos(q1)) * cos(q2) - 0.49e-1 * sin(phi) 0.645359e0 - 0.425e0 * sin(q2) + zmp]';
Jelbow6 = [cos(phi) -0.2125000000e0 * sin(phi + q1 + q2) - 0.2125000000e0 * sin(phi + q1 - q2) + 0.49e-1 * sin(phi) 0 -0.2125000000e0 * sin(phi + q1 + q2) - 0.2125000000e0 * sin(phi + q1 - q2) -0.2125000000e0 * sin(phi + q1 + q2) + 0.2125000000e0 * sin(phi + q1 - q2) 0 0 0 0; sin(phi) 0.2125000000e0 * cos(phi + q1 - q2) + 0.2125000000e0 * cos(phi + q1 + q2) - 0.49e-1 * cos(phi) 0 0.2125000000e0 * cos(phi + q1 - q2) + 0.2125000000e0 * cos(phi + q1 + q2) -0.2125000000e0 * cos(phi + q1 - q2) + 0.2125000000e0 * cos(phi + q1 + q2) 0 0 0 0; 0 0 0.1e1 0 -0.4250000000e0 * cos(q2) 0 0 0 0; 0 0 0 0 -sin(phi + q1) -sin(phi + q1) 0 0 0; 0 0 0 0 cos(phi + q1) cos(phi + q1) 0 0 0; 0 0.1e1 0 0.1e1 0 0 0 0 0;];
Jelbow = Jelbow6(1:3,:);

%Vector position of a point on the top of the mobile platform
mob_plat_height = 0.5;
pb=[pos_elbow(1);pos_elbow(2);mob_plat_height];

%% Calculate the collision gradient
pa_pb=(pos_elbow-pb);
dist=norm(pa_pb);
deltad_deltaq=1/dist*(Jelbow'*pa_pb);
deltaP_deltad=-rho*exp(-alpha*dist)*dist^(-beta)*(beta/dist+alpha);
gradP = abs(deltaP_deltad*deltad_deltaq);
%dist
%P=exp(-alpha*dist)*dist^(-beta)
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
elbowz=pos_elbow(3);
prevGradP = gradP;
end