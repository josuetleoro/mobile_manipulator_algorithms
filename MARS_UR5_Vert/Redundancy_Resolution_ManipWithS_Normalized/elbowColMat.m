function [Wcol, dist_signed] = elbowColMat(q,rho,alpha,beta)
%% Returns the Weight matrix for elbow collision
% Minimum height of the elbow
elbow_safe_dist = 0.5;
x=q(1);
y=q(2);
phi=q(3);
zmp=q(4);
q1=q(5);
q2=q(6);
persistent prevGradPElbow;
if isempty(prevGradPElbow)
   prevGradPElbow=zeros(9,1) ;
end

%% Calculate the elbow position and Jacobian
% The Jacobian is with respect to a point on the center of the wheels
% projected to the floor. This frame allows to define the vector pa_pb just
% as the z position of the elbow minus the safe distance
pos_elbow_z = 0.645359e0 - 0.425e0 * sin(q2) + zmp;
Jelbow = [0 0 1 0 -0.4250000000e0 * cos(q2) 0 0 0 0];

%% Calculate the collision gradient
pa_pb=pos_elbow_z - elbow_safe_dist;
dist_signed = pa_pb;
dist=norm(pa_pb);
deltad_deltaq=1/dist*(Jelbow'*pa_pb);
deltaP_deltad=-rho*exp(-alpha*dist)*dist^(-beta)*(beta/dist+alpha);
gradP = abs(deltaP_deltad*deltad_deltaq);
%dist
% P=exp(-alpha*dist)*dist^(-beta)
% pause()
%gradP
Wcol=eye(9,9);
gradPDif = gradP - prevGradPElbow;
for i=3:5
    if gradPDif(i) >= 0
        Wcol(i,i) = 1 + gradP(i);
    else
        Wcol(i,i) = 1;
    end
end
%Wcol(3,3)=1;
%elbowz=pos_elbow(3);
prevGradPElbow = gradP;
end