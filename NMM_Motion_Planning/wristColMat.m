function [Wcol, dist_signed, wrist_pos] = wristColMat(q,rho,alpha,beta)
persistent prevGradPWrist;

if isempty(prevGradPWrist)
   prevGradPWrist=zeros(9,1) ;
end

%% Returns the Weight matrix for wrist collision
% Minimum distance in x 
wrist_safe_dist = 0.35;
x=q(1);
y=q(2);
phi=q(3);
zmp=q(4);
q1=q(5);
q2=q(6);
q3=q(7);

%% Calculate the wrist position and Jacobian
% The Jacobian is with respect to a point on the center of the wheels
% projected to the floor. This frame allows to define the vector pa_pb just
% as the x position of the wrist minus the safe distance
wrist_pos = [0.39225e0 * cos(q1) * cos(q2) * cos(q3) - 0.39225e0 * cos(q1) * sin(q2) * sin(q3) - 0.49e-1 + 0.425e0 * cos(q1) * cos(q2) 0.39225e0 * sin(q1) * cos(q2) * cos(q3) - 0.39225e0 * sin(q1) * sin(q2) * sin(q3) + 0.425e0 * sin(q1) * cos(q2) -0.39225e0 * sin(q2) * cos(q3) - 0.39225e0 * cos(q2) * sin(q3) + 0.645359e0 - 0.425e0 * sin(q2) + zmp]';
Jwrist = [0 0 0 -0.1961250000e0 * sin(q3 + q1 + q2) - 0.1961250000e0 * sin(-q3 + q1 - q2) - 0.2125000000e0 * sin(q1 + q2) - 0.2125000000e0 * sin(q1 - q2) -0.1961250000e0 * sin(q3 + q1 + q2) + 0.1961250000e0 * sin(-q3 + q1 - q2) - 0.2125000000e0 * sin(q1 + q2) + 0.2125000000e0 * sin(q1 - q2) -0.1961250000e0 * sin(q3 + q1 + q2) + 0.1961250000e0 * sin(-q3 + q1 - q2) 0 0 0];

%% Calculate the collision gradient
Wcol=eye(9,9);
pa_pb=(wrist_pos(1)-wrist_safe_dist);
dist_signed = pa_pb;
dist = norm(pa_pb);
% For the wrist calculate the gradient only if the wrist height is below a 
% safe distance, otherwise use identity matrix
if wrist_pos(3) > 0.5    
    return;
end
deltad_deltaq=1/dist*(Jwrist'*pa_pb);
deltaP_deltad=-rho*exp(-alpha*dist)*dist^(-beta)*(beta/dist+alpha);
gradP = abs(deltaP_deltad*deltad_deltaq);
%dist
%P=exp(-alpha*dist)*dist^(-beta)
gradPDif = gradP - prevGradPWrist;
for i=3:9
    if gradPDif(i) >= 0
        Wcol(i,i) = 1/sqrt(1 + gradP(i));
    else
        Wcol(i,i) = 1;
    end
end
prevGradPWrist = gradP;
end