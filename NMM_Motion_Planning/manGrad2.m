function [dP,w,UR5_dP,UR5_w]=manGradJBar(q,J)
%Calculates the manipulability of the system and the arm alone

%The first element, second and fourth element are zero (x,y,z does not
%exist in the jacobian matrix, therefore the derivative of JJt with respect
%to these is zero). The same happens with the last joint q10.
%It is important to consider that the derivative is performed on JBar,
%therefore dq2 (JBar), corresponds to dq3 (J)
[dJdq3,dJdq5,dJdq6,dJdq7,dJdq8,dJdq9]=evaluatedJdq(q(3),q(5),q(6),q(7),q(8),q(9));

%% Mobile Manipulator

%Calculate JJt
JJt=J*J';
Jt=J';

%Calculate the manipulability measure
% eigval=svd(J);
% w=prod(eigval);

det_JJt=det(JJt);
w=sqrt(det_JJt);

%Calculate each of the elements of the gradient 
inv_JJt=inv(JJt);
w_2=w/2; %We use -w because the internal motion is substracted
dP(1,1)=0;
dP(2,1)=0;
dP(3,1)=w_2*trace(inv_JJt*(dJdq3*Jt+J*dJdq3'));
dP(4,1)=0;
dP(5,1)=w_2*trace(inv_JJt*(dJdq5*Jt+J*dJdq5'));
dP(6,1)=w_2*trace(inv_JJt*(dJdq6*Jt+J*dJdq6'));
dP(7,1)=w_2*trace(inv_JJt*(dJdq7*Jt+J*dJdq7'));
dP(8,1)=w_2*trace(inv_JJt*(dJdq8*Jt+J*dJdq8'));
dP(9,1)=w_2*trace(inv_JJt*(dJdq9*Jt+J*dJdq9'));
dP(10,1)=0;
% w_2
% dP

%% UR5

%Get the manipulability measure
UR5_J=evaluateUR5J(q(5),q(6),q(7),q(8),q(9));
UR5_w=abs(det(UR5_J));
[dJdq1,dJdq2,dJdq3,dJdq4,dJdq5]=evaluateUR5dJdq(q(5),q(6),q(7),q(8),q(9));

%Calculate each of the elements of the gradient 
UR5_J_inv=inv(UR5_J);
UR5_w_2=UR5_w; %We use -w because the internal motion is substracted
UR5_dP(1,1)=0;
UR5_dP(2,1)=0;
UR5_dP(3,1)=0;
UR5_dP(4,1)=0;
UR5_dP(5,1)=UR5_w_2*trace(UR5_J_inv*dJdq1);
UR5_dP(6,1)=UR5_w_2*trace(UR5_J_inv*dJdq2);
UR5_dP(7,1)=UR5_w_2*trace(UR5_J_inv*dJdq3);
UR5_dP(8,1)=UR5_w_2*trace(UR5_J_inv*dJdq4);
UR5_dP(9,1)=UR5_w_2*trace(UR5_J_inv*dJdq5);
UR5_dP(10,1)=0;
% UR5_w_2
% UR5_dP
% pause()


end