function [dP,w,UR5_dP,UR5_w]=manGrad3(q,J)
%Calculates the manipulability of the system and the arm alone

%The first element, second and fourth element are zero (x,y,z does not
%exist in the jacobian matrix, therefore the derivative of JJt with respect
%to these is zero). The same happens with the last joint q10.
%It is important to consider that the derivative is performed on JBar,
%therefore dq2 (JBar), corresponds to dq3 (J)
[dJJtdq3_ev,dJJtdq5_ev,dJJtdq6_ev,dJJtdq7_ev,dJJtdq8_ev,dJJtdq9_ev]=evaluatedJJtdq(q(3),q(5),q(6),q(7),q(8),q(9));

%% Mobile Manipulator

%Calculate JJt
JJt=J*J';

%Calculate the manipulability measure
%eigval=svd(JJt);
[U,S,V]=svd(JJt);
w=prod(diag(S))^(1/9);

%Calculate each of the elements of the gradient 
inv_JJt=inv(JJt);
%inv_JJt=V*inv(S)*U';
w_m=w/9; %We use -w because the internal motion is substracted
dP(1,1)=0;
dP(2,1)=0;
dP(3,1)=w_m*trace(inv_JJt*dJJtdq3_ev);
dP(4,1)=0;
dP(5,1)=w_m*trace(inv_JJt*dJJtdq5_ev);
dP(6,1)=w_m*trace(inv_JJt*dJJtdq6_ev);
dP(7,1)=w_m*trace(inv_JJt*dJJtdq7_ev);
dP(8,1)=w_m*trace(inv_JJt*dJJtdq8_ev);
dP(9,1)=w_m*trace(inv_JJt*dJJtdq9_ev);
dP(10,1)=0;
% w_2
% dP

%% UR5

%Get the manipulability measure
UR5_J=evaluateUR5J(q(5),q(6),q(7),q(8),q(9));
UR5_J=UR5_J*UR5_J';

%eigval=svd(JJt);
%[U,S,V]=svd(UR5_J);
UR5_w=det(UR5_J)^(1/6);

[dJdq1,dJdq2,dJdq3,dJdq4,dJdq5]=evaluateUR5dJdq(q(5),q(6),q(7),q(8),q(9));

%Calculate each of the elements of the gradient 
UR5_J_inv=inv(UR5_J);
%UR5_J_inv=V*inv(S)*U';
UR5_w_m=UR5_w/6; %We use -w because the internal motion is substracted
UR5_dP(1,1)=0;
UR5_dP(2,1)=0;
UR5_dP(3,1)=0;
UR5_dP(4,1)=0;
UR5_dP(5,1)=UR5_w_m*trace(UR5_J_inv*dJdq1);
UR5_dP(6,1)=UR5_w_m*trace(UR5_J_inv*dJdq2);
UR5_dP(7,1)=UR5_w_m*trace(UR5_J_inv*dJdq3);
UR5_dP(8,1)=UR5_w_m*trace(UR5_J_inv*dJdq4);
UR5_dP(9,1)=UR5_w_m*trace(UR5_J_inv*dJdq5);
UR5_dP(10,1)=0;

end