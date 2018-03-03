function [dP,w,UR5_dP,UR5_w,w5]=manGrad(q,J)

%The first element, second and fourth element are zero (x,y,z does not
%exist in the jacobian matrix, therefore the derivative of JJt with respect
%to these is zero). The same happens with the last joint q10.
[dJJtdq2_ev,dJJtdq4_ev,dJJtdq5_ev,dJJtdq6_ev,dJJtdq7_ev,dJJtdq8_ev]=evaluatedJJtdq(q(3),q(5),q(6),q(7),q(8),q(9));

%% Mobile Manipulator

%Calculate JJt
JJt=J*J';

%Calculate the manipulability measure
S=svd(J);
w=prod(S);
w5=sqrt(1-(S(end)^2/S(1)^2));

%det_JJt=det(JJt);
%w=sqrt(det_JJt);

%Calculate each of the elements of the gradient 
inv_JJt=inv(JJt);
w_2=-w/2; %We use -w because the internal motion is substracted
dP(1,1)=0;
dP(2,1)=w_2*trace(inv_JJt*dJJtdq2_ev);
dP(3,1)=0;
dP(4,1)=w_2*trace(inv_JJt*dJJtdq4_ev);
dP(5,1)=w_2*trace(inv_JJt*dJJtdq5_ev);
dP(6,1)=w_2*trace(inv_JJt*dJJtdq6_ev);
dP(7,1)=w_2*trace(inv_JJt*dJJtdq7_ev);
dP(8,1)=w_2*trace(inv_JJt*dJJtdq8_ev);
dP(9,1)=0;

%% UR5

%Get the manipulability measure
UR5_J=J(:,4:9);
UR5_JJt=UR5_J*UR5_J';
UR5_det_JJt=det(UR5_JJt);
UR5_w=sqrt(UR5_det_JJt);

%Calculate each of the elements of the gradient 
UR5_inv_JJt=inv(UR5_JJt);
UR5_w_2=-UR5_w/2; %We use -w because the internal motion is substracted
UR5_dP(1,1)=0;
UR5_dP(2,1)=UR5_w_2*trace(UR5_inv_JJt*dJJtdq2_ev);
UR5_dP(3,1)=0;
UR5_dP(4,1)=UR5_w_2*trace(UR5_inv_JJt*dJJtdq4_ev);
UR5_dP(5,1)=UR5_w_2*trace(UR5_inv_JJt*dJJtdq5_ev);
UR5_dP(6,1)=UR5_w_2*trace(UR5_inv_JJt*dJJtdq6_ev);
UR5_dP(7,1)=UR5_w_2*trace(UR5_inv_JJt*dJJtdq7_ev);
UR5_dP(8,1)=UR5_w_2*trace(UR5_inv_JJt*dJJtdq8_ev);
UR5_dP(9,1)=0;

end