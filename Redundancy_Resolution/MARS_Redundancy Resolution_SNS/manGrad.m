function [dP,w]=manGrad(q,Jev)

%Evaluate the matrices JJt and dJJt
JJt_ev=evaluateJJt(q(3),q(5),q(6),q(7),q(8),q(9));

%The first element, second and fourth element are zero (x,y,z does not
%exist in the jacobian matrix, therefore the derivative of JJt with respect
%to these is zero). The same happens with the last joint q10.
[dJJtdq3_ev,dJJtdq5_ev,dJJtdq6_ev,dJJtdq7_ev,dJJtdq8_ev,dJJtdq9_ev]=evaluatedJJtdq(q(3),q(5),q(6),q(7),q(8),q(9));

%Find the determinant and inverse of JJt
%det_JJt=det(JJt_ev);
%inv_JJt=inv(JJt_ev);
%w=-1/2*prod(svd(Jev));

inv_JJt=inv(JJt_ev);
S=svd(Jev);
w=prod(S);
w_2=-w/2;

%Calculate each of the elements of the gradient 
dP(1,1)=0;
dP(2,1)=0;
dP(3,1)=w_2*trace(inv_JJt*dJJtdq3_ev);
dP(4,1)=0;
dP(5,1)=w_2*trace(inv_JJt*dJJtdq5_ev);
dP(6,1)=w_2*trace(inv_JJt*dJJtdq6_ev);
dP(7,1)=w_2*trace(inv_JJt*dJJtdq7_ev);
dP(8,1)=w_2*trace(inv_JJt*dJJtdq8_ev);
dP(9,1)=w_2*trace(inv_JJt*dJJtdq9_ev);
dP(10,1)=0;

% dP(1,1)=0;
% dP(2,1)=0;
% dP(3,1)=w*trace(JJt_ev\dJJtdq3_ev);
% dP(4,1)=0;
% dP(5,1)=w*trace(JJt_ev\dJJtdq5_ev);
% dP(6,1)=w*trace(JJt_ev\dJJtdq6_ev);
% dP(7,1)=w*trace(JJt_ev\dJJtdq7_ev);
% dP(8,1)=w*trace(JJt_ev\dJJtdq8_ev);
% dP(9,1)=w*trace(JJt_ev\dJJtdq9_ev);
% dP(10,1)=0;

end