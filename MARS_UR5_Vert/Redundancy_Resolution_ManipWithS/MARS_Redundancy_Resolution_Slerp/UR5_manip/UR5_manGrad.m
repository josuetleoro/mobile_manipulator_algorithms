function [dP,w]=UR5_manGrad(q)

%Evaluate the matrices JJt and dJJt
[J, dJJtdq1, dJJtdq2, dJJtdq3, dJJtdq4, dJJtdq5]=UR5_eval_dJJtdq(q(1),q(2),q(3),q(4),q(5));

J

pause()
%Get the manipulability measure
JJt=J*J';
det_JJt=det(JJt);
w=sqrt(det_JJt);

%Calculate each of the elements of the gradient 
inv_JJt=inv(JJt);
w_2=-w/2; %We use -w because the internal motion is substracted
dP(1,1)=w_2*trace(inv_JJt*dJJtdq1);
dP(2,1)=w_2*trace(inv_JJt*dJJtdq2);
dP(3,1)=w_2*trace(inv_JJt*dJJtdq3);
dP(4,1)=w_2*trace(inv_JJt*dJJtdq4);
dP(5,1)=w_2*trace(inv_JJt*dJJtdq5);
dP(6,1)=0;

end