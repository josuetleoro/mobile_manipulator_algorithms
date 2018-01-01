function [w]=UR5_manGrad(q)

%Evaluate the matrices JJt and dJJt
[J, dJJtdq1, dJJtdq2, dJJtdq3, dJJtdq4, dJJtdq5]=UR5_eval_JJt(q(1),q(2),q(3),q(4),q(5));

%Get the manipulability measure
JJt=J*J';
det_JJt=det(JJt);
w=sqrt(det_JJt);

end