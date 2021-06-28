function [pout]=rotPointDualQuat(p,Q)
%Extract the primary and dual part from Q
q=Q.prim;
q0=Q.dual;

%Create the quaternion representing the point
qp=Quat(0,p);

%Use the rotation formula
qP=q*qp*q.conj()+q0*q.conj()-q*q0.conj();

pout=qP.getV();

end