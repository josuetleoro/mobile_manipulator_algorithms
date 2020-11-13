function [pout]=rotVectorDualQuat(v,Q)
%Extract the primary part from Q
q=Q.prim;

%Use the rotation formula
pout=rotVectorQuat(v,q);
end