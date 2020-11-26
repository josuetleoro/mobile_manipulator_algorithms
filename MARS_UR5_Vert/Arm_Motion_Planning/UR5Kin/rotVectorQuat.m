function [pout]=rotVectorQuat(v,q)
%Efficient quaternion rotation of a vector
t=2*cross(q.getV(),v);
pout=v+q.getS()*t+cross(q.getV(),t);
end