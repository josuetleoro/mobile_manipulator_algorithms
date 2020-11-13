function T = posQuat2RotMat(xi)
%First check the input has the required size
if size(xi,1)~=7
    error('A pos_quat element must have 7 elements')
end

R=quat2rotm(xi(4:7)');
T=zeros(4,4);
T(4,4)=1;
T(1:3,1:3)=R;
T(1:3,4)=xi(1:3);

end