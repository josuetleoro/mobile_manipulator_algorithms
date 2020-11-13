 function q=cartToQuat(R)
%%%%%%%%%%%%%% Matlab Aerospace toolbox %%%%%%%%%%%%
quat_aux = rotm2quat(R);
q=quat_aux';
 end