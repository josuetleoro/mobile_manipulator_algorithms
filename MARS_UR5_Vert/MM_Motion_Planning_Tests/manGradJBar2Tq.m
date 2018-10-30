function [dP,w,dPa,wa]=manGradJBar2Tq(q,J,invTq)
%Calculates the manipulability of the system and the arm alone. 
%JBar is used for manip of the system. JBar is 9x1, however, the gradient is considered
%as a 10x1 by adding a zero for the y direction.
%The Jacobian of UR5 is calculated separetely, without considering the
%orientation of the platform.
%Note: the derivative is performed on JBar, therefore dq2(JBar), 
%corresponds to dq3(J)

%% Mobile Manipulator
[dJdq3,dJdq5,dJdq6,dJdq7,dJdq8,dJdq9]=evaluate_dJbardq(q(3),q(5),q(6),q(7),q(8),q(9));

J=J*invTq;
%Calculate the manipulability measure
% eigval=svd(J);
% w=prod(eigval);
Jt=J';
JJt=J*Jt;
w=sqrt(det(JJt));

%Calculate each of the elements of the gradient 
inv_JJt=inv(JJt);
w_2=w/2; %We use -w because the internal motion is substracted
dP(1,1)=0;
dP(2,1)=0;
dP(3,1)=w_2*trace(inv_JJt*(dJdq3*invTq*invTq*Jt+J*invTq*invTq*dJdq3'));
dP(4,1)=0;
dP(5,1)=w_2*trace(inv_JJt*(dJdq5*invTq*invTq*Jt+J*invTq*invTq*dJdq5'));
dP(6,1)=w_2*trace(inv_JJt*(dJdq6*invTq*invTq*Jt+J*invTq*invTq*dJdq6'));
dP(7,1)=w_2*trace(inv_JJt*(dJdq7*invTq*invTq*Jt+J*invTq*invTq*dJdq7'));
dP(8,1)=w_2*trace(inv_JJt*(dJdq8*invTq*invTq*Jt+J*invTq*invTq*dJdq8'));
dP(9,1)=w_2*trace(inv_JJt*(dJdq9*invTq*invTq*Jt+J*invTq*invTq*dJdq9'));
dP(10,1)=0;
% w_2
% dP

%% UR5
Ja=evaluate_Ja(q(5),q(6),q(7),q(8),q(9));
Ja=Ja*invTq(4:9,4:9);
[dJadq5,dJadq6,dJadq7,dJadq8,dJadq9]=evaluate_dJadq(q(5),q(6),q(7),q(8),q(9));

wa=abs(det(Ja));

%Calculate each of the elements of the gradient 
inv_Ja=inv(Ja);
dPa(1,1)=0;
dPa(2,1)=0;
dPa(3,1)=0;
dPa(4,1)=0;
dPa(5,1)=wa*trace(inv_Ja*dJadq5*invTq(4:9,4:9));
dPa(6,1)=wa*trace(inv_Ja*dJadq6*invTq(4:9,4:9));
dPa(7,1)=wa*trace(inv_Ja*dJadq7*invTq(4:9,4:9));
dPa(8,1)=wa*trace(inv_Ja*dJadq8*invTq(4:9,4:9));
dPa(9,1)=wa*trace(inv_Ja*dJadq9*invTq(4:9,4:9));
dPa(10,1)=0;
% UR5_w_2
% UR5_dP
% pause()


end