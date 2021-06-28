function [dP,w,dPa,wa]=manGradJ2(q,J)
%Calculates the manipulability of the system and the arm alone. 
%J is used for manip of the system. 
%The Jacobian of UR5 is calculated separetely, without considering the
%orientation of the platform.
%% Mobile Manipulator
[dJdq3,dJdq5,dJdq6,dJdq7,dJdq8,dJdq9]=evaluate_dJdq(q(3),q(5),q(6),q(7),q(8),q(9));

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
dP(3,1)=w_2*trace(inv_JJt*(dJdq3*Jt+J*dJdq3'));
dP(4,1)=0;
dP(5,1)=w_2*trace(inv_JJt*(dJdq5*Jt+J*dJdq5'));
dP(6,1)=w_2*trace(inv_JJt*(dJdq6*Jt+J*dJdq6'));
dP(7,1)=w_2*trace(inv_JJt*(dJdq7*Jt+J*dJdq7'));
dP(8,1)=w_2*trace(inv_JJt*(dJdq8*Jt+J*dJdq8'));
dP(9,1)=w_2*trace(inv_JJt*(dJdq9*Jt+J*dJdq9'));
dP(10,1)=0;
% w_2
% dP

%% UR5
Ja=evaluate_Ja(q(5),q(6),q(7),q(8),q(9));
[dJadq5,dJadq6,dJadq7,dJadq8,dJadq9]=evaluate_dJadq(q(5),q(6),q(7),q(8),q(9));

wa=abs(det(Ja));

%Calculate each of the elements of the gradient 
inv_Ja=inv(Ja);
dPa(1,1)=0;
dPa(2,1)=0;
dPa(3,1)=0;
dPa(4,1)=0;
dPa(5,1)=wa*trace(inv_Ja*dJadq5);
dPa(6,1)=wa*trace(inv_Ja*dJadq6);
dPa(7,1)=wa*trace(inv_Ja*dJadq7);
dPa(8,1)=wa*trace(inv_Ja*dJadq8);
dPa(9,1)=wa*trace(inv_Ja*dJadq9);
dPa(10,1)=0;

% UR5_w_2
% UR5_dP
% pause()


end