function [dP,w,dPa,wa]=manGradJBar(q,J)
%Calculates the manipulability of the system and the arm using using JBar
%for all the calculations. JBar is 9x1, however, the gradient is considered
%as a 10x1 by adding a zero for the y direction.
%Note: the derivative is performed on JBar, therefore dq2(JBar), 
%corresponds to dq3(J)

%% Mobile Manipulator
[dJdq3,dJdq5,dJdq6,dJdq7,dJdq8,dJdq9]=evaluate_dJbardq(q(3),q(5),q(6),q(7),q(8),q(9));

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
Ja=J(:,4:9);
dJadq3=dJdq3(:,4:9);
dJadq5=dJdq5(:,4:9);
dJadq6=dJdq6(:,4:9);
dJadq7=dJdq7(:,4:9);
dJadq8=dJdq8(:,4:9);
dJadq9=dJdq9(:,4:9);

Jat=Ja';
JaJat=Ja*Jat;
wa=sqrt(det(JaJat));

%Calculate each of the elements of the gradient 
inv_JaJat=inv(JaJat);
wa_2=wa/2; %We use -w because the internal motion is substracted
dPa(1,1)=0;
dPa(2,1)=0;
dPa(3,1)=wa_2*trace(inv_JaJat*(dJadq3*Jat+Ja*dJadq3'));
dPa(4,1)=0;
dPa(5,1)=wa_2*trace(inv_JaJat*(dJadq5*Jat+Ja*dJadq5'));
dPa(6,1)=wa_2*trace(inv_JaJat*(dJadq6*Jat+Ja*dJadq6'));
dPa(7,1)=wa_2*trace(inv_JaJat*(dJadq7*Jat+Ja*dJadq7'));
dPa(8,1)=wa_2*trace(inv_JaJat*(dJadq8*Jat+Ja*dJadq8'));
dPa(9,1)=wa_2*trace(inv_JaJat*(dJadq9*Jat+Ja*dJadq9'));
dPa(10,1)=0;
% UR5_w_2
% UR5_dP
% pause()


end