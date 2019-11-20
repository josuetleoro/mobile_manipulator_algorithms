clear all

%% Mobile manipulator
% alpha = 0.01;
% q = rand(10,1);
% q(4) = 0.125;
% q(5:end)=[0.0;-0.40;1.06;5*pi/4;-pi/2;0.0];
% q_prev = 0;
% w_max = 0;
% while true
% 
% if q(4) > 0.25
%     q(4) = 0.25;
% end
% if q(4) < 0
%    q(4) = 0; 
% end    
%     
% %Calculate the Jacobian
% J=evaluateJBar(q(3),q(5),q(6),q(7),q(8),q(9));
% [dJdq3,dJdq5,dJdq6,dJdq7,dJdq8,dJdq9]=evaluate_dJbardq(q(3),q(5),q(6),q(7),q(8),q(9));
% 
% %Calculate the manipulability measure
% Jt=J';
% JJt=J*Jt;
% w=sqrt(det(JJt));
% 
% if w > w_max
%     w_max = w;
% end
% 
% %Calculate each of the elements of the gradient 
% inv_JJt=inv(JJt);
% w_2=w/2; %We use -w because the internal motion is substracted
% dP(1,1)=0;
% dP(2,1)=0;
% dP(3,1)=w_2*trace(inv_JJt*(dJdq3*Jt+J*(dJdq3')));
% dP(4,1)=0;
% dP(5,1)=w_2*trace(inv_JJt*(dJdq5*Jt+J*(dJdq5')));
% dP(6,1)=w_2*trace(inv_JJt*(dJdq6*Jt+J*(dJdq6')));
% dP(7,1)=w_2*trace(inv_JJt*(dJdq7*Jt+J*(dJdq7')));
% dP(8,1)=w_2*trace(inv_JJt*(dJdq8*Jt+J*(dJdq8')));
% dP(9,1)=w_2*trace(inv_JJt*(dJdq9*Jt+J*(dJdq9')));
% dP(10,1)=0;
% 
% q_prev_a = q;
% q = q + alpha*dP;
% 
% error = dot(q-q_prev_a, q-q_prev_a)
% if (error <= 1e-35)
%     break;
% end
% end
% q
% w_max

% %% Robot arm
% alpha = 0.05;
% qa=[0.0;-0.40;1.06;5*pi/4;-pi/2;0.0];
% %qa=rand(6,1)*pi;
% q_prev_a = 0;
% wa_max = 0;
% while true
% Ja=evaluate_Ja(qa(1),qa(2),qa(3),qa(4),qa(5));
% [dJadq5,dJadq6,dJadq7,dJadq8,dJadq9]=evaluate_dJadq(qa(1),qa(2),qa(3),qa(4),qa(5));
% 
% wa=abs(det(Ja));
% if wa > wa_max
%     wa_max = wa;
% end
% 
% %Calculate each of the elements of the gradient 
% inv_Ja=inv(Ja);
% 
% dPa(1,1)=wa*trace(inv_Ja*dJadq5);
% dPa(2,1)=wa*trace(inv_Ja*dJadq6);
% dPa(3,1)=wa*trace(inv_Ja*dJadq7);
% dPa(4,1)=wa*trace(inv_Ja*dJadq8);
% dPa(5,1)=wa*trace(inv_Ja*dJadq9);
% dPa(6,1)=0;
% 
% q_prev_a = qa;
% qa = qa + alpha*dPa;
% 
% error = dot(qa-q_prev_a, qa-q_prev_a)
% if (error <= 1e-35)
%     break;
% end
% end
% qa
% wa_max

%% Mobile manipulator + arm
alpha = 0.01;
q = rand(10,1);
q(4) = 0.125;
q(5:end)=rand(6,1)*pi/2;
%q(5:end)=[0.0;-0.40;1.06;5*pi/4;-pi/2;0.0]
%q(5:end)=[-0.14533,-0.5708,1.1,-2.0832,-1.5832,0]
q = [0, 0, 0, 0.125, -0.24533, -0.070796, 0.25, -1.7832, pi/2, 0]';
wM = 0;
wM_max = 0;
while true

%Calculate the Jacobian
J=evaluateJBar(q(3),q(5),q(6),q(7),q(8),q(9));
[dJdq3,dJdq5,dJdq6,dJdq7,dJdq8,dJdq9]=evaluate_dJbardq(q(3),q(5),q(6),q(7),q(8),q(9));

%Calculate the manipulability measure
Jt=J';
JJt=J*Jt;
w=sqrt(det(JJt));

%Calculate each of the elements of the gradient 
inv_JJt=inv(JJt);
w_2=w/2; %We use -w because the internal motion is substracted
dP(1,1)=0;
dP(2,1)=0;
dP(3,1)=w_2*trace(inv_JJt*(dJdq3*Jt+J*(dJdq3')));
dP(4,1)=0;
dP(5,1)=w_2*trace(inv_JJt*(dJdq5*Jt+J*(dJdq5')));
dP(6,1)=w_2*trace(inv_JJt*(dJdq6*Jt+J*(dJdq6')));
dP(7,1)=w_2*trace(inv_JJt*(dJdq7*Jt+J*(dJdq7')));
dP(8,1)=w_2*trace(inv_JJt*(dJdq8*Jt+J*(dJdq8')));
dP(9,1)=w_2*trace(inv_JJt*(dJdq9*Jt+J*(dJdq9')));
dP(10,1)=0;
    
    
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

wM = w+wa;
if wM > wM_max
    wM_max = wM;
end

dP = dP + dPa;
q_prev = q;
q = q + alpha*dP;

error = dot(q-q_prev, q-q_prev)
if (error <= 1e-35)
    break;
end
end
q
wM_max    

% %% Mobile manipulator * arm
% alpha = 0.01;
% q = rand(10,1);
% q(4) = 0.125;
% %q(5:end)=rand(6,1)*pi/2
% %q(5:end)=[0.0;-0.40;1.06;5*pi/4;-pi/2;0.0]
% %q(5:end)=[-0.14533,-0.5708,1.1,-2.0832,-1.5832,0]
% q = [0, 0, 0, 0.125, -0.12217, -0.5236, 1.0996, -2.1468, -4.7124, 0]';
% wM = 0;
% wM_max = 0;
% while true
% 
% %Calculate the Jacobian
% J=evaluateJBar(q(3),q(5),q(6),q(7),q(8),q(9));
% [dJdq3,dJdq5,dJdq6,dJdq7,dJdq8,dJdq9]=evaluate_dJbardq(q(3),q(5),q(6),q(7),q(8),q(9));
% 
% %Calculate the manipulability measure
% Jt=J';
% JJt=J*Jt;
% w=sqrt(det(JJt));
% 
% %Calculate each of the elements of the gradient 
% inv_JJt=inv(JJt);
% w_2=w/2; %We use -w because the internal motion is substracted
% dP(1,1)=0;
% dP(2,1)=0;
% dP(3,1)=w_2*trace(inv_JJt*(dJdq3*Jt+J*(dJdq3')));
% dP(4,1)=0;
% dP(5,1)=w_2*trace(inv_JJt*(dJdq5*Jt+J*(dJdq5')));
% dP(6,1)=w_2*trace(inv_JJt*(dJdq6*Jt+J*(dJdq6')));
% dP(7,1)=w_2*trace(inv_JJt*(dJdq7*Jt+J*(dJdq7')));
% dP(8,1)=w_2*trace(inv_JJt*(dJdq8*Jt+J*(dJdq8')));
% dP(9,1)=w_2*trace(inv_JJt*(dJdq9*Jt+J*(dJdq9')));
% dP(10,1)=0;
%     
%     
% Ja=evaluate_Ja(q(5),q(6),q(7),q(8),q(9));
% [dJadq5,dJadq6,dJadq7,dJadq8,dJadq9]=evaluate_dJadq(q(5),q(6),q(7),q(8),q(9));
% 
% wa=abs(det(Ja));
% 
% %Calculate each of the elements of the gradient 
% inv_Ja=inv(Ja);
% dPa(1,1)=0;
% dPa(2,1)=0;
% dPa(3,1)=0;
% dPa(4,1)=0;
% dPa(5,1)=wa*trace(inv_Ja*dJadq5);
% dPa(6,1)=wa*trace(inv_Ja*dJadq6);
% dPa(7,1)=wa*trace(inv_Ja*dJadq7);
% dPa(8,1)=wa*trace(inv_Ja*dJadq8);
% dPa(9,1)=wa*trace(inv_Ja*dJadq9);
% dPa(10,1)=0;
% 
% wM = w*wa;
% if wM > wM_max
%     wM_max = wM;
% end
% 
% dP = wa*dP + w*dPa;
% q_prev = q;
% q = q + alpha*dP;
% 
% error = dot(q-q_prev, q-q_prev)
% if (error <= 1e-35)
%     break;
% end
% end
% q
% wM_max    