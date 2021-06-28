function eO=errorFromQuats(qd,qe)
%Calculate the orientation error
sO=qd(1)*qe(1)+qd(2:4)'*qe(2:4);
eO=qe(1)*qd(2:4)-qd(1)*qe(2:4)-cross(qd(2:4),qe(2:4));
% Since by convention the quaternion scalar part must be kept positive,
% multiply the quaternion by -1 when the scalar part is negative.
if sO < 0
    eO = -1*eO;
end
%Matlab 2018b
% Qd=quaternion(qd(1),qd(2),qd(3),qd(4));
% Qe=quaternion(qe(1),qe(2),qe(3),qe(4));
% deltaQ=Qd*conj(Qe);
% deltaQvec=compact(deltaQ);
% if deltaQvec < 0
%     eO=-1*deltaQvec(2:4)';
% else
%     eO=deltaQvec(2:4)';
% end
end