function eO=errorFromQuats(qd,qe)
%Calculate the orientation error
eO=qe(1)*qd(2:4)-qd(1)*qe(2:4)-cross(qd(2:4),qe(2:4));
end