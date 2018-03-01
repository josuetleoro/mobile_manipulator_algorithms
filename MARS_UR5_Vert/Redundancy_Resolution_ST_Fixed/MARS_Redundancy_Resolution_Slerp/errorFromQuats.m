function eO=errorFromQuats(qd,qe)

%Create the skew matrix from qd_v
%S_vd=skewMatrix(qd(2:4));

%Calculate the orientation error
%eO=qe(1)*qd(2:4)-qd(1)*qe(2:4)-S_vd*qe(2:4);
eO=qe(1)*qd(2:4)-qd(1)*qe(2:4)-cross(qd(2:4),qe(2:4));

end