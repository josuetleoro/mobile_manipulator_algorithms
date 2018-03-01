function w=quatRatesToAxesRates(quat,dquat)
% %First representation
% Qr=[-quat(2), quat(1),-quat(4), quat(3);
%     -quat(3), quat(4), quat(1),-quat(2);
%     -quat(4),-quat(3), quat(2), quat(1)];
% w=2*Qr*dquat;

%Second representation
    q=Quat(quat');
    dq=Quat(dquat');
    w4=2*dq*q.inv();
    w=w4.getV();
    
% %Third representation
% Qr=[ quat(1), quat(4),-quat(3), quat(2);
%     -quat(4), quat(1), quat(2), quat(3);
%      quat(3),-quat(2), quat(1), quat(4);
%     -quat(2),-quat(3),-quat(4), quat(1)];
% w4=2*Qr'*dquat
% w=w4(1:3);

end