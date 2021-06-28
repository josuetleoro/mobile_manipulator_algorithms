function [pout]=rotPointQuat(qrmp,qrmpc,pin)
qpin=Quat(0,pin);
qpin_phi=qrmp*qpin*qrmpc; %Rotate the point
pout=qpin_phi.getV();
end