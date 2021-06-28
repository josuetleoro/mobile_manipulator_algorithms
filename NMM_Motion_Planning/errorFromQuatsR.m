function eO=errorFromQuatsR(Rd,Re)
Rerror=Rd*Re';
qerror=rotm2quat(Rerror)';
%Calculate the orientation error
eO=qerror(2:4);
end