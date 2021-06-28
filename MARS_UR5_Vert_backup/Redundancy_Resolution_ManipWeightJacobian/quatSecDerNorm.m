function ddq=quatSecDerNorm(w,dw,dN,ddN,q)
ddq=1/2*dw*q+dN*w*q-1/4*norm(w.getV())*q+ddN*q;
end