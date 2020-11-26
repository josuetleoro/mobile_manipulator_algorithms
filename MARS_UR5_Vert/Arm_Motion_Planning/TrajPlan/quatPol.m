function q=quatPol(tau,p0,p1,p2,p3,p4,p5)
q=(1-tau)^3*(p0+p1*tau+p2*tau^2)+tau^3*(p3+p4*(1-tau)+p5*(1-tau)^2);
end