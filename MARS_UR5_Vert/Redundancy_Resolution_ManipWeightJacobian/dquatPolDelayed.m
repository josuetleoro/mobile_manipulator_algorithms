function dq=dquatPolDelayed(t,h,tf,p0,p1,p2,p3,p4,p5)
dq = -6*(1-2*h/(tf-t+h))^2*(p0+2*p1*h/(tf-t+h)+4*p2*h^2/(tf-t+h)^2)*h/(tf-t+h)^2+(1-2*h/(tf-t+h))^3*(2*p1*h/(tf-t+h)^2+8*p2*h^2/(tf-t+h)^3)+24*h^3*(p3+p4*(1-2*h/(tf-t+h))+p5*(1-2*h/(tf-t+h))^2)/(tf-t+h)^4+8*h^3*(-2*p4*h/(tf-t+h)^2-4*p5*(1-2*h/(tf-t+h))*h/(tf-t+h)^2)/(tf-t+h)^3;
end