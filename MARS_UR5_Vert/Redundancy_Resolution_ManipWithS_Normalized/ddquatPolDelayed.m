function ddq=ddquatPolDelayed(t,h,tf,p0,p1,p2,p3,p4,p5)
ddq = (24*(1-2*h/(tf-t+h)))*(p0+2*p1*h/(tf-t+h)+4*p2*h^2/(tf-t+h)^2)*h^2/(tf-t+h)^4-12*(1-2*h/(tf-t+h))^2*(2*p1*h/(tf-t+h)^2+8*p2*h^2/(tf-t+h)^3)*h/(tf-t+h)^2-12*(1-2*h/(tf-t+h))^2*(p0+2*p1*h/(tf-t+h)+4*p2*h^2/(tf-t+h)^2)*h/(tf-t+h)^3+(1-2*h/(tf-t+h))^3*(4*p1*h/(tf-t+h)^3+24*p2*h^2/(tf-t+h)^4)+96*h^3*(p3+p4*(1-2*h/(tf-t+h))+p5*(1-2*h/(tf-t+h))^2)/(tf-t+h)^5+48*h^3*(-2*p4*h/(tf-t+h)^2-4*p5*(1-2*h/(tf-t+h))*h/(tf-t+h)^2)/(tf-t+h)^4+8*h^3*(-4*p4*h/(tf-t+h)^3+8*p5*h^2/(tf-t+h)^4-8*p5*(1-2*h/(tf-t+h))*h/(tf-t+h)^3)/(tf-t+h)^3;
end