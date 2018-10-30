function dq=dquatPol(t,ti,tf,p0,p1,p2,p3,p4,p5)
dq = -3*(1-(t-ti)/(tf-ti))^2*(p0+p1*(t-ti)/(tf-ti)+p2*(t-ti)^2/(tf-ti)^2)/(tf-ti)+(1-(t-ti)/(tf-ti))^3*(p1/(tf-ti)+2*p2*(t-ti)/(tf-ti)^2)+3*(t-ti)^2*(p3+p4*(1-(t-ti)/(tf-ti))+p5*(1-(t-ti)/(tf-ti))^2)/(tf-ti)^3+(t-ti)^3*(-1*p4/(tf-ti)-2*p5*(1-(t-ti)/(tf-ti))/(tf-ti))/(tf-ti)^3;
end