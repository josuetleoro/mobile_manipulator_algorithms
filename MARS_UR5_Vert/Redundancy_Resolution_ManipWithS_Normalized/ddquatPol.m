function ddq=ddquatPol(t,ti,tf,p0,p1,p2,p3,p4,p5)
ddq = (6*(1-(t-ti)/(tf-ti)))*(p0+p1*(t-ti)/(tf-ti)+p2*(t-ti)^2/(tf-ti)^2)/(tf-ti)^2-6*(1-(t-ti)/(tf-ti))^2*(p1/(tf-ti)+2*p2*(t-ti)/(tf-ti)^2)/(tf-ti)+2*(1-(t-ti)/(tf-ti))^3*p2/(tf-ti)^2+(6*(t-ti))*(p3+p4*(1-(t-ti)/(tf-ti))+p5*(1-(t-ti)/(tf-ti))^2)/(tf-ti)^3+6*(t-ti)^2*(-1*p4/(tf-ti)-2*p5*(1-(t-ti)/(tf-ti))/(tf-ti))/(tf-ti)^3+2*(t-ti)^3*p5/(tf-ti)^5;
end