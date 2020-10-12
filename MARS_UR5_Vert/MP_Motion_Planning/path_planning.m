function [x_des,y_des,theta_des,v_des,w_des]=path_planning(xi,yi,thetai,xf,yf,thetaf,k,k_dir,tf,ts)
if k_dir == 1
    k = -1*k;
end

t=0:ts:tf;
tao=t/tf;

alpha_x=k*cos(thetaf)-3*xf;
alpha_y=k*sin(thetaf)-3*yf;

beta_x=k*cos(thetai)+3*xi;
beta_y=k*sin(thetai)+3*yi;

N=length(tao);
x_des=zeros(1,N);
y_des=zeros(1,N);
theta_des=zeros(1,N);
xp_des=zeros(1,N);
yp_des=zeros(1,N);
xpp_des=zeros(1,N);
ypp_des=zeros(1,N);
v_des=zeros(1,N);
w_des=zeros(1,N);

for i=1:N
    s=tao(i);
    x_des(i)=s^3*xf-(s-1)^3*xi+alpha_x*s^2*(s-1)+beta_x*s*(s-1)^2;
    y_des(i)=s^3*yf-(s-1)^3*yi+alpha_y*s^2*(s-1)+beta_y*s*(s-1)^2;
    xp_des(i)=3*s^2*xf-3*(s-1)^2*xi+2*alpha_x*s*(s-1)+alpha_x*s^2+beta_x*(s-1)^2+2*beta_x*s*(s-1);
    yp_des(i)=3*s^2*yf-3*(s-1)^2*yi+2*alpha_y*s*(s-1)+alpha_y*s^2+beta_y*(s-1)^2+2*beta_y*s*(s-1);
    xpp_des(i)=(s-1)*(2*alpha_x-6*xi+4*beta_x)+s*(6*xf+4*alpha_x+2*beta_x);
    ypp_des(i)=(s-1)*(2*alpha_y-6*yi+4*beta_y)+s*(6*yf+4*alpha_y+2*beta_y);
    theta_des(i)=atan2(yp_des(i),xp_des(i))+k_dir*pi;
    
    % Make sure theta are bounded by [-pi,pi]
    theta_des(i)= mod((theta_des(i) + pi),2*pi)-pi;    
    
    if k_dir == 0
        v_des(i)=1/tf*sqrt(xp_des(i)^2+yp_des(i)^2);
    else
        v_des(i)=-1/tf*sqrt(xp_des(i)^2+yp_des(i)^2);
    end
    w_des(i)=1/tf*(ypp_des(i)*xp_des(i)-xpp_des(i)*yp_des(i))/(xp_des(i)^2+(yp_des(i)^2));
end
end