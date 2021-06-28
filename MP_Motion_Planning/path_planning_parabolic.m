function [x_des,y_des,theta_des,v_des,w_des]=path_planning_parabolic(xi,yi,thetai,xf,yf,thetaf,k,k_dir,tf,ts,tb)
if k_dir == 1
    k = -1*k;
end

sf=1;
max_vel=sf/(tf-tb);
max_a=max_vel/tb;

%% Define the evolution of parameter s

i=1;    %Element position
%Initial parabolic segment
for t=0:ts:tb
    s(i)=max_a/2*t^2;
    ds(i)=max_a*t;
    time(i)=t;
    i=i+1;
end
%Knot point
pa_x=s(i-1);
%Linear segment
for t=time(i-1)+ts:ts:tf-tb
    s(i)=pa_x+max_vel*(t-tb);
    ds(i)=max_vel;
    time(i)=t;
    i=i+1;
end

%Final parabolic segment
for t=time(i-1)+ts:ts:tf
    %Initial parabolic segment
    s(i)=sf-max_a/2*(tf-t)^2;
    ds(i)=max_a*(tf-t);
    time(i)=t;
    i=i+1;
end

alpha_x=k*cos(thetaf)-3*xf;
alpha_y=k*sin(thetaf)-3*yf;

beta_x=k*cos(thetai)+3*xi;
beta_y=k*sin(thetai)+3*yi;

N=length(s);
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
    x_des(i)=s(i)^3*xf-(s(i)-1)^3*xi+alpha_x*s(i)^2*(s(i)-1)+beta_x*s(i)*(s(i)-1)^2;
    y_des(i)=s(i)^3*yf-(s(i)-1)^3*yi+alpha_y*s(i)^2*(s(i)-1)+beta_y*s(i)*(s(i)-1)^2;
    xp_des(i)=3*s(i)^2*xf-3*(s(i)-1)^2*xi+2*alpha_x*s(i)*(s(i)-1)+alpha_x*s(i)^2+beta_x*(s(i)-1)^2+2*beta_x*s(i)*(s(i)-1);
    yp_des(i)=3*s(i)^2*yf-3*(s(i)-1)^2*yi+2*alpha_y*s(i)*(s(i)-1)+alpha_y*s(i)^2+beta_y*(s(i)-1)^2+2*beta_y*s(i)*(s(i)-1);
    xpp_des(i)=(s(i)-1)*(2*alpha_x-6*xi+4*beta_x)+s(i)*(6*xf+4*alpha_x+2*beta_x);
    ypp_des(i)=(s(i)-1)*(2*alpha_y-6*yi+4*beta_y)+s(i)*(6*yf+4*alpha_y+2*beta_y);
    theta_des(i)=atan2(yp_des(i),xp_des(i))+k_dir*pi;
    
    % Make sure theta are bounded by [-pi,pi]
    theta_des(i)= mod((theta_des(i) + pi),2*pi)-pi;    
    
    if k_dir == 0
        v_des(i)=sqrt(xp_des(i)^2+yp_des(i)^2)*ds(i);
    else
        v_des(i)=-sqrt(xp_des(i)^2+yp_des(i)^2)*ds(i);
    end
    w_des(i)=(ypp_des(i)*xp_des(i)-xpp_des(i)*yp_des(i))/(xp_des(i)^2+(yp_des(i)^2))*ds(i);
end

end