clear all

L = 4.0;
h = 0.6;
tf = 10;
Nwaves = 2;
ts = 1 / 50;
lambda = L / Nwaves;
vel = L / tf;
omega = 2*pi*vel/lambda;

k=1;
for i=0:ts:L
    t(k)=i;
    x(k)=h*sin(2*pi/lambda*i);
    k = k+1;
end

plot(t,x)
