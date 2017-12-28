function drawRect(rectCx,rectCy,w,L,theta)

X = [-L/2 L/2 L/2 -L/2 -L/2];
Y = [w/2 w/2 -w/2 -w/2 w/2];

P = [X;Y];
ct = cos(theta);
st = sin(theta);
R = [ct -st;st ct];
P = R * P;

plot(P(1,:)+rectCx,P(2,:)+rectCy,'b-'); hold on

end