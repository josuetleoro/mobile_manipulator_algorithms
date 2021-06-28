subplot(2,4,[1 2 5 6])
drawTruckAndTrailer2;
axis(drawLimits)

%Plot x
subplot(2,4,3)
plot(x);
title('x(m)')

%Plot phi
subplot(2,4,4)
plot(state(3,:));
title('phi(deg)')

%Plot eps
subplot(2,4,7)
plot(state(4,:));
title('eps(deg)')

%Plot theta
subplot(2,4,8)
plot(thetaGraph);
title('theta(deg)')

%axis([x(k)-15 x(k)+15 y(k)-15 y(k)+15])
drawnow