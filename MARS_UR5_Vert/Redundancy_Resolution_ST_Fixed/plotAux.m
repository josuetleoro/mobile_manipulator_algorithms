time=MotPlan.time(1:k-1);

subplot(3,1,1)
plot(time,xi(3,1:k-1))
subplot(3,1,2)
plot(time,q(4,1:k-1))
subplot(3,1,3)
plot(time,eta(4,1:k-1))