function trans = manipQuinticTrans(N,tf,blend_perc,ts)
tb1 = tf*blend_perc;
tb2 = tf*(1-blend_perc);

% Manually calculate the coefficients
a1 = 10/(tb1^3);
a2 = -15/(tb1^4);
a3 = 6/(tb1^5);

trans = zeros(1,N);
t=0;
k=1;
while k <= N
    if (t < tb1)
       trans(k)=a1*t^3+a2*t^4+a3*t^5;
    end
    
    if (t >= tb1 && t <= tb2)
       trans(k)=1;
    end
    
    if (t > tb2)
       trans(k)=1-a1*(t-tb2)^3-a2*(t-tb2)^4-a3*(t-tb2)^5;
    end
    k=k+1;
    t=t+ts;    
end
trans(end) = 0;
% size(trans)
% time = 0:ts:tf;
% size(time)
% plot(time, trans)
% pause()
end