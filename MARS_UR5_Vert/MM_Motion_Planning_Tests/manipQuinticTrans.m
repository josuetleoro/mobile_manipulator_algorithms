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
time = 0:ts:tf;
% %Plot transition profile
% blue=[0    0.4470    0.7410];
% labelFontSize=14;
% lineWidth=1.8;
% set(0,'defaulttextinterpreter','latex')
% set(0,'defaulttextfontname', 'Times')
% set(0,'defaulttextfontsize',16)
% set(0, 'defaultAxesTickLabelInterpreter','latex');
% set(0, 'defaultLegendInterpreter','latex');
% set(0,'defaultaxesfontsize',16)
% set(0,'defaultaxesfontname', 'Times')
% plot(time, trans,'LineWidth',lineWidth,'Color',blue)
% grid on
% xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize);
% ylabel('$\beta$','interpreter','latex','FontSize',labelFontSize);
% set(gcf, 'Position',  [200, 500, 490, 310])
% pause()
end