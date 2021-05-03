clear all
limits=[0.0, 0.25];
q = limits(1):0.001:limits(2);
N = length(q);
gradH = zeros(1,N);
labelFontSize=14;
lineWidth=1.8;

blue = [0,0,1];

set(0,'defaulttextinterpreter','latex')
set(0,'defaulttextfontname', 'Times')
set(0,'defaulttextfontsize',16)

set(0, 'defaultAxesTickLabelInterpreter','latex');
set(0, 'defaultLegendInterpreter','latex');
set(0,'defaultaxesfontsize',16)
set(0,'defaultaxesfontname', 'Times')

for i =1:N
    gradH(i) = abs((limits(2)-limits(1))^2*(2*q(i)-limits(2)-limits(1))/(4*(limits(2)-q(i))^2*(q(i)-limits(1))^2));
end

plot(q,gradH,'LineWidth',lineWidth,'Color',blue); hold on;
xlabel('$q$','interpreter','latex','FontSize',labelFontSize)
% ylabel('$|\dfrac{\partial{H(q)}}{\partial{q}}$','interpreter','latex','FontSize',labelFontSize)
ylabel('$|{\partial H(q)}/{\partial q}|$','interpreter','latex','FontSize',labelFontSize)
grid on
set(gcf, 'Position',  [200, 500, 490, 310])