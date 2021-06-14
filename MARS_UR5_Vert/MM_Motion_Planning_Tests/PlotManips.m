close all

%Plots properties
colors = distinguishableColors(6);
blue=colors(1,:);
red=colors(2,:);
green=colors(3,:);
black=colors(4,:);
pink=colors(5,:);
yellow=colors(6,:);
labelFontSize=14;
lineWidth=1.4;
limLineWidth = 1.8;

set(0,'defaulttextinterpreter','latex')
set(0,'defaulttextfontname', 'Times')
set(0,'defaulttextfontsize',16)

set(0, 'defaultAxesTickLabelInterpreter','latex');
set(0, 'defaultLegendInterpreter','latex');
set(0,'defaultaxesfontsize',16)
set(0,'defaultaxesfontname', 'Times')

plots_end_time = tf;
spc = floor(N/15);
markerIdx = 8:spc:(N-1);
markerSize = 8;

%% Manipulability plots
figure()
% MM_man_max = max(MM_man_measure);
% ur5_man_max = max(ur5_man_measure);
% MM_man_measure = MM_man_measure / MM_man_max;
% ur5_man_measure = ur5_man_measure / ur5_man_max;
% W_measure_max = max(W_measure);
% W_measure = W_measure / W_measure_max;

markerIdxError = 20:20:(length(time)-1);
h1=plot(time,MM_man_measure,'LineWidth',1.0,'Color',blue,'Marker','o','MarkerIndices',markerIdx,'MarkerSize',markerSize); hold on
h2=plot(time,ur5_man_measure,'LineWidth',1.0,'Color',red,'Marker','d','MarkerIndices',markerIdx,'MarkerSize',markerSize);
h3=plot(time,W_measure,'LineWidth',1.0,'Color',green,'Marker','square','MarkerIndices',markerIdx,'MarkerSize',markerSize); hold off
uistack(h3,'top')
uistack(h2,'top')
uistack(h1,'top')
legend([h1 h2 h3],'$\Omega_{p+a}$','$\Omega_{a}$','$\Omega_{obj}$','interpreter','latex','FontSize',labelFontSize)
xlim([0 plots_end_time])
xlabel('$t(s)$','interpreter','latex','FontSize',labelFontSize)
ylim([0 1])
ylabel('$(m)$','interpreter','latex','FontSize',labelFontSize)
grid on
set(gcf, 'Position',  [200, 500, 490, 310])

title('Manipulability measure')

