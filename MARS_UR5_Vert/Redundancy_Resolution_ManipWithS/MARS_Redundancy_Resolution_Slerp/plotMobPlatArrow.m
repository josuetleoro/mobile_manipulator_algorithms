function plotMobPlatArrow(tx,ty,theta,width,name)
quiver3(tx,ty,0,cos(theta),sin(theta),0,'m','LineWidth',width,'MaxHeadSize',0.4,'AutoScale','off'); hold on;
text(tx,ty,0,['    ' name],'HorizontalAlignment','left','FontSize',10);
end