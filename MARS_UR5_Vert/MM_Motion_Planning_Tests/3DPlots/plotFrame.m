function plotFrame(T,width,name)
Pos=T(1:3,4);
R=T(1:3,1:3);
quiver3(Pos(1),Pos(2),Pos(3),R(1,1),R(2,1),R(3,1),'r','LineWidth',width,'MaxHeadSize',0.4,'AutoScale','off'); hold on;
quiver3(Pos(1),Pos(2),Pos(3),R(1,2),R(2,2),R(3,2),'g','LineWidth',width,'MaxHeadSize',0.4,'AutoScale','off');
quiver3(Pos(1),Pos(2),Pos(3),R(1,3),R(2,3),R(3,3),'b','LineWidth',width,'MaxHeadSize',0.4,'AutoScale','off'); 
text(Pos(1),Pos(2),Pos(3),['    ' name],'HorizontalAlignment','left','FontSize',10);
end