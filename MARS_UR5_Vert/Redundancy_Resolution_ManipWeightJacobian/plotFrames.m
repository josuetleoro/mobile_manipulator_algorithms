function plotFrame(T0,Tf,width)
Pos0=T0(1:3,4);
R0=T0(1:3,1:3);
Posf=Tf(1:3,4);
Rf=Tf(1:3,1:3);

%figure()

quiver3(Pos0(1),Pos0(2),Pos0(3),R0(1,1),R0(2,1),R0(3,1),'r','LineWidth',width,'MaxHeadSize',0.4,'AutoScale','off'); hold on;
quiver3(Pos0(1),Pos0(2),Pos0(3),R0(1,2),R0(2,2),R0(3,2),'g','LineWidth',width,'MaxHeadSize',0.4,'AutoScale','off');
quiver3(Pos0(1),Pos0(2),Pos0(3),R0(1,3),R0(2,3),R0(3,3),'b','LineWidth',width,'MaxHeadSize',0.4,'AutoScale','off'); 
text(Pos0(1),Pos0(2),Pos0(3),'    T0','HorizontalAlignment','left','FontSize',10);

quiver3(Posf(1),Posf(2),Posf(3),Rf(1,1),Rf(2,1),Rf(3,1),'r','LineWidth',width,'MaxHeadSize',0.4,'AutoScale','off');
quiver3(Posf(1),Posf(2),Posf(3),Rf(1,2),Rf(2,2),Rf(3,2),'g','LineWidth',width,'MaxHeadSize',0.4,'AutoScale','off');
quiver3(Posf(1),Posf(2),Posf(3),Rf(1,3),Rf(2,3),Rf(3,3),'b','LineWidth',width,'MaxHeadSize',0.4,'AutoScale','off'); 
text(Posf(1),Posf(2),Posf(3),'    Tf','HorizontalAlignment','left','FontSize',10);

xlabel('x')
ylabel('y')
zlabel('z')
view(-133,31)

end