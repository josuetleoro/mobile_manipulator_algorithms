function plotFrame(T,width,scale)
Pos=T(1:3,4);
R=T(1:3,1:3);
quiver3(Pos(1),Pos(2),Pos(3),R(1,1),R(2,1),R(3,1),scale,'r','LineWidth',width,'MaxHeadSize',0.6); hold on;
quiver3(Pos(1),Pos(2),Pos(3),R(1,2),R(2,2),R(3,2),scale,'g','LineWidth',width,'MaxHeadSize',0.6);
quiver3(Pos(1),Pos(2),Pos(3),R(1,3),R(2,3),R(3,3),scale,'b','LineWidth',width,'MaxHeadSize',0.6); 
end