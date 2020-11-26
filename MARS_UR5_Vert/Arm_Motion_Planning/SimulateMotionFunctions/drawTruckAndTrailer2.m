%Find the position of the center of the trailer
trailerCx=x(k)+trailerL/2*cos(phi(k));
trailerCy=y(k)+trailerL/2*sin(phi(k));
drawRect(trailerCx,trailerCy,trialerW,trailerL,phi(k));
%plot(x,y,'r.','MarkerSize',20); 

%Find the link coordiantes
linkX(1)=x(k)+trailerL*cos(phi(k));
linkY(1)=y(k)+trailerL*sin(phi(k));
linkX(2)=linkX(1)+linkL*cos(phi(k)+eps(k));
linkY(2)=linkY(1)+linkL*sin(phi(k)+eps(k));
plot(linkX,linkY,'b-');
%plot(linkX(1),linkY(1),'r.','MarkerSize',20);
%plot(linkX(2),linkY(2),'r.','MarkerSize',20); 

%Find the position of the center of the truck
truckCx=linkX(2)+truckL/2*cos(phi(k)+eps(k));
truckCy=linkY(2)+truckL/2*sin(phi(k)+eps(k));
drawRect(truckCx,truckCy,truckW,truckL,phi(k)+eps(k));
hold off






