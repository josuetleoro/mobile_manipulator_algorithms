function drawMobilePlatform(x,y,phi,endEff_x,endEff_y)
    %Platform dimensions
    wid=0.6;
    len=0.7;
    
    %Wheel dimensions
    wheelWid=0.1;
    wheelLen=0.4;
    
    wheelOffy=wid/2+wheelWid;
    wheelOffx=len/2-wheelLen/2;
    
    %Draw the body of the mobile manipulator
    drawRect(x,y,wid,len,phi);
    
    %Draw the wheels
    %Right wheel
    Rwheelx=wheelOffx*cos(phi)-wheelOffy*sin(phi)+x;
    Rwheely=wheelOffx*sin(phi)+wheelOffy*cos(phi)+y;    
    drawRect(Rwheelx,Rwheely,wheelWid,wheelLen,phi);
    
    %Left wheel
    Lwheelx=wheelOffx*cos(phi)+wheelOffy*sin(phi)+x;
    Lwheely=wheelOffx*sin(phi)-wheelOffy*cos(phi)+y;    
    drawRect(Lwheelx,Lwheely,wheelWid,wheelLen,phi);
    
    %End Effector Position    
    plot(endEff_x,endEff_y,'r.','MarkerSize',20); %The end effector
    plot([x endEff_x],[y endEff_y]); % A line that representes the UR5

end