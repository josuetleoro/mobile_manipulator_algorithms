% % MARS joints constrains
q_limit=[-Inf, Inf;
         -Inf, Inf;
           0.0, 0.25;
          -100*pi/180, 1*pi/180;
          -90*pi/180, 15*pi/180;         %Needs review
          
          0*pi/180, 180*pi/180;    %Needs review
          %-pi/2, 150*pi/180;        %Needs review
          
          -2*pi, 2*pi;         
          -2*pi, 2*pi;
          -2*pi, 2*pi];

dq_limit=[0.6;
          pi/2;
          0.1;
          pi;
          pi;
          pi;
          pi;
          pi;
          pi]; 
      
ddq_limit=[2.0;
           4.0;
           0.1;
           pi/6;
           pi/6;
           pi/6;
           pi/6;
           pi/6;
           pi/6];