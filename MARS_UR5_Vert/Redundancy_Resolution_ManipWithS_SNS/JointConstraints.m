%% MARS joints constrains
q_limit=[-Inf, Inf;
         -Inf, Inf;
           0.0, 0.8;
          -pi/2, 5*pi/180;
          -22*pi/12, 22*pi/12;
          -22*pi/12, 22*pi/12;
          -22*pi/12, 22*pi/12;
          -22*pi/12, 22*pi/12;
          -22*pi/12, 22*pi/12];

dq_limit=[1.0;
          pi/2;
          0.1;
          pi;
          pi;
          pi;
          pi;
          pi;
          pi];
      
ddq_limit=[1.0;
           4.0; %4.0; 
           0.1;
           pi/6;
           pi/6;
           pi/6;
           pi/6;
           pi/6;
           pi/6];
