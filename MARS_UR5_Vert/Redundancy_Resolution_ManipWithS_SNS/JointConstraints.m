%% MARS joints constrains
q_limit=[-Inf, Inf;
         -Inf, Inf;
           0.0, 0.8;
          -2*pi, 2*pi;
          -22*pi/12, 22*pi/12;
          -22*pi/12, 22*pi/12;
          -22*pi/12, 22*pi/12;
          -22*pi/12, 22*pi/12;
          -22*pi/12, 22*pi/12];

dq_limit=[0.8;
          pi/3;
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
