% % MARS joints constrains
q_limit=[-Inf, Inf;
         -Inf, Inf;
           0.0, 0.4;
          -100*pi/180, 0.01;
          -pi/2, 25*pi/180;         %Needs review
          -pi/2, 150*pi/180;        %Needs review
          -2*pi, 2*pi;         
          -2*pi, 2*pi;
          -2*pi, 2*pi];

dq_limit=[0.8;
          pi/2;
          0.2;
          pi;
          pi;
          pi;
          pi;
          pi;
          pi];      

% dq_limit=[4.0;
%           2*pi;
%           1.0;
%           2*pi;
%           2*pi;
%           2*pi;
%           2*pi;
%           2*pi;
%           2*pi];  
      
ddq_limit=[2.0;
           4.0;
           0.1;
           pi/6;
           pi/6;
           pi/6;
           pi/6;
           pi/6;
           pi/6];
