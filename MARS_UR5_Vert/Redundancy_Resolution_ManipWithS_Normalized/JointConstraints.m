% % MARS joints constrains
% q_limit=[-Inf, Inf;
%          -Inf, Inf;
%            -0.1, 0.4;
%            -0.1, 150*pi/180;
%           -22*pi/12, 22*pi/12;
%           -22*pi/12, 22*pi/12;
%           -22*pi/12, 22*pi/12;
%           -22*pi/12, 22*pi/12;
%           -22*pi/12, 22*pi/12];

q_limit=[-Inf, Inf;
         -Inf, Inf;
           -0.1, 0.4;
           -0.1, 150*pi/180;
          -pi, pi;         %Needs review
          -pi, pi;             %Needs review
          -2*pi, 2*pi;         
          -2*pi, 2*pi;
          -2*pi, 2*pi];

dq_limit=[1.0;
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
