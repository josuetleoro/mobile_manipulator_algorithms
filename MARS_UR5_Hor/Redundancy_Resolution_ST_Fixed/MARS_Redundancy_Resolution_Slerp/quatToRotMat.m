function R=quatToRotMat(q)
% R=[1-2*(q(3)^2+q(4)^2),      2*(q(2)*q(3)+q(1)*q(4)),  2*(q(2)*q(4)-q(1)*q(3));
%    2*(q(2)*q(3)-q(1)*q(4)),  1-2*(q(2)^2+q(4)^2),      2*(q(3)*q(4)+q(1)*q(2));
%    2*(q(2)*q(4)+q(1)*q(3)),  2*(q(3)*q(4)-q(1)*q(2)),  1-2*(q(2)^2+q(3)^2)     ];


% %Animating Rotation with Quaternion Curves
% R=[1-2*(q(3)^2+q(4)^2),      2*(q(2)*q(3)+q(1)*q(4)),  2*(q(2)*q(4)-q(1)*q(3));
%    2*(q(2)*q(3)-q(1)*q(4)),  1-2*(q(2)^2+q(4)^2),      2*(q(3)*q(4)+q(1)*q(2));
%    2*(q(2)*q(4)+q(1)*q(3)),  2*(q(3)*q(4)-q(1)*q(2)),  1-2*(q(2)^2+q(3)^2)     ];


%Bruno Siciliano (Working but representation problem)
R=[2*(q(1)^2+q(2)^2)-1,      2*(q(2)*q(3)-q(1)*q(4)),  2*(q(2)*q(4)+q(1)*q(3));
   2*(q(2)*q(3)+q(1)*q(4)),  2*(q(1)^2+q(3)^2)-1,      2*(q(3)*q(4)-q(1)*q(2));
   2*(q(2)*q(4)-q(1)*q(3)),  2*(q(3)*q(4)+q(1)*q(2)),  2*(q(1)^2+q(4)^2)-1   ];

end