function tri = genTrianglePoints(x, y, theta, scale)
% x, y: center of the triangle
% theta: angle with respect to the x axis
% scale: scaling from original triangle

%% Original triangle located at pos
p1 = [0.8;0];
p2 = [-0.5;0.5];
p3 = [-0.5;-0.5];

%% Rotate about z axis and scale the triangle
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
p1 = scale * R * p1 + [x; y];
p2 = scale * R * p2 + [x; y];
p3 = scale * R * p3 + [x; y];

%% Plot
tri = [p1(1) p2(1) p3(1) p1(1); p1(2) p2(2) p3(2) p1(2)];


end