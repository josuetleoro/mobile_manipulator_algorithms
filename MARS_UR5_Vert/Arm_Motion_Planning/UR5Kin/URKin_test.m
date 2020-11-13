% test points FK
% P1
% q = [0;0;0;0;0;0];
% RES
% pos: -0.8173; -0.1915; -0.0055
% quat: 0.7071; 0.0; 0.0; 0.7071

% P2
% q = [-6.28;-2.30;2.35;3.34;2.84;-3.10];
% RES
% pos: -0.110; -0.031; 0.482
% quat: 0.029; 0.679; 0.711; 0.178

% P3
% q = [5.14;-0.79;1.54;-2.35;-1.63;-0.39]
% RES
% pos: -0.376; 0.581; 0.043
% quat: 0.918; 0.395; 0.031; -0.016

% P4
% q = [0.19;4.90;-1.34;2.96;3.90;-2.20];
% RES
% pos: 0.360; 0.020; 0.586
% quat: -0.511; 0.398; 0.082; 0.758

% P5
% q = [-2.876;3.1416;0;3.1416;-2.876; 0];  %Same as 0;0;0;0;0;0
% RES
% pos: -0.8173; -0.1915; -0.0055
% quat: 0.7071; 0.0; 0.0; 0.7071

% P6 (More precision)
q = [0.5818229;4.2901589;1.6468228;-1.3006193;-1.7442122; 0];
% RES
% pos: -0.195; -0.242; 0.536
% quat: 0.475; 0.875; 0.008; -0.094

robot = UR5Robot();

T0e = robot.forwardKin(q);
pos = T0e(1:3,4)'
quat = rotmat2quatROS(T0e(1:3,1:3))

% Calculate the inverse kinematics
sols = robot.inverseKin(T0e)

posD = T0e(1:3,4);
AdD = T0e(1:3,3);
OdD = T0e(1:3,2);
count = 0;
for i=1:size(sols, 1)
    th = sols(i, :);
    T0e_i = robot.forwardKin(th');
    
    pos_i = T0e_i(1:3,4);
    Ad_i = T0e_i(1:3,3);
    Od_i = T0e_i(1:3,2);
    errPos = posD - pos_i;
    errAd  = AdD  - Ad_i;
    errNd  = OdD  - Od_i;
    
    err = [posD/norm(posD) - pos_i/norm(pos_i); errAd; errNd];
    fprintf('Error %d: %.4f\n', i, err'*err)
    if err'*err < 1e-3
        count = count + 1;
    end

%     if err'*err < 1e-3
%         disp('ok!!')
%         disp(' ')
%         count = count + 1;
%     else
%         disp('NO!!!!')
%         disp(' ')
%     end
end

disp('Total sols:')
count






