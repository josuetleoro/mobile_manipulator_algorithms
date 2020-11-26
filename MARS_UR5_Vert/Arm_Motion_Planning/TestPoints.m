% TestPoints
%% Choose the initial joint values, final poistion and times

% Transformation from euler angles to axes angle
% Rf2=eul2rotm([yaw pitch roll],'ZYX');
% rotm2axang(Rf2)
% pause()

switch(testN)
    case 1 
        % %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        qa = deg2rad([0;-80;110;-120;-90;0.0]);
        
        %Desired Pose
        Pos_f=[-0.6;0.05;0.5];
        quat_f=[0; 0.7071; 0.7071; 0];
        tf=10;

    case 2
        % %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        qa = deg2rad([0;-80;110;-120;-90;0.0]);
        
        %Desired Pose
        Pos_f=[-0.6;0.0;-0.3];
        quat_f=[0; 0.7071; 0.7071; 0];
        tf=10;
        
    case 3
        % %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        qa = deg2rad([0;-80;110;-120;-90;0.0]);
        
        %Desired Pose
        Pos_f=[-0.1;0.7;0.3];
        quat_f=axang2quat([1 0 0 -pi/2])';
        tf=10;
        
    case 4  % Painting task arm desired position before paint trajectory
        %Initial joints values
        tx=0.7790;
        ty=-1.3450;
        phi_mp=0;
        tz=0.2;
        qa=deg2rad([0;-80;110;-120;-90;0.0]);
       
        %Desired Pose
        Pos_f=[-0.2116;0.6539;0.1828];
        quat_f=[0.6102 -0.7920 -0.0160 -0.0160]';
        tf=10;      
end
