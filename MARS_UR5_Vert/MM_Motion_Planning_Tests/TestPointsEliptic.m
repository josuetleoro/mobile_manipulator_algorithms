% TestPoints
%% Choose the initial joint values, final poistion and times

% Transformation from euler angles to axes angle
% Rf2=eul2rotm([yaw pitch roll],'ZYX');
% rotm2axang(Rf2)
% pause()


switch(testN)
    case 1 
        % %Initial joints values
        tx=-1;
        ty=0.75;
        phi_mp=-pi;
        tz=0.1;
        qa=[0;-80;110;-120;-90;0]*pi/180;
        
        %Desired Pose
        Pos_f=[1.5;-1.3;1.0];
        quat_f=[0;0.7071;-0.7071;0];
        tf=25;
        
    case 2 
        % %Initial joints values
        tx=-1.2997;
        ty=0.5585;
        phi_mp=-0.0108;
        tz=0.24;
        qa=[0;-80;110;-120;-90;0]*pi/180;
        
        %Desired Pose
        Pos_f=[1.55;-1.0;0.26];
        quat_f=[0.2706;0.6533;0.6533;-0.2706];  
        tf=20;        

end
