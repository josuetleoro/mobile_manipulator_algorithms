% TestPoints
%% Choose the initial joint values, final poistion and times

% Transformation from euler angles to axes angle
% Rf2=eul2rotm([yaw pitch roll],'ZYX');
% rotm2axang(Rf2)
% pause()


switch(testN)
    case 1
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        %qa=[0.0;-80*pi/180;135*pi/180;-55*pi/180;-pi/2;0.0];
        qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
        
        %Desired Pose
        %Pos_f=[8;10;0.5];
        Pos_f=[2.5;3.5;0.4];
        quat_f=axang2quat([0 1 0 pi])';
        roll=-180*pi/180;
        pitch=0*pi/180;
        yaw=-180*pi/180;     
        tf=12;
        
    case 2
       %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
        
        qa=[0.0;-70*pi/180;130*pi/180;-150*pi/180;-pi/2;0.0];
        %qa=[0.0;-80*pi/180;135*pi/180;-55*pi/180;-pi/2;0.0];
        %qa=[0.0;-0.40;1.06;5*pi/4;-pi/2;0.0]; %Joint angles with maximum manipulability
        
        %Desired Pose
        %Pos_f=[-2.2;2.2;1.3];
        Pos_f=[-2.2;2.2;1.3];
        quat_f=axang2quat([0 -1 -1 pi])';
        %Passes through singularity
        roll=90*pi/180;
        pitch=0*pi/180;
        yaw=180*pi/180;
        tf=10;          
        
    case 3 
        % %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        qa=[0.0;-80*pi/180;135*pi/180;-55*pi/180;-pi/2;0.0];
        %qa=[0.0;-0.40;1.06;5*pi/4;-pi/2;0.0]; %Joint angles with maximum manipulability
        
        %Desired Pose
        Pos_f=[3.0;0.0;0.21];
        quat_f=axang2quat([0 1 0 pi/2])';
        roll=0*pi/180;
        pitch=90*pi/180;
        yaw=0*pi/180;        
        tf=8;      
end
