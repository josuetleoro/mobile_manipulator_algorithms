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
        qa=[0.0;-0.40;1.06;5*pi/4;-pi/2;0.0]; %Joint angles with maximum manipulability
        
        %Desired Pose
        Pos_f=[3;-0.1091;0.18];
        quat_f=axang2quat([0 1 0 pi])';
        roll=0*pi/180;
        pitch=180*pi/180;
        yaw=0*pi/180;       
        tf=12;          %Desired final time
        
    case 2 
        % %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        qa=[0.0;-0.40;1.06;5*pi/4;-pi/2;0.0]; %Joint angles with maximum manipulability
        
        %Desired Pose
        Pos_f=[3;0.1091;0.23];
        quat_f=axang2quat([0 1 0 pi/2])';
        roll=0*pi/180;
        pitch=90*pi/180;
        yaw=0*pi/180;        
        tf=18;        
        
    case 3 
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
       
        %Desired Pose
        Pos_f=[2;3;1.4];
        quat_f=axang2quat([0 1 0 pi])';
        roll=-180*pi/180;
        pitch=0*pi/180;
        yaw=-180*pi/180;
        tf=25;        
        
    case 4 
        % %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
        
        %Desired Pose
        Pos_f=[1.0;-0.8;0.3];
        quat_f=axang2quat([1 1 -1 120*pi/180])';
        roll=90*pi/180;
        pitch=90*pi/180;
        yaw=0*pi/180;
        tf=12;        
        
    case 5 
        %Initial joints values
        tx=0;
        ty=0.1;
        phi_mp=0;
        tz=0.2;
        qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
        
        %Desired Pose
        Pos_f=[-1.2;-1.2;0.15];
        quat_f=axang2quat([1 0 0 pi])';
        roll=180*pi/180;
        pitch=0*pi/180;
        yaw=0*pi/180;
        tf=15;       
        
    case 6 
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        %qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
        qa=[0.0;-0.40;1.06;5*pi/4;-pi/2;0.0]; %Joint angles with maximum manipulability
        
        %Desired Pose
        Pos_f=[-2;2;0.3];
        quat_f=axang2quat([0 -1 -1 pi])';
        %Passes through singularity
        roll=90*pi/180;
        pitch=0*pi/180;
        yaw=180*pi/180;
        tf=18;      
        
    case 7 
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        qa=[0.0;-0.40;1.06;5*pi/4;-pi/2;0.0]; %Joint angles with maximum manipulability
        
        %Desired Pose
        quat_f=axang2quat([0 1 0 pi/2])';
        Pos_f=[1;1;0.6];
        roll=0*pi/180;
        pitch=90*pi/180;
        yaw=0*pi/180;
        tf=6;     
        
    case 8 
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
        
        %Desired Pose
        Pos_f=[1.2;1.2;0.7];
        quat_f=axang2quat([-1 1 1 120*pi/180])';
        roll=180*pi/180;
        pitch=90*pi/180;
        yaw=-90*pi/180;
        tf=8;       
        
    case 9
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.05;
        qa=[0.0;-0.40;1.06;5*pi/4;-pi/2;0.0]; %Joint angles with maximum manipulability
        
        %Desired Pose
        Pos_f=[0.5;-3;0.1];
        quat_f=axang2quat([-1 1 1 120*pi/180])';
        roll=-180*pi/180;
        pitch=90*pi/180; %-90*pi/180 cannot be executed with high lambda
        yaw=-90*pi/180;
        tf=15;     
        
     case 10
        % %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        qa=[0.0;-0.40;1.06;5*pi/4;-pi/2;0.0]; %Joint angles with maximum manipulability
        
        %Desired Pose
        Pos_f=[-3;0.1091;0.8];
        quat_f=axang2quat([1 1 -1 120*pi/180])';
        roll=90*pi/180;
        pitch=90*pi/180;
        yaw=0*pi/180;      
%         %Passes through singularity
%         quat_d=axang2quat([0 0 1 pi]);
%         roll=0*pi/180;
%         pitch=0*pi/180;
%         yaw=180*pi/180;        
        tf=25;     
        
    case 11
        % %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
        
        %Desired Pose
        Pos_f=[2;1;0.4];
        quat_f=axang2quat([0.9351 0.2506 -0.2506 3.0075])';
        roll=180*pi/180;
        pitch=30*pi/180;
        yaw=30*pi/180;
        tf=12;         
        
    case 12 
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        qa=[0.0;-pi/2;pi/2;-pi/2;-pi/2;0.0];
        
        %Desired Pose
        Pos_f=[8;10;0.5];
        quat_f=axang2quat([0 1 0 pi])';
        roll=-180*pi/180;
        pitch=0*pi/180;
        yaw=-180*pi/180;
        
        Rf2=eul2rotm([yaw pitch roll],'ZYX');
        rotm2axang(Rf2)        
        
        tf=40;           
        
    case 13 
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        qa=[0.0;-0.40;1.06;5*pi/4;-pi/2;0.0]; %Joint angles with maximum manipulability
       
        %Desired Pose
        Pos_f=[2;3;0.15];
        quat_f=axang2quat([0 1 0 pi])';
        roll=-180*pi/180;
        pitch=0*pi/180;
        yaw=-180*pi/180;
        tf=13;     
        
    case 14 
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.05;
        qa=[0.0;-0.40;1.06;5*pi/4;-pi/2;0.0]; %Joint angles with maximum manipulability
    
        %Desired Pose
        Pos_f=[-0.3;0;0.04];
        quat_f=axang2quat([0 1 0 pi])';
        roll=0*pi/180;
        pitch=180*pi/180;
        yaw=0*pi/180;   
        tf=10;
        
     case 15
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=pi/2;
        tz=0.05;
        %qa=[-pi/2;-0.40;1.06;5*pi/4;-pi/2;0.0];
        qa=[-pi/2;-pi/4;pi/2;3*pi/4;-pi/2;0.0];
        
        %Desired Pose
        Pos_f=[0.7221;3;0.7246];
        
        %Dificult case, where the final position is far and the final
        %orientation puts the joint 5 in a singular position
%         Pos_f=[0.7221;8;0.7246];
%         tf=30;
        
%         quat_f=axang2quat([-1 1 1 120*pi/180])';
%         roll=-90*pi/180;
%         pitch=90*pi/180;
%         yaw=0*pi/180;
        %Orientation with a problem of mobile platform rotation        
        quat_f=axang2quat([-1 1 -1 120*pi/180])';
        roll=-90*pi/180;
        pitch=0*pi/180;
        yaw=-90*pi/180;       
        
        tf=15;        
end
