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
        tf=14;
        
    case 2 
        % %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        qa=[0.0;-0.40;1.06;5*pi/4;-pi/2;0.0]; %Joint angles with maximum manipulability
        
        %Desired Pose
        Pos_f=[3.0;0.0;0.32];
        quat_f=axang2quat([0 1 0 pi/2])';
        roll=0*pi/180;
        pitch=90*pi/180;
        yaw=0*pi/180;        
        tf=15;        
        
    case 3 
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
       
        %Desired Pose
        Pos_f=[2;3;1.3];
        quat_f=axang2quat([0 1 0 pi])';
        roll=-180*pi/180;
        pitch=0*pi/180;
        yaw=-180*pi/180;
        tf=22;        
        
    case 4 % Problem with initial configuration: qa=[0.0;-80*pi/180;135*pi/180;-90*pi/180;-pi/2;0.0];
        % %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
        
        %Desired Pose
        Pos_f=[1.0;-0.8;0.4];
        quat_f=axang2quat([1 1 -1 120*pi/180])';
        roll=90*pi/180;
        pitch=90*pi/180;
        yaw=0*pi/180;
        tf=18;        
        
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
        %Pos_f=[-2.2;2.2;1.3];
        Pos_f=[-2.2;2.2;1.5];
        quat_f=axang2quat([0 -1 -1 pi])';
        %Passes through singularity
        roll=90*pi/180;
        pitch=0*pi/180;
        yaw=180*pi/180;
        tf=23;      
        
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
        Pos_f=[0.5;-3;0.15];
        quat_f=axang2quat([-1 1 1 120*pi/180])';
        roll=-180*pi/180;
        pitch=90*pi/180; %-90*pi/180 cannot be executed with high lambda
        yaw=-90*pi/180;
        tf=20;     
        
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
        tf=18;     
        
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
        tf=14;         
        
    case 12 
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        qa=[0.0;-pi/2;pi/2;-pi/2;-pi/2;0.0];
        
        %Desired Pose
        %Pos_f=[8;10;0.5];
        Pos_f=[5.5;6.5;1.2];
        quat_f=axang2quat([0 1 0 pi])';
        roll=-180*pi/180;
        pitch=0*pi/180;
        yaw=-180*pi/180;
        
        Rf2=eul2rotm([yaw pitch roll],'ZYX');
        rotm2axang(Rf2)        
        
        tf=25;       
        
    case 13 
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        qa=[0.0;-0.40;1.06;5*pi/4;-pi/2;0.0]; %Joint angles with maximum manipulability
       
        %Desired Pose
        Pos_f=[2;3;0.15];  %Lower limit that can be solved
        quat_f=axang2quat([0 1 0 pi])';
        roll=-180*pi/180;
        pitch=0*pi/180;
        yaw=-180*pi/180;
        tf=20;     
        
    case 14 
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.05;
        qa=[0.0;-0.40;1.06;5*pi/4;-pi/2;0.0]; %Joint angles with maximum manipulability
    
        %Desired Pose
        Pos_f=[-0.3;0;0.13];
        quat_f=axang2quat([0 1 0 pi])';
        roll=0*pi/180;
        pitch=180*pi/180;
        yaw=0*pi/180;   
        tf=8;
        
     case 15
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=pi/2;
        tz=0.05;
        qa=[-pi/2;-pi/4;pi/2;3*pi/4;-pi/2;0.0];
        
        %Desired Pose       
        %Dificult case, where the final position is far and the final
        %orientation puts the joint 5 in a singular position
        Pos_f=[0.7221;8;0.7246];

        quat_f=axang2quat([-1 1 1 120*pi/180])';
        roll=-90*pi/180;
        pitch=90*pi/180;
        yaw=0*pi/180;
        %Orientation with a problem of mobile platform rotation        
        quat_f=axang2quat([-1 1 -1 120*pi/180])';
        
        tf=14;
    
    case 16 %Case1 paper
        %Initial joints values
        tx=-1.2;
        ty=0.6;
        phi_mp=pi;
        tz=0.1;
        %qa=[-pi/2;-0.40;1.06;5*pi/4;-pi/2;0.0];
        qa=[0;-80;110;-120;-90;0]*pi/180;
        
        %Desired Pose
        Pos_f=[1.6;-1.3;1.2];
        quat_f=[0;1;0;0];
        
        tf=30;
    
    case 17 %Case2 paper
        %Initial joints values
        qa=[0;-80;110;-120;-90;0]*pi/180;
        
        tx=-1.4;
        ty=0.6;
        phi_mp=0;
        tz=0.24;
%         tx=-1.64;
%         ty=-0.35;
%         phi_mp=0;
%         tz=0.24;
        %Desired Pose
%         Pos_f=[1.75;-0.4;0.26];
%         quat_f=[0.342;0;0.939;0];        
%         tf=20;
       
        %Desired Pose
        Pos_f=[1.6;-0.6;0.25];
        quat_f=[0.2706;0.6533;0.6533;-0.2706];
        
        tf=20;
        
end