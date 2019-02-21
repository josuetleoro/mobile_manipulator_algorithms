% TestPoints
%% Choose the initial joint values, final poistion and times

% Transformation from euler angles to axes angle
% Rf2=eul2rotm([yaw pitch roll],'ZYX');
% rotm2axang(Rf2)
% pause()

switch(testN)
    case 1 %Point in the front and down
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.12;
        qa=[0.0;-80*pi/180;110*pi/180;-120*pi/180;-pi/2;0.0];        
        
        %Desired Pose
        Pos_f=[2.0;2.1092;0.5];
        %quat_f=axang2quat([0 1 0 pi/2])';
        quat_f=[0.5;-0.5;0.5;-0.5];
        roll=-180*pi/180;
        pitch=0*pi/180;
        yaw=-180*pi/180;     
        tf=12;
        
    case 2 %Point in the back and up
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.12;
        qa=[0.0;-80*pi/180;110*pi/180;-120*pi/180;-pi/2;0.0];        
        
       %Desired Pose
        %Pos_f=[-2.2;2.2;1.3];
        Pos_f=[-1.1;1.1;1.3];
        quat_f=axang2quat([0 -1 -1 pi])';
        %Passes through singularity
        roll=90*pi/180;
        pitch=0*pi/180;
        yaw=180*pi/180;
        tf=12;
    case 3 %From ROS
        %Initial joints values
%         tx=0.8124;
%         ty=1.1148;
%         phi_mp=-0.0421;
%         tz=0.1165;
%         qa=[-0.0000;-1.3964;1.9198;-2.0938;-1.5708;-0.0001];
        %Get initial state from ROS
        GetMarsCurrState        
        
        %Desired Pose
        Pos_f=[1.8;1.0;1.2];
        quat_f=eul2quat([-pi/2 0 -pi/2],'ZYX')';
        tf=15;
end
