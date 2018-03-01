% TestPoints
%% Choose the initial joint values, final poistion and times

switch(testN)
    case 1 %working
        % %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0;
        qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0]; 
        
        %Desired Pose
        Pos=[3;0.1091;0.8];
        roll=90*pi/180;
        pitch=90*pi/180;
        yaw=0*pi/180;        
        tf=15;          %Desired final time
        ts=0.05;        %time step
        tb=5;           %Blending time
        lambda = 1;     %error weight
        
    case 2 %working
        % %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0;
        qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0]; 
        
        %Desired Pose
        Pos=[3;0.1091;0.8];
        roll=0*pi/180;
        pitch=90*pi/180;
        yaw=0*pi/180;        
        tf=15;          %Desired final time
        ts=0.05;        %time step
        tb=5;           %Blending time
        lambda = 1;     %error weight 
        
    case 3 % not working        
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0;
        qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
        
        %Desired Pose
        Pos=[0.5;0.8;0.5];
%         roll=-180*pi/180;
%         pitch=0*pi/180;
%         yaw=-180*pi/180;
        roll=90*pi/180;
        pitch=90*pi/180;
        yaw=0*pi/180;
        tf=20;          %Desired final time
        ts=0.05;        %time step
        tb=5;           %Blending time
        lambda = 1;     %error weight
    case 4 % Not working (Check the reason here)
        % %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0;
        %qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
        qa=[0.0;-pi/2;pi/2;-pi/2;-pi/2;0.0];
        
        %Desired Pose
        Pos=[3;-2.0;0.8];
        roll=90*pi/180;
        pitch=0*pi/180;
        yaw=90*pi/180;
%         roll=90*pi/180;
%         pitch=90*pi/180;
%         yaw=0*pi/180;

        tf=20;          %Desired final time
        ts=0.05;        %time step
        tb=5;           %Blending time
        lambda = 1;     %error weight
    case 5 %Not working
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0;
        qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
        
        %Desired Pose
        Pos=[-1.5;-1.5;0.6];
        roll=180*pi/180;
        pitch=0*pi/180;
        yaw=0*pi/180;
        tf=35;          %Desired final time
        ts=0.05;        %time step
        tb=5;           %Blending time
        lambda = 1;     %error weight
    case 6 %Not Working
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0;
        qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
        
        %Desired Pose
        Pos=[-2;2;0.6];
        roll=90*pi/180;
        pitch=0*pi/180;
        yaw=180*pi/180;
        tf=35;          %Desired final time
        ts=0.05;        %time step
        tb=5;           %Blending time
        lambda = 1;     %error weight
    case 7 %Not working
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0;
        qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
        
        %Desired Pose
        Pos=[2;2;0.6];
        roll=0*pi/180;
        pitch=90*pi/180;
        yaw=0*pi/180;
        tf=20;          %Desired final time
        ts=0.05;        %time step
        tb=5;           %Blending time
        lambda = 1;     %error weight
    case 8 %Not working
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0;
        qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
        
        %Desired Pose
        Pos=[3.0;3.0;0.7];
        roll=180*pi/180;
        pitch=90*pi/180;
        yaw=-90*pi/180;
        tf=15;          %Desired final time
        ts=0.05;        %time step
        tb=5;           %Blending time
        lambda = 1;     %error weight   
    case 9
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0;
        qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
        
        %Desired Pose
        Pos=[0.5;-3;0.5];
        roll=-180*pi/180;
        pitch=-90*pi/180;
        yaw=-90*pi/180;
        tf=20;          %Desired final time
        ts=0.05;         %time step
        tb=5;           %Blending time
        lambda = 1;    %error weight
     case 10
        % %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0;
        qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
        
        %Desired Pose
        Pos=[-3;0.1091;0.8];
        roll=90*pi/180;
        pitch=90*pi/180;
        yaw=0*pi/180;        
        tf=15;          %Desired final time
        ts=0.05;        %time step
        tb=5;           %Blending time
        lambda = 1;     %error weight
    case 11
        % %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0;
        qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
        
        %Desired Pose
        Pos=[2;1;0.4];
        roll=180*pi/180;
        pitch=30*pi/180;
        yaw=30*pi/180;
        tf=15;          %Desired final time
        ts=0.05;        %time step
        tb=5;           %Blending time
        lambda = 1;     %error weight
end
