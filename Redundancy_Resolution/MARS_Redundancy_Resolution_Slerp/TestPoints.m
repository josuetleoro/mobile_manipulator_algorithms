% TestPoints
%% Choose the initial joint values, final poistion and times

switch(testN)
    case 1 %working
        % %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0;
        qa=[0.0; -pi; 3*pi/4; -pi/4; pi/2; 0.0]; %(The starting position is very important for the algorithm to work)
        
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
        qa=[0.0; -pi; 3*pi/4; -pi/4; pi/2; 0.0]; %(The starting position is very important for the algorithm to work)
        
        %Desired Pose
        Pos=[3;0.1091;0.8];
        roll=0*pi/180;
        pitch=90*pi/180;
        yaw=0*pi/180;        
        tf=15;          %Desired final time
        ts=0.05;        %time step
        tb=5;           %Blending time
        lambda = 1;     %error weight 
        
    case 3 %working        
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0;
        qa=[0.0; -pi; pi/2; -pi/2; -pi/2; pi];
        
        %Desired Pose
        Pos=[2;3;0.5];
        roll=-180*pi/180;
        pitch=0*pi/180;
        yaw=-180*pi/180;
        tf=20;          %Desired final time
        ts=0.05;        %time step
        tb=5;           %Blending time
        lambda = 1;     %error weight
    case 4 %working
        % %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0;
        qa=[0.0; -pi; 3*pi/4; -pi/4; pi/2; 0.0]; %(The starting position is very important for the algorithm to work)
        
        %Desired Pose
        Pos=[3;-2.0;0.8];
        roll=90*pi/180;
        pitch=0*pi/180;
        yaw=90*pi/180;
        tf=20;          %Desired final time
        ts=0.05;        %time step
        tb=5;           %Blending time
        lambda = 1;     %error weight
    case 5 %Working
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0;
        qa=[0.0; -pi; 3*pi/4; -pi/4; pi/2; 0.0]; %(The starting position is very important for the algorithm to work)
        
        %Desired Pose
        Pos=[-1.5;-1.5;0.6];
        roll=180*pi/180;
        pitch=0*pi/180;
        yaw=0*pi/180;
        tf=35;          %Desired final time
        ts=0.05;        %time step
        tb=5;           %Blending time
        lambda = 1;     %error weight
    case 6 %Working
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0;
        qa=[0.0; -pi; 3*pi/4; -pi/4; pi/2; 0.0]; %(The starting position is very important for the algorithm to work)
        
        %Desired Pose
        Pos=[-2;2;0.6];
        roll=90*pi/180;
        pitch=0*pi/180;
        yaw=180*pi/180;
        tf=35;          %Desired final time
        ts=0.05;        %time step
        tb=5;           %Blending time
        lambda = 1;     %error weight
    case 7 %Singularity
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0;
        qa=[0.0; -pi; pi/2; -pi/2; -pi/2; -pi/2]; %(The starting position is very important for the algorithm to work)
        
        %Desired Pose
        Pos=[2;2;0.6];
        roll=0*pi/180;
        pitch=90*pi/180;
        yaw=0*pi/180;
        tf=20;          %Desired final time
        ts=0.05;        %time step
        tb=5;           %Blending time
        lambda = 1;     %error weight
    case 8 %Orientation problem because the rotation is done from the other side
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0;
        qa=[0.0; -pi; pi/2; -pi/2; -pi/2; pi]; %(The starting position is very important for the algorithm to work)
        
        %Desired Pose
        Pos=[2;2;0.7];
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
        qa=[0.0; -pi; 3*pi/4; -pi/4; pi/2; 0.0];
        
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
        qa=[0.0; -pi; 3*pi/4; -pi/4; pi/2; 0.0]; %(The starting position is very important for the algorithm to work)
        
        %Desired Pose
        Pos=[-3;0.1091;0.8];
        roll=90*pi/180;
        pitch=90*pi/180;
        yaw=0*pi/180;        
        tf=15;          %Desired final time
        ts=0.05;        %time step
        tb=5;           %Blending time
        lambda = 1;     %error weight
end
