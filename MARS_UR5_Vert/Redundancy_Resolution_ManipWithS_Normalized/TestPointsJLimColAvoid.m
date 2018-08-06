% TestPoints
%% Choose the initial joint values, final poistion and times

switch(testN)
    case 1 
        % %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0]; 
        
        %Desired Pose
        Pos=[3;-0.1091;0.12];
        roll=0*pi/180;
        pitch=180*pi/180;
        yaw=0*pi/180;       
        tf=12;          %Desired final time
        ts=0.05;        %time step
        tb=3;           %Blending time
        
    case 2 
        % %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        %qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0]; 
        qa=[0.0;-0.40;1.06;5*pi/4;-pi/2;0.0]; %Joint angles with maximum manipulability
        
        %Desired Pose
        Pos=[3;0.1091;0.22];
        roll=0*pi/180;
        pitch=90*pi/180;
        yaw=0*pi/180;        
        tf=18;          
        ts=0.05;        
        tb=3;           
        
    case 3 
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
        %qa=[0.0;-0.40;1.06;5*pi/4;-pi/2;0.0]; %Joint angles with maximum manipulability
       
        %Desired Pose
        Pos=[2;3;1.4];
        roll=-180*pi/180;
        pitch=0*pi/180;
        yaw=-180*pi/180;
        tf=12;          
        ts=0.05;        
        tb=3;           
        
    case 4 
        % %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
        %qa=[0.0;-pi/2;pi/2;-pi/2;-pi/2;0.0];
        
        %Desired Pose
        Pos=[1.0;-0.8;0.5];
        roll=90*pi/180;
        pitch=90*pi/180;
        yaw=0*pi/180;
        tf=12;          
        ts=0.05;        
        tb=3;           
        
    case 5 
        %Initial joints values
        tx=0;
        ty=0.1;
        phi_mp=0;
        tz=0.2;
        qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
        
        %Desired Pose
        Pos=[-1.2;-1.2;0.6];
        roll=180*pi/180;
        pitch=0*pi/180;
        yaw=0*pi/180;
        tf=12;          
        ts=0.05;        
        tb=3;           
        
    case 6 
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        %qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
        qa=[0.0;-0.40;1.06;5*pi/4;-pi/2;0.0]; %Joint angles with maximum manipulability
        
        %Desired Pose
        Pos=[-2;2;0.3];
        %Passes through singularity
        roll=90*pi/180;
        pitch=0*pi/180;
        yaw=180*pi/180;
        tf=18;          
        ts=0.05;        
        tb=3;          
        
    case 7 
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        %qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
        qa=[0.0;-0.40;1.06;5*pi/4;-pi/2;0.0]; %Joint angles with maximum manipulability
        
        %Desired Pose
        Pos=[1;1;0.6];
        roll=0*pi/180;
        pitch=90*pi/180;
        yaw=0*pi/180;
        tf=6;          
        ts=0.05;        
        tb=2;     
        
    case 8 
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
        
        %Desired Pose
        Pos=[1.2;1.2;0.7];
        roll=180*pi/180;
        pitch=90*pi/180;
        yaw=-90*pi/180;
        tf=8;          
        ts=0.05;        
        tb=2;           
        
    case 9
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.05;
        %qa=[0.0;-pi/2;3*pi/4;pi/4;-pi/2;0.0];
        qa=[0.0;-0.40;1.06;5*pi/4;-pi/2;0.0]; %Joint angles with maximum manipulability
        
        %Desired Pose
        Pos=[0.5;-3;0.10];
        roll=-180*pi/180;
        pitch=90*pi/180; %-90*pi/180 cannot be executed with high lambda
        yaw=-90*pi/180;
        tf=10;          
        ts=0.05;         
        tb=2;        
        
     case 10
        % %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
        
        %Desired Pose
        Pos=[-3;0.1091;0.8];
        %Passes through singularity        
        roll=90*pi/180;
        pitch=90*pi/180;
        yaw=0*pi/180;        
        
%         roll=0*pi/180;
%         pitch=0*pi/180;
%         yaw=180*pi/180;        
        tf=25;          
        ts=0.05;        
        tb=5;           
        
    case 11
        % %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        qa=[0.0;-pi/2;3*pi/4;5*pi/4;-pi/2;0.0];
        %qa=[0.0;-0.40;1.06;5*pi/4;-pi/2;0.0]; %Joint angles with maximum manipulability
        
        %Desired Pose
        Pos=[2;1;0.4];
        roll=180*pi/180;
        pitch=30*pi/180;
        yaw=30*pi/180;
        tf=12;          
        ts=0.05;        
        tb=5;           
        
    case 12 
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        qa=[0.0;-pi/2;pi/2;-pi/2;-pi/2;0.0];
        
        %Desired Pose
        Pos=[8;10;0.5];
        roll=-180*pi/180;
        pitch=0*pi/180;
        yaw=-180*pi/180;
        tf=30;          %Desired final time (tf=40 has problems)
        ts=0.05;        
        tb=5;           
        
    case 13 
        %Initial joints values
        tx=0;
        ty=0;
        phi_mp=0;
        tz=0.2;
        qa=[0.0;-0.40;1.06;5*pi/4;-pi/2;0.0]; %Joint angles with maximum manipulability
       
        %Desired Pose
        Pos=[2;3;1.4];
        roll=-180*pi/180;
        pitch=0*pi/180;
        yaw=-180*pi/180;
        tf=12;          
        ts=0.05;        
        tb=3;           
end
