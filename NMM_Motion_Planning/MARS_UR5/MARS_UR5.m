classdef MARS_UR5
    %Class for the mobile manipulator with the UR5 robot arm and a
    %prismatic joing in the mobile plaftorm.
    properties
        %lenght of the links
        l1@double = double(0.089159);
        l2@double = double(0.425);
        l3@double = double(0.39225);
        l4@double = double(0.10915);
        l5@double = double(0.09465);
        l6@double = double(0.0823);        
        %Position of the base of the UR5 robot with respect to the center of the wheels
        a = 0.011; 
        b = 0.0762 + 0.48296; %(wheels radius + center of wheels to prism joint)
        %Direction and Points of each joint
        p1
        p2
        p3
        p4
        p5
        p6
        p7
        p8
        d1=[0;0;1];  
        d2=[0;1;0]; 
        d3=[0;1;0]; 
        d4=[0;1;0]; 
        d5=[0;0;-1]; 
        d6=[0;1;0]; 
        d7
        d8
        %Moments of the axes
        m1
        m2
        m3
        m4
        m5
        m6
        m7
        m8
        %Dual Quaternion rotation operator for each joint
        dq1@DualQuat
        dq2@DualQuat
        dq3@DualQuat
        dq4@DualQuat
        dq5@DualQuat
        dq6@DualQuat
        %Quaternion representation of the mobile platform
        dqmp@DualQuat
        %Dual quaternion representation of the axis that represent the tool
        dqL6@DualQuat   %Approach
        dqL7@DualQuat   %Orientation
        dqL8@DualQuat   %Auxiliar for inverse kinematics    
    end
    
    methods
        %Constructor
        function obj=MARS_UR5()
            %Direction and Points of each joint
            obj.p1=[obj.a;0;obj.b+obj.l1];
            obj.p2=[obj.a;0;obj.b+obj.l1];
            obj.p3=[obj.a+obj.l2;0;obj.b+obj.l1];
            obj.p4=[obj.a+obj.l2+obj.l3;0;obj.b+obj.l1];
            obj.p5=[obj.a+obj.l2+obj.l3;obj.l4;obj.b+obj.l1];
            obj.p6=[obj.a+obj.l2+obj.l3;obj.l4+obj.l6;obj.b+obj.l1-obj.l5];
            
            %Auxiliar line for tool position and Orientation Direction
            obj.d7=[0;0;1]; obj.p7=obj.p6;
            %Second auxiliar axis (Not used at the moment)
            obj.d8=obj.d6; obj.p8=obj.p6+obj.d7*obj.l6;
            
            %Get the axes in the plucker coordinates
            obj.m1=cross(obj.p1,obj.d1); %Joint 1
            obj.m2=cross(obj.p2,obj.d2); %Joint 2
            obj.m3=cross(obj.p3,obj.d3); %Joint 3
            obj.m4=cross(obj.p4,obj.d4); %Joint 4
            obj.m5=cross(obj.p5,obj.d5); %Joint 5
            obj.m6=cross(obj.p6,obj.d6); %Joint 6
            %Get the auxiliar axis moments
            obj.m7=cross(obj.p7,obj.d7);  %Aux 1 for tool position and normal direction
            obj.m8=cross(obj.p8,obj.d8);  %Aux 2 for tool position and approaching direction
        end
        function T0e=forwardKin(this,q)
            %First check the required
            if size(q,1)~=10
                error('The forward kinematics require the 10DOF values')
                return
            end
            %Create the dual quaternions for each joint
            %% Mobile Platform and 7th degree of freedom
            t=[q(1) q(2) q(4)]; %tx ty tz
            %Translation and rotation quaternions
            qrot=Quat.rotQuat(q(3),[0 0 1]); %Rotation of an angle phi about the z axis
            qt=Quat(0,t);
            %Composite operator
            this.dqmp=DualQuat(qrot,1/2*qt*qrot);                        
            
            %% UR5
            %Get the transformation operator of each axis
            this.dq1=DualQuat(Quat(cos(q(5)/2),sin(q(5)/2)*this.d1),Quat(0,sin(q(5)/2)*this.m1));
            this.dq2=DualQuat(Quat(cos(q(6)/2),sin(q(6)/2)*this.d2),Quat(0,sin(q(6)/2)*this.m2));
            this.dq3=DualQuat(Quat(cos(q(7)/2),sin(q(7)/2)*this.d3),Quat(0,sin(q(7)/2)*this.m3));
            this.dq4=DualQuat(Quat(cos(q(8)/2),sin(q(8)/2)*this.d4),Quat(0,sin(q(8)/2)*this.m4));
            this.dq5=DualQuat(Quat(cos(q(9)/2),sin(q(9)/2)*this.d5),Quat(0,sin(q(9)/2)*this.m5));
            this.dq6=DualQuat(Quat(cos(q(10)/2),sin(q(10)/2)*this.d6),Quat(0,sin(q(10)/2)*this.m6));            
            
            %% Rigid transformation
            Q=this.dqmp*this.dq1*this.dq2*this.dq3*this.dq4*this.dq5*this.dq6;
            Qc=Q.conj;
            
            %Dual quaternion representation of the axis that represent the tool
            qL6=DualQuat(Quat(0,this.d6),Quat(0,this.m6));
            qL7=DualQuat(Quat(0,this.d7),Quat(0,this.m7));
            
            %Get the new positions and orientations of L6 and L7
            newQL6=Q*qL6*Qc;
            newQL7=Q*qL7*Qc;
            
            %Get the intersections of L6 and L7
            pos=LinePlucker.lineInt(newQL6,newQL7);
            approach=newQL6.prim.getV;
            orientation=newQL7.prim.getV;
            normal=cross(orientation,approach);           
            
            %Create the transformation matrix
            T0e=zeros(4,4);
            T0e(1:3,1)=normal;
            T0e(1:3,2)=orientation;
            T0e(1:3,3)=approach;
            T0e(1:3,4)=pos;
            T0e(4,4)=1;
        end
    end
end

















