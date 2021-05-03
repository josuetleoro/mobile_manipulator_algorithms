classdef UR5Robot
    %Class for the kinematics of UR5 robot arm.
    properties
        %lenght of the links
        l1@double = double(0.089159);
        l2@double = double(0.425);
        l3@double = double(0.39225);
        l4@double = double(0.10915);
        l5@double = double(0.09465);
        l6@double = double(0.0823);
%         l1@double = double(0.0892);
%         l2@double = double(0.425);
%         l3@double = double(0.392);
%         l4@double = double(0.1093);
%         l5@double = double(0.09475);
%         l6@double = double(0.0825);
        %Directions and Points of each joint
        % directions
        d1
        d2
        d3
        d4
        d5
        d6
        d7
        % points
        p1
        p2
        p3
        p4
        p5
        p6
        p7
        l_end;
        
        %Moments of the axes
        m1
        m2
        m3
        m4
        m5
        m6
        m7
        
        %Vectors of new subproblem to find theta1
        Pw0
        omega3
        y
        y_norm
        b_new_qeq
        b_2_qeq        
        params={}
        
        %Dual Quaternion rotation operator for each joint
        dq1@DualQuat
        dq2@DualQuat
        dq3@DualQuat
        dq4@DualQuat
        dq5@DualQuat
        dq6@DualQuat       
        
    end
    
    methods
        %Constructor
        function obj=UR5Robot()
            % directions
            obj.d1 = [0;0;1];
            obj.d2 = [0;-1;0];
            obj.d3 = [0;-1;0];
            obj.d4 = [0;-1;0];
            obj.d5 = [0;0;-1];
            obj.d6 = [0;-1;0];
            obj.d7 = [0;0;1];
            % points
            obj.p1 = [0;0;obj.l1];
            obj.p2 = obj.p1;
            obj.p3 = obj.p2 + [-obj.l2;0;0];
            obj.p4 = obj.p3 + [-obj.l3;0;0];
            obj.p5 = obj.p4 + [0;-obj.l4;0];
            obj.p6 = obj.p5 + [0;-obj.l6;-obj.l5];
            obj.p7 = obj.p6 + 0.5*obj.d7;
            obj.l_end = obj.l6;
            %Moments of the axes
            obj.m1 = cross(obj.p1, obj.d1);
            obj.m2 = cross(obj.p2, obj.d2);
            obj.m3 = cross(obj.p3, obj.d3);
            obj.m4 = cross(obj.p4, obj.d4);
            obj.m5 = cross(obj.p5, obj.d5);
            obj.m6 = cross(obj.p6, obj.d6);
            obj.m7 = cross(obj.p7, obj.d7);
            %Vectors of new subproblem to find theta1
            obj.Pw0 = obj.p6 - obj.d6*obj.l_end;  
            obj.omega3 = cross(obj.d1, obj.d2);
            % Here we assume that p1 only has component on the z direction
            obj.y = obj.Pw0 - obj.p1;
            obj.b_new_qeq = dot(obj.y, obj.omega3);
            obj.b_2_qeq = obj.b_new_qeq * obj.b_new_qeq;
            obj.y_norm = dot(obj.y, obj.y);
            obj.params{1}=obj.omega3;
            obj.params{2}=obj.y;
            obj.params{3}=obj.b_new_qeq;
            obj.params{4}=obj.b_2_qeq;
            obj.params{5}=obj.y_norm;
        end
        
        function T0e=forwardKin(this,q)
            %First check the required number of joints is provided
            if size(q,1)~=6
                error('The forward kinematics require the 6DOF values')
            end
            %Create the dual quaternions for each joint
            this.dq1 = DualQuat.RigidTransf(q(1),this.d1,this.m1);
            this.dq2 = DualQuat.RigidTransf(q(2),this.d2,this.m2);
            this.dq3 = DualQuat.RigidTransf(q(3),this.d3,this.m3);
            this.dq4 = DualQuat.RigidTransf(q(4),this.d4,this.m4);
            this.dq5 = DualQuat.RigidTransf(q(5),this.d5,this.m5);
            this.dq6 = DualQuat.RigidTransf(q(6),this.d6,this.m6);
            
            %% Rigid transformation
            Q=this.dq1*this.dq2*this.dq3*this.dq4*this.dq5*this.dq6;
            
            % Using point and vector rotation
            pos = rotPointDualQuat(this.p6, Q); 
            approach = rotVectorDualQuat(this.d6, Q);
            orientation = rotVectorDualQuat(this.d7, Q);         

            %Create the transformation matrix
            T0e=zeros(4,4);
            T0e(1:3,1)=cross(orientation, approach);
            T0e(1:3,2)=orientation;
            T0e(1:3,3)=approach;
            T0e(1:3,4)=pos;
            T0e(4,4)=1;
        end
        
        function sols=inverseKin(this, T)
            %First check the transformation has the required size
            if size(T,1)~=4 || size(T,2)~=4
                error('The inverse kinematics requires a 4x4 transformation matrix as input')
            end
            %% Extract the desired position, approaching direction and normal direction
            Ped = T(1:3,4);
            Ad = T(1:3,3);
            Od = T(1:3,2);
            ZeroTh=0.0001;
            
            %The number of possible solutions is 8. We create a matrix to store the
            %solutions, where each column is the value of the angle and the 7th row
            %tells us if it is a valid solution or not, one for valid and zero for not
            %valid
            theta=zeros(8,7);
            theta(1:8,7)=1;  %Start all the solutions with one

            %% Step 1: Find theta 1 using the geometry of the robot
            tic
            r = zeros(3,2);
            p1qV = zeros(3,2);
            th1 = zeros(2,1);
            % Calculate the position of the wrist      
            Pwd = Ped - this.l_end * Ad;
            qV = dot((Pwd - this.Pw0),this.d1)*this.d1;
            q = Pwd - qV - this.p1;
            s = sqrt(this.b_2 - (this.c_qeq-dot(q,q))); 
            r(:,1) = this.p - (this.b_new_qeq + s)*this.omega3;
            r(:,2) = this.p - (this.b_new_qeq - s)*this.omega3;
            p1qV(:,1) = r(:,1) + this.p1 + qV;
            p1qV(:,2) = r(:,2) + this.p1 + qV;
            % Use subproblem 1
            th1(1) = subproblem1(this.d1, r(:,1), q);
            th1(2) = subproblem1(this.d1, r(:,2), q);
            
            % TODO: Check the solutions th1 are valid
            %Copy the solutions to the solution matrix
            theta(1:4,1)=th1(1);
            theta(5:8,1)=th1(2);
            
            %% Step 2: Find theta 5
            %We will Rotate L5 for the next step
            P5_1 = zeros(3,4);
            th5 = zeros(2,1);
            Q1(1:2) = DualQuat;
            for i=1:2
                Q1(i) = DualQuat.RigidTransf(-1*th1(i),this.d1,this.m1);
                % Rotate the approaching diraction -theta 1
                Ad_1 = rotVectorDualQuat(Ad, Q1(i));
                z5_aux=cross(this.d4,Ad_1);
                
                if abs(norm(z5_aux)<ZeroTh) %check if th5 is zero
                    %asign th5 zero
                    th5(1)=0.0;
                    th5(2)=0.0;
                    
                    %Make z5_1 parallel to Od rotated back to the referece frame(i.e., force th6=0)
                    z5_1(:,2*i-1)=rotVectorDualQuat(Od, Q1(i));
                    z5_1(:,2*i)=-1*z5_1(:,2*i-1);
                    
                    P5_1(:,2*i-1)=p1qV(:,i)-z5_1(:,2*i-1)*this.l5; %Sign is negative becase z5 in original position points down
                    P5_1(:,2*i)=p1qV(:,i)-z5_1(:,2*i)*this.l5;
                else
                    %Normalize z5
                    z5_aux=z5_aux/norm(z5_aux);
                    
                    %Get the position of p5 for wrist up and wrist down
                    z5_1(:,2*i-1)=z5_aux;
                    z5_1(:,2*i)=-z5_aux;
                    
                    P5_1(:,2*i-1)=p1qV(:,i)-z5_1(:,2*i-1)*this.l5; %Sign is negative becase z5 in original position points down
                    P5_1(:,2*i)=p1qV(:,i)-z5_1(:,2*i)*this.l5;
                    
                    %Caculate th5 using subproblem1
                    th5(1)=subproblem1(z5_1(:,2*i-1), this.d6, Ad_1);
                    th5(2)=subproblem1(z5_1(:,2*i), this.d6, Ad_1);
                end
                
                % TODO: Check the solutions th5 are valid
                index = 4 * i - 3;
                %Copy the solutions to the solution matrix
                theta(index:index+1,5)=th5(1);
                theta(index+2:index+3,5)=th5(2);              
            end
                         
            %% Step 3: Find theta 2 and theta 3 using the position of the wrist
            th2 = zeros(2, 1);
            th3 = zeros(2, 1);
            for i=1:4
                % Use subproblem2pa with the position of the wrist
                [th2(1),th2(2),th3(1),th3(2),sol]=subproblem2pa(this.d2, this.p5, P5_1(:,i), this.p3, this.p2);
                
                index = i * 2 - 1;
                if sol == 0
                    % Eliminate the invalid solutions
                    theta(index, 7) = 0;
                    theta(index + 1, 7) = 0;
                elseif sol == 1
                    % Copy the valid solution
                    theta(index, 2) = th2(1);
                    theta(index, 3) = th3(1);
                    % Eliminate the other one
                elseif sol == 2
                    % Copy both solutions
                    theta(index, 2) = th2(1);
                    theta(index, 3) = th3(1);
                    theta(index + 1, 2) = th2(2);
                    theta(index + 1, 3) = th3(2);
                end                
            end
            %% Step 4: Find theta 4 and theta 6
            k=1; % Index for valid solutions
            for i=1:8
                if (theta(i,7) ~= 0)
                    % Create the transformation operator
                    Q2 = DualQuat.RigidTransf(-1*theta(i, 2), this.d2, this.m2);
                    Q3 = DualQuat.RigidTransf(-1*theta(i, 3), this.d3, this.m3);
                    index =ceil(i/4);
                    Q = Q3 * Q2 * Q1(index);
                    % Rotate pwd back using th1, th2 and th3 
                    Pwd_123 = rotPointDualQuat(Pwd, Q);
                    
                    % Calculate theta 4 with subproblem 1 
                    th4 = subproblem1(this.d4, this.Pw0, Pwd_123, this.p4);
                    theta(i, 4) = th4;
                    
                    % Add th4 and th5 to the transformation operator
                    Q4 = DualQuat.RigidTransf(-1*theta(i, 4), this.d4, this.m4);
                    Q5 = DualQuat.RigidTransf(-1*theta(i, 5), this.d5, this.m5);
                    Q = Q5 * Q4 * Q;
                    
                    % Find a point close to p6 in the direction of Nd after
                    % rotating all the joints back.
                    Od_12345 = rotVectorDualQuat(Od, Q);
                    Paux = this.p6 + 5 * Od_12345;
                    
                    % Calculate theta 6 with subproblem1
                    th6 = subproblem1(this.d6, this.p7, Paux, this.p6);                    
                    theta(i, 6) = th6;
                    
                    % Store the valid solution
                    sols(k,:) = theta(i,1:6);
                    k = k + 1;
                end                
            end
            if size(sols,1) < 1
                error('Could not find a valid IK solution')
            end
            toc
        end
        
        function sols=inverseKinQuat(this, T)
            %First check the transformation has the required size
            if size(T,1)~=4 || size(T,2)~=4
                error('The inverse kinematics requires a 4x4 transformation matrix as input')
            end
            %% Extract the desired position, approaching direction and normal direction
            Ped = T(1:3,4);
            Ad = T(1:3,3);
            Od = T(1:3,2);
            ZeroTh=0.0001;
            
            %The number of possible solutions is 8. We create a matrix to store the
            %solutions, where each column is the value of the angle and the 7th row
            %tells us if it is a valid solution or not, one for valid and zero for not
            %valid
            theta=zeros(8,7);
            theta(1:8,7)=1;  %Start all the solutions with one
            
            %% Step 1: Find theta 1 using the new subproblem3
            tic
            %Calculate the Position of the wrist
            Pwd=Ped-this.l_end*Ad;
            %Use the new sub problem to find theta 1 and p1qV
            [th1(1),th1(2),c,sol]=subproblem3_new(this.d1,this.Pw0,Pwd,this.p1,this.params);
            % TODO: Check the solutions th1 are valid
            %Copy the solutions to the solution matrix
            theta(1:4,1)=th1(1);
            theta(5:8,1)=th1(2);
            
            %% Step 2: Find theta 5
            %We will Rotate L5 for the next step
            z5_1=zeros(3,4);
            P5_1=zeros(3,4);
            th5 = zeros(2,1);
            Q1(1:2) = Quat;
            for i=1:2
                %Rotation operator of theta 1
                Q1(i) = Quat.rotQuat(-1*th1(i),this.d1);
                
                %Rotate the approaching direction to the reference position
                Ad_1=rotVectorQuat(Ad,Q1(i));
                z5_aux=cross(this.d4,Ad_1); 
                
                if abs(norm(z5_aux)<ZeroTh) %check if th5 is zero (Singular configuration)
                    %asign th5 zero
                    th5(1)=0.0;
                    th5(2)=0.0;
        
                    %Make z5_1 parallel to Od rotated back to the referece frame(i.e., force th6=0)
                    z5_1(:,2*i-1)=rotVectorQuat(Od,Q1(i));
                    z5_1(:,2*i)=-1*z5_1(:,2*i-1);
        
                    P5_1(:,2*i-1)=c(:,i)-z5_1(:,2*i-1)*this.l5; %Sign is negative becase z5 in original position points down
                    P5_1(:,2*i)=c(:,i)-z5_1(:,2*i)*this.l5;
                else
                    %Normalize z5
                    z5_aux=z5_aux/norm(z5_aux);
        
                    %Get the position of p5 for wrist up and wrist down
                    z5_1(:,2*i-1)=z5_aux;
                    z5_1(:,2*i)=-z5_aux;        
                    P5_1(:,2*i-1)=c(:,i)-z5_1(:,2*i-1)*this.l5; %Sign is negative becase z5 in original position points down
                    P5_1(:,2*i)=c(:,i)-z5_1(:,2*i)*this.l5;
        
                    %Caculate th5 using subproblem1
                    th5(1)=subproblem1(z5_1(:,2*i-1), this.d6, Ad_1);
                    th5(2)=subproblem1(z5_1(:,2*i), this.d6, Ad_1);
                end
                
                % TODO: Check the solutions th5 are valid
                index=4 * i - 3;
                %Copy the solutions to the solution matrix
                theta(index:index+1,5)=th5(1);
                theta(index+2:index+3,5)=th5(2);            
            end
                        
            %% Step 3: Find theta 2 and theta 3 using the position of the wrist and subproblem 2pa
            th2 = zeros(2, 1);
            th3 = zeros(2, 1);
            for i=1:4
                % Use subproblem2pa with the position of the wrist
                [th2(1),th2(2),th3(1),th3(2),sol]=subproblem2pa(this.d2, this.p5, P5_1(:,i), this.p3, this.p2);
                
                index = 2 * i - 1;
                %Check if the solution exists
                if sol==0
                    theta(index:index+1,7)=0;
                elseif sol==1
                    %Copy one solution
                    theta(index,2)=th2(1);
                    theta(index,3)=th3(1);
                    %Eliminate the other one
                    theta(index+1,7)=0;
                elseif sol==2
                    %Copy the solutions to the solution matrix
                    theta(index,2)=th2(1);
                    theta(index,3)=th3(1);
                    theta(index+1,2)=th2(2);
                    theta(index+1,3)=th3(2);
                end
            end
            
            %% Step 4: Find theta 4 and theta 6
            k=1; % Index for valid solutions
            for i=1:8
                if (theta(i,7) ~= 0)
                    %Rotation operators for theta 2 and theta 3
                    Q2 = Quat.rotQuat(-1*theta(i, 2), this.d2);
                    Q3 = Quat.rotQuat(-1*theta(i, 3), this.d3);
                    Q32 = Q3*Q2;
                    
                    % Get the direction of z5 that has already been rotated
                    % by -th1
                    index5=ceil(i/2);
                    z5_aux=z5_1(:,index5);
                    
                    %Rotate z5_1 backward  using th1, th2, th3
                    d5_23=rotVectorQuat(z5_aux,Q32);        
                    th4=subproblem1(this.d4,this.d5,d5_23);
                    theta(i,4)=th4;
                    
                    %Find theta 6
                    index1=ceil(i/4);
                    %add th1, q32, th4 and th5 to the rotation operator
                    Q12345=Quat.rotQuat(-theta(i,5),this.d5)*Quat.rotQuat(-theta(i,4),this.d4)*Q32*Q1(index1);
                    
                    %Rotate the desired orientation direction back to the reference
                    %frame using Q12345
                    Od_12345=rotVectorQuat(Od,Q12345);
    
                    %Use Subproblem 1
                    th6=subproblem1(this.d6, this.d7, Od_12345);
                    theta(i,6)=th6;
                    
                    % Store the valid solution
                    sols(k,:) = theta(i,1:6);
                    k = k + 1;
                end
            end
            if k < 2
                theta
                disp('Error with the following transformation')
                T
                error('Could not find a valid IK solution')
            end
            toc
        end
        
        function q=closestIK(this, T, q0)
            %First check the transformation has the required size
            if size(T,1)~=4 || size(T,2)~=4
                error('The inverse kinematics requires a 4x4 transformation matrix as input')
            end
            % Also check the given joint positions has the required size
            if size(q0,1)~=6
                error('The number of robot joints is 6')
            end
            
            % Convert the angles between -180 and 180
            for i=1:6
                if q0(i) > 0
                    q0(i) = mod(q0(i), pi);
                else
                    q0(i) = mod(q0(i), -pi);
                end
            end
            
            % Find all the IK solutions
            sols = this.inverseKinQuat(T);                     
            
            min_dist = Inf;
            min_idx = 1;
            for i=1:size(sols, 1)
                % Calculate the error
                angle_error = q0 - sols(i,:)';
                % Convert between -pi and pi
                angle_error = mod(angle_error + pi, 2*pi) - pi;
                % Sum the errors
                sol_dist = sum(abs(angle_error));
                                
                if sol_dist < min_dist
                   min_dist =  sol_dist;
                   min_idx = i;
                end
            end
            q = sols(min_idx,:)';
        end
    end
end
