classdef Quat
    %Class for quaternions
    properties
        s %scalar (real part)
        v %vector (imaginary part)
    end
    methods
        %Constructor
        function obj=Quat(varargin)
            switch nargin
                case 0
                    %No argument - set default quaternion
                    obj.s=0;
                    obj.v=[0;0;0];
                case 1
                    %1 argument - representation as a vector
                    if size(varargin{1},2)==4
                        obj.s=varargin{1}(1);
                        obj.v=varargin{1}(2:4)';
                    else
                        error('The quaterion object must have four elements')
                    end
                case 2
                    %2 arguments - escalar and vector
                    obj.s=varargin{1};
                    if size(varargin{2},1)==3                        
                        obj.v=varargin{2};
                    elseif size(varargin{2},2)==3
                        obj.v=varargin{2}';
                    else
                        error('The vector must have three elements')
                    end
                case 4
                    %4 arguments - each component separately
                    obj.s=varargin{1};
                    obj.v(1,1)=varargin{2};
                    obj.v(2,1)=varargin{3};
                    obj.v(3,1)=varargin{4};
                otherwise
                    error('Wrong number of input arguments')
            end
        end
        
        %Quaternion display
        function disp(obj)
            %Usage: disp(q)
            %Purpose: display a quaternion number object
            %Input: q -- quaternion number object
            %Output: Display the quaternion number
            fprintf('%.4f',obj.s)
            if(obj.v(1)>=0.0)
                fprintf('+%.4fi',obj.v(1))
            else
                fprintf('%.4fi',obj.v(1))
            end
            if(obj.v(2)>=0.0)
                fprintf('+%.4fj',obj.v(2))
            else
                fprintf('%.4fj',obj.v(2))
            end
            if(obj.v(3)>=0.0)
                fprintf('+%.4fk\n',obj.v(3))
            else
                fprintf('%.4fk\n',obj.v(3))
            end
        end
        
        %Get s
        function QuatS=getS(this)
            QuatS=this.s;
        end
        %Get v
        function QuatV=getV(this)
            QuatV=this.v;
        end
        
        %Quaternion addition
        function obj=plus(q1,q2)
            obj=Quat(q1.s+q2.s,q1.v+q2.v);
        end
        
        %Quaternion difference
        function obj=minus(q1,q2)
            obj=Quat(q1.s-q2.s,q1.v-q2.v);
        end
        
        %Quaternion multiplication
        function obj=mtimes(q1,q2)
            if isa(q1,'double') %If the first argument is a scalar
                s=q1*q2.s;
                v=q1*q2.v;
                obj=Quat(s,v);  %Create the quaternion object using s and v
                return
            end
            if isa(q2,'double') %If the second argument is a scalar
                s=q2*q1.s;
                v=q2*q1.v;
                obj=Quat(s,v);  %Create the quaternion object using s and v
                return
            end
            if isa(q1,'Quat')&&isa(q2,'Quat') %If both arguments are quaternions
                s=q1.s*q2.s-dot(q1.v,q2.v);
                v=q1.s*q2.v+q2.s*q1.v+cross(q1.v,q2.v);
                obj=Quat(s,v);  %Create the quaternion object using s and v
                return
            end
        end
        
        %Quaternion division
        function obj=mrdivide(q,scalar)
            if isa(scalar,'double')
                s=q.s/scalar;
                v=q.v/scalar;
                obj=Quat(s,v);  %Create the quaternion object using s and v
            else
                error('The second argument must be a scalar')
            end            
        end
        
        %Quaternion conjugate
        function obj=conj(this)
            s=this.s;
            v=-1*this.v;
            obj=Quat(s,v);
        end
        
        %Quaternion norm
        function n=norm(this)
            q=this*this.conj();
            n=sqrt(q.s*q.s+dot(q.v,q.v));
        end
        
        %Quaternion inverse
        function obj=inv(this)
            obj=this.conj()/this.norm();
        end        
        
        %Representation as a vector of 4 elements
        function obj=vecRep(this)
            obj=[this.s; this.v];
        end        
        
        
    end  %End methods
    
    methods(Static)        
        %Rotation quaternion
        function obj=rotQuat(theta,axis)
            obj=Quat(cos(theta/2),sin(theta/2)*axis);
        end
        
        %Logarithm of a quaternion        
        function obj=log(q)
            if isa(q,'Quat') %If the argument is a Quat object
                %Get the scalar and the vector
                quatS=q.getS();
                quatV=q.getV();
                
                %Calculate the logarithm
                [s,v]=Quat.calculateLog([quatS;quatV]);
                obj=Quat(s,v);  %Create the quaternion object using s and v
                return
            end
            if isa(q,'double') %If the argument is a vector
                if length(q)~=4
                    error('the input quaternion does not have four elements')
                end
                
                %Calculate the logarithm
                [s,v]=Quat.calculateLog(q);
                obj=[s;v];  %Create the quaternion using s and v
                return
            end            
        end
        function [s,v]=calculateLog(q)
            quatS=q(1);
            quatV=q(2:4);
            v_norm=norm(quatV);
            q_norm=norm(q);
            
            theta=atan2(norm(quatV),quatS);
            
            s=log(q_norm);
            %Check if the norm of v is close to zero
            if v_norm > 0.0001
                v=theta/v_norm*quatV;
            else
                %Calculate the factor theta/sin(theta) using Taylor series
                factor=1+theta^2/6+7*theta^4/360+31*theta^6/15120;
                v=factor/q_norm*quatV;                
            end
        end
        
        %exponential of a quaternion 
        function obj=exp(q)
            if isa(q,'Quat') %If the argument is a Quat object
                %Get the vector part of the quaternion
                quatV=q.getV();
                
                %Calculate the exponential
                [s,v]=Quat.calculateExp(quatV);
                obj=Quat(s,v);  %Create the quaternion object using s and v
                return
            end
            if isa(q,'double') %If the argument is a vector
                if length(q)~=4
                    error('the input quaternion does not have four elements')
                end
                
                %Calculate the exponential
                [s,v]=Quat.calculateExp(q(2:4));
                obj=[s;v];  %Create the quaternion using s and v
                return
            end            
        end
        function [s,v]=calculateExp(quatV)
            theta=norm(quatV); %The v norm transforms in theta for the exponential
            s=cos(theta);
            %Check if the norm of v is close to zero
            if theta > 0.0001
                v=sin(theta)/theta*quatV;
            else
                %Calculate the factor sin(theta)/theta using Taylor series
                factor=1-theta^2/6+theta^4/120-theta^6/5040;
                v=factor*quatV;                
            end
        end 
        
        %Power of a quaternion 
        function obj=pow(q,n)
            if isa(q,'Quat') %If the argument is a Quat object
                quatS=q.getS();
                %Get the vector part of the quaternion
                quatV=q.getV();
                
                %Calculate the Power
                [s,v]=Quat.calculatePow([quatS;quatV],n);
                obj=Quat(s,v);  %Create the quaternion object using s and v
                return
            end
            if isa(q,'double') %If the argument is a vector
                if length(q)~=4
                    error('the input quaternion does not have four elements')
                end
                
                %Calculate the exponential
                [s,v]=Quat.calculatePow(q,n);
                obj=[s;v];  %Create the quaternion using s and v
                return
            end
        end
        function [s,v]=calculatePow(quat,n)
            [quat_log_s,quat_log_v]=Quat.calculateLog(quat);
            quat_log=[quat_log_s;quat_log_v];
            n_quat_log=n*quat_log(2:4);
            [s,v]=Quat.calculateExp(n_quat_log);
        end
    end %End static methods
    
end



























