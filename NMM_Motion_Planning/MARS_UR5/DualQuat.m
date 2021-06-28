classdef DualQuat
   %Class for dual quaternions
   properties
       prim %primary part
       dual %dual part
   end
   methods
       %Constructor
       function obj=DualQuat(varargin)
           switch nargin
               case 0
                   %No argument - set default dual quaternion
                   obj.prim=Quat();
                   obj.dual=Quat();
               case 2
                   %2 arguments - two arguments (two quaternions)
                   if isa(varargin{1},'Quat')&&isa(varargin{2},'Quat')
                       obj.prim=varargin{1};
                       obj.dual=varargin{2};
                   else
                       error('The two arguments must be quaternions')
                   end
               case 8
                   %8 arguments - each component separately
                   obj.prim=Quat(varargin{1},varargin{2},varargin{3},varargin{4});
                   obj.dual=Quat(varargin{5},varargin{6},varargin{7},varargin{8});
               otherwise
                   error('Wrong number of input arguments')
           end
       end
       
       %Dual quaternion display
       function disp(obj)
           %Usage: disp(dualQ)
           %Purpose: display a quaternion number object
           %Input: dualQ -- dual quaternion number object
           %Output: Display the quaternion number
           
           %Print primary quaternion
           fprintf('%.4f',obj.prim.s)
           if(obj.prim.v(1)>=0.0)
               fprintf('+%.4fi',obj.prim.v(1))
           else
               fprintf('%.4fi',obj.prim.v(1))
           end
           if(obj.prim.v(2)>=0.0)
               fprintf('+%.4fj',obj.prim.v(2))
           else
               fprintf('%.4fj',obj.prim.v(2))
           end
           if(obj.prim.v(3)>=0.0)
               fprintf('+%.4fk\n',obj.prim.v(3))
           else
               fprintf('%.4fk\n',obj.prim.v(3))
           end
           
           %Print dual quaternion
           fprintf('+E(%.4f',obj.dual.s)
           if(obj.dual.v(1)>=0.0)
               fprintf('+%.4fi',obj.dual.v(1))
           else
               fprintf('%.4fi',obj.dual.v(1))
           end
           if(obj.dual.v(2)>=0.0)
               fprintf('+%.4fj',obj.dual.v(2))
           else
               fprintf('%.4fj',obj.dual.v(2))
           end
           if(obj.dual.v(3)>=0.0)
               fprintf('+%.4fk',obj.dual.v(3))
           else
               fprintf('%.4fk',obj.dual.v(3))
           end
           fprintf(')\n')
           
       end
       
       %Dual quaternion addition
       function obj=plus(dq1,dq2)
           obj=DualQuat(dq1.prim+dq2.prim,dq1.dual+dq2.dual);
       end
       
       %Dual quaternion difference
       function obj=minus(dq1,dq2)
           obj=DualQuat(dq1.prim-dq2.prim,dq1.dual-dq2.dual);
       end      
       
       %Dual quaternion multiplication
       function obj=mtimes(dq1,dq2)
           if isa(dq1,'double') %If the first argument is a scalar
               prim=dq1*dq2.prim;
               dual=dq1*dq2.dual;
               obj=DualQuat(prim,dual);  %Create the dual quaternion object using prim and dual
               return
           end
           if isa(dq2,'double') %If the second argument is a scalar
               prim=dq2*dq1.prim;
               dual=dq2*dq1.dual;
               obj=DualQuat(prim,dual);  %Create the dual quaternion object using prim and dual
               return
           end
           if isa(dq1,'DualQuat')&&isa(dq2,'DualQuat') %If both arguments are quaternions
               prim=dq1.prim*dq2.prim;
               dual=dq1.prim*dq2.dual+dq1.dual*dq2.prim;
               obj=DualQuat(prim,dual);  %Create the dual quaternion object using prim and dual
               return
           end           
       end
       
       %Dual quaternion division
       function obj=mrdivide(dq,scalar)
           if isa(scalar,'double')
               prim=dq.prim/scalar;
               dual=dq.dual/scalar;
               obj=DualQuat(prim,dual);  %Create the dual quaternion object using prim and dual
           else
               error('The second argument must be a scalar')
           end
       end

        %Dual quaternion conjugate
        function obj=conj(this)
            prim=this.prim.conj();
            dual=this.dual.conj();
            obj=DualQuat(prim,dual);
        end
       
        %Dual quaternion norm
        function n=norm(this)
            dq=this*this.conj();
            n=dq;
        end
   end
end