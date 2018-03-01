classdef LinePlucker
    properties (Access=private)
       mom %moment
       dir %direction
    end
    
    methods
        %Constructor
        function obj=LinePlucker(p,d) %The input must be a point in the line 
                                      %and the direction unit vector
            if size(p,2)~=3
               error('The size of the point vector must be 3');
               return
            end
            if size(d,2)~=3
               error('The size of the direction vector must be 3');
               return
            end
            if norm(d)~=1
               error('The direction vector must be unitary');
               return
            end           
            obj.mom=cross(p,d);
            obj.dir=d;            
        end
        
        %LinePlucker display
        function disp(obj)
            %Usage: disp(l)
            %Purpose: display the plucker coordinates members of a LinePlucker object
            %Input: l -- LinePlucker object
            %Output: Display the members of LinePlucker (m,d)
            fprintf('(%.4f %.4f %.4f, %.4f %.4f %.4f)\n',obj.mom(1),obj.mom(2),obj.mom(3),obj.dir(1),obj.dir(2),obj.dir(3))
        end
        
        function m=getMom(this)
            m=this.mom;            
        end
                
        function d=getDir(this)
            d=this.dir;            
        end
    end
    
    methods(Static)
        function r=lineInt(varargin)
            switch nargin
                case 2  
                    %2 arguments - Two LinePlucker objects
                    if isa(varargin{1},'LinePlucker')&&isa(varargin{2},'LinePlucker')
                        ma=varargin{1}.getMom;
                        da=varargin{1}.getDir;
                        mb=varargin{2}.getMom;
                        db=varargin{2}.getDir; 
                    end
                    %2 arguments - Two DualQuaternions Line objects
                    if isa(varargin{1},'DualQuat')&&isa(varargin{2},'DualQuat')
                        ma=varargin{1}.dual.getV;
                        da=varargin{1}.prim.getV;
                        mb=varargin{2}.dual.getV;
                        db=varargin{2}.prim.getV;                    
                    end                    
                case 4  %4 arguments - ma,da,mb,db
                    ma=varargin{1};
                    da=varargin{2};
                    mb=varargin{3};
                    db=varargin{4};
                otherwise
                    error('Wrong number of input arguments');
                    return
            end
            r=cross(da,ma)+dot(cross(db,mb),da)*da;
        end
    end %End static methods    
end