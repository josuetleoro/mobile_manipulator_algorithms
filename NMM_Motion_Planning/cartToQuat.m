 function q=cartToQuat(R)

% %Animating Rotation with Quaternion Curves
% %Step 1: Find w^2
% w2=0.25*(1+R(1,1)+R(2,2)+R(3,3));
% 
% zero_th=0.00001;
% if(w2>zero_th) %Check the condition for w2
%     w=sqrt(w2);
%     x=(R(2,3)-R(3,2))/(4*w);
%     y=(R(3,1)-R(1,3))/(4*w);
%     z=(R(1,2)-R(2,1))/(4*w);
% else
%     w=0;
%     %Calculate x2
%     x2=-0.5*(R(2,2)+R(3,3));
%     if(x2>zero_th) %Check the condition for x2
%         x=sqrt(x2);
%         y=R(1,2)/(2*x);
%         z=R(1,3)/(2*x);
%     else
%         x=0;
%         %Calculate y2
%         y2=0.5*(1-R(3,3));
%         if(y2>zero_th)
%             y=sqrt(y2);
%             z=R(2,3)/(2*y);
%         else
%             y=0;
%             z=1;
%         end
%     end
% end
% q=[w;x;y;z];

% %Multiple Attitude Representations
% %%%%%%%%%%%%%%Conversion Algoritm 2%%%%%%%%%%%%
% if (R(2,2)>-R(3,3))&&(R(1,1)>-R(2,2))&&(R(1,1)>-R(3,3))
%     den=sqrt(1+R(1,1)+R(2,2)+R(3,3));
%     q=1/2*[den;
%            (R(2,3)-R(3,2))/den;
%            (R(3,1)-R(1,3))/den;
%            (R(1,2)-R(2,1))/den];
% elseif (R(2,2)<-R(3,3))&&(R(1,1)>R(2,2))&&(R(1,1)>R(3,3))
%     den=sqrt(1+R(1,1)-R(2,2)-R(3,3));
%     q=1/2*[(R(2,3)-R(3,2))/den;
%             den;
%            (R(1,2)+R(2,1))/den;
%            (R(3,1)+R(1,3))/den];
% elseif (R(2,2)>R(3,3))&&(R(1,1)<R(2,2))&&(R(1,1)<-R(3,3))
%     den=sqrt(1-R(1,1)+R(2,2)-R(3,3));
%     q=1/2*[(R(3,1)-R(1,3))/den;
%            (R(1,2)+R(2,1))/den;
%             den;
%            (R(2,3)+R(3,2))/den];           
% elseif (R(2,2)<R(3,3))&&(R(1,1)<-R(2,2))&&(R(1,1)<R(3,3))
%     den=sqrt(1-R(1,1)-R(2,2)+R(3,3));
%     q=1/2*[(R(1,2)-R(2,1))/den;
%            (R(3,1)+R(1,3))/den;
%            (R(2,3)+R(3,2))/den;
%             den];   
% end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %Bruno Siciliano (Working but representation problem)
% %%%%%%%%%%%%%%Conversion Algoritm 3%%%%%%%%%%%%%%%%%%%%
% w=1/2*sqrt(R(1,1)+R(2,2)+R(3,3)+1);
% x=1/2*sign(R(3,2)-R(2,3))*sqrt(R(1,1)-R(2,2)-R(3,3)+1);
% y=1/2*sign(R(1,3)-R(3,1))*sqrt(R(2,2)-R(3,3)-R(1,1)+1);
% z=1/2*sign(R(2,1)-R(1,2))*sqrt(R(3,3)-R(1,1)-R(2,2)+1);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% q=[w;x;y;z];

%%%%%%%%%%%%%% Matlab Aerospace toolbox %%%%%%%%%%%%
quat_aux = rotm2quat(R);
q=quat_aux';
 end