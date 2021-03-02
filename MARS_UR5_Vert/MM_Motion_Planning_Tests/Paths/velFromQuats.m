function vel=velFromQuats(p,q,ts)
%%      When two quaterions are close together        
        qdiff = quatmultiply(q',quatconj(p'))';
        vel = 2*qdiff(2:4)/ts;
        if qdiff(1) < 0
            vel = -1*vel;
        end

%%        When the two quaternions are not very close together
%         len = norm(qdiff);
%         angle = 2*atan2(len, qdiff(1))
%         axis = [0;0;0];
%         if len > 0
%             axis = qdiff(1:3)/len;
%         else
%             axis = [1;0;0];
%         end
%         vel = axis*angle/ts        

end