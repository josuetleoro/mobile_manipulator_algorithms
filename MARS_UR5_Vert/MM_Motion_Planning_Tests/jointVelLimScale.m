function smin=jointVelLimScale(dq,dq_max)
n = length(dq);
dqlim_exceeded = false;
for i=1:n
    if abs(dq(i)) > dq_max(i)
        dqlim_exceeded = true;
        break;
    end
end
if dqlim_exceeded
    vel_scale = zeros(6,1);
    for i=1:n
        vel_scale(i) = dq_max(i) / abs(dq(i));
    end
    min_scale = min(vel_scale);
    if min_scale >= 1.0
        smin = 1.0;
    else
        smin = min_scale;
    end    
else
    smin = 1.0;
end
end