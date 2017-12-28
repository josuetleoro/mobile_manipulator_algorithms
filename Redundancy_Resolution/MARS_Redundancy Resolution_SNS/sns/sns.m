function dq=sns(qk,dx,ts,J,q_limit,dq_limit,ddq_limit)
%dx=task velocity
%J=actual jacobian matrix

%Get the number of degrees of freedom and the task dimension
n=size(J,2);
m=length(dx);
%Initialize the weights, dq_N, s and s_star
W=eye(n);
dq_N=zeros(n,1);
s=1.0;
%Best solution
s_star=-1.0;
dq_N_star = zeros(n,1);
limit_exceeded=false;

%Get the velocity bounds
[dQMax, dQMin]=shapeJointVelBound(qk,q_limit,dq_limit,ddq_limit,ts);

%Use algorithm 1
while true
    JW_inv=pinv(J*W);
    dq=dq_N+JW_inv*(s*dx-J*dq_N);
    dq_N;
    
    a = JW_inv*dx;
    b = dq - a;
    
    % j: most critical joint
    [scalingFactor,j]=getTaskScalingFactor(a,b,W,dQMax,dQMin);

    %Check if the current velocity violates any of the box contrains by
    %evaluating the scalingFactor
    if scalingFactor >= 1.0
        disp('No task scaling needed')
        break;
    else
        limit_exceeded=true;
        %%%%%% TODO TASK %%%%%%
        % Check for singularity                
        
        %Save best solution so far
        if(scalingFactor > s_star)
            s_star=scalingFactor;
            dq_N_star=dq_N;
            JW_inv_star=JW_inv;
        end
        
        %Saturate most critical joint
        W(j,j)=0;
        if dq(j) > dQMax(j)
            dq_N(j) = dQMax(j);        
        end
        if dq(j) < dQMin(j)
            dq_N(j) = dQMin(j);        
        end       
        %Check if the rank of JW is strictly less than m
        if rank(J*W) < m
            if (s_star>=0.0)
                disp('SNS algorithm completed');
                dq_N=dq_N_star;
                dq=dq_N_star+JW_inv_star*(s_star*dx-J*dq_N);
                limit_exceeded=false; %The algorithm finishes (outputs best solution)
            else
                disp('task could not be executed: reached sing');
            end
            disp('Best scaling factor')
            s_star
        end        
    end
    
    if ~limit_exceeded
        break;
    end
end

end