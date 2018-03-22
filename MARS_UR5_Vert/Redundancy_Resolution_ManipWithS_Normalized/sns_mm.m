function [eta_sns,s,W]=sns_mm(dQmin,dQmax,dq_N,J,dx,error_cont)
%Get the number of degrees of freedom and the task dimension
n=size(J,2);
m=length(dx);

%Initialize the weights, dq_N, s and s_star
W=eye(n);
%dq_N=0;
s=1.0;
%Best solution
s_star=-1.0;
dq_N_star = dq_N; %Start with the one given by the manipulability measure
inv_JW_star = pinv(J*W);
W_star=W;

count = 0;
limit_exceeded=true;
%Use algorithm 1
while limit_exceeded
    count = count + 1;
    if(count > 2*n)
        disp('Infinite loop for the given task');
        s = -1;
        break;
    end
    
    limit_exceeded=false;
    JW_inv=pinv(J*W);
    dq=dq_N+JW_inv*(s*dx+error_cont-J*dq_N);
    
    a = JW_inv*dx;
    b = dq - a;
    
    % j: most critical joint
    [scalingFactor,j]=getTaskScalingFactor(a,b,W,dQmax,dQmin);
    %W
    %scalingFactor
    %pause
    
    %Check if the current velocity violates any of the box contrains by
    %evaluating the scalingFactor
    if scalingFactor >= 1.0
        %pause()
        %disp('No task scaling needed')
    else
        limit_exceeded=true;
        disp('saturation applied');
        %%%%%% TODO TASK %%%%%%
        % Check for singularity                
        
        %Save best solution so far
        %scalingFactor
        %s_star
        %pause()
        if(scalingFactor > s_star)
            s_star=scalingFactor;
            W_star=W;
            inv_JW_star = JW_inv;
            dq_N_star=dq_N;            
        end
%         scalingFactor
%         pause()
        j
        %Saturate most critical joint
        W(j,j)=0;
        if dq(j) > dQmax(j)
            dq_N(j) = dQmax(j);        
        end
        if dq(j) < dQmin(j)
            dq_N(j) = dQmin(j);        
        end       
        %Check if the rank of JW is strictly less than m
        if rank(J*W) < m
            %scalingFactor = s_star;
            s = s_star;
            W = W_star;
            dq_N = dq_N_star;
            %JW_inv = pinv(J*W);
            dq=dq_N+inv_JW_star*(s*dx+error_cont-J*dq_N);
            %limit_exceeded = false;
            if(s_star>=0.0)
                limit_exceeded = false;
            else
                disp('task could not be executed: reached sign')
                %pause()
                scalingFactor = s_star;
                break;
            end
                
%             if (s_star>=0.0)
%                 disp('SNS algorithm completed');
%                 dq_N=dq_N_star;
%                 JW_inv=pinv(J*W);
%                 dq=S'*dq_N_star+JW_inv*(s_star*dx-J*S'*dq_N);
%                 limit_exceeded=false; %The algorithm finishes (outputs best solution)
%             else
%                 disp('task could not be executed: reached sing');
%                 s_star=1;
%             end
            %disp('Best scaling factor')
            %s_star
        end        
    end   
end
eta_sns=dq;
end