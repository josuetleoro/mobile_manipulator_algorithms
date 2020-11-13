function [th1_1,th1_2,th2_1,th2_2,sol]=subproblem2(d1,d2,p,q,r)
    switch nargin
    case 4
       u=p; v=q;
    case 5
       u=p-r;
       v=q-r;
    otherwise
        error('Wrong number of input arguments')
    end
    
    dot_d1d2=d1'*d2;
    cross_d1d2=cross(d1,d2);
        
    %Calcualte alpha, beta and gamma
    alpha=(dot_d1d2*d2'*u-d1'*v)/(dot_d1d2^2-1);
    beta=(dot_d1d2*d1'*v-d2'*u)/(dot_d1d2^2-1);
    gamma2=(dot(u,u)-alpha^2-beta^2-2*alpha*beta*d1'*d2)/(dot(cross_d1d2,cross_d1d2));

    %We start assuming we have two solutions
    sol=2; 
    if(abs(gamma2)<1.0e-8) %Check if the value is really small which means it is actually zero
        sol=1;
        z1=alpha*d1+beta*d2;
        %One solution
        th1_1=subproblem1(d1,z1,v);
        th2_1=subproblem1(d2,u,z1);      
        th1_2=0;
        th2_2=0;        
    elseif gamma2<0 %If gamma2 is negative there is no solution        
        th1_1=0;th1_2=0;th2_1=0;th2_2=0;
        sol=0;
    else %Otherwise we have two solutions
        gamma=sqrt(gamma2);            
        %Calculate z1 and z2
        z1=alpha*d1+beta*d2+gamma*cross_d1d2;
        z2=alpha*d1+beta*d2-gamma*cross_d1d2;    
        
        %First solution
        th1_1=subproblem1(d1,z1,v);
        th2_1=subproblem1(d2,u,z1);

        %Second solution
        th1_2=subproblem1(d1,z2,v);
        th2_2=subproblem1(d2,u,z2);           
    end    
end