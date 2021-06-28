function [th1_1,th1_2,th2_1,th2_2,sol]=subproblem2pa(d1,p,q,r1,r2x)
    rq=(r1 - r2x)'*d1*d1;
    r2=r2x+rq;
    u=p-r1;
    v=q-r2;
    rp=r1-r2;
    d2=rp/norm(rp);
    
    cross_d1d2=cross(d1,d2);
    %Calcualte alpha, beta and gamma
    alpha=dot(d1,u);   
    beta=(dot(u,u)-dot(v,v)-dot(rp,rp))/(-2*dot(rp,d2));
    gamma2=dot(v,v)-alpha*alpha-beta*beta;
    %gamma2
    
    %We start assuming we have two solutions
    sol=2;  
    if abs(gamma2)<1.0e-15     %Check if the value is really small which means it is actually zero
        sol=1;
        %Calculate z1 and z2
        z2=alpha*d1+beta*d2;
        z1=z2-rp;
        %One solution
        th1_1=subproblem1(d1,z2,v);
        th2_1=subproblem1(d1,u,z1);
        th1_2=0;
        th2_2=0;
    elseif gamma2<0 %If gamma2 is negative there is no solution
        sol=0;
        th1_1=0;th1_2=0;th2_1=0;th2_2=0;
    else %Otherwise we have two solutions
        gamma=sqrt(gamma2);
        %Calculate z1 and z2
        z2=alpha*d1+beta*d2+gamma*cross_d1d2;
        z1=z2-rp;
        
        %First solution
        th1_1=subproblem1(d1,z2,v);
        th2_1=subproblem1(d1,u,z1);
        
        %Calculate z1 and z2
        z2=alpha*d1+beta*d2-gamma*cross_d1d2;
        z1=z2-rp;
        
        %Second solution
        th1_2=subproblem1(d1,z2,v);
        th2_2=subproblem1(d1,u,z1);     
    end
end