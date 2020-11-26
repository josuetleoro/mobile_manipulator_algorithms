function [th1_1,th1_2,p1qV,sol]=subproblem3_new(d,p,q,r,params)
    %params (da,R,b,b_2,c)
    da=params{1};
    R=params{2};
    b=params{3};
    b_2=params{4};
    c=params{5};
    
    qV=((q - p)'*d)*d;
    Q=q-qV-r;
    s=b_2-4*(c-Q'*Q);
    s=sqrt(s); %Still need to check if valid
    P(:,1)=R-0.5*(b+s)*da;
    P(:,2)=R-0.5*(b-s)*da;
    
    p1qV(:,1)=r+qV+P(:,1);
    p1qV(:,2)=r+qV+P(:,2);
    
    %Use subproblem 1
    th1_1=subproblem1(d,P(:,1),Q);
    th1_2=subproblem1(d,P(:,2),Q);
    sol=2;
end