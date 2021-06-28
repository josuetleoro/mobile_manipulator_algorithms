function [th1_1,th1_2,c,sol]=subproblem3_new(d,p,q,r,params)
    % This version projects q and c on the plane where p is located.
    % This is a more efficient version because it requires less
    % calculations

    omega3=params{1};   % omega3
    y=params{2};    % (Pw0-p1) or y
    b=params{3};    % (Pw0-p1)'*omega3 or 2y'omega3
    b_2=params{4};  % b^2
    y_norm=params{5};    % (Pw0-p1)'(Pw0-p1) or y'y
   
    qV=((q - p)'*d)*d;
    Q=q-qV-r;                % q projected onto the plane of pw0
    s=sqrt(b_2-(y_norm-Q'*Q));
    C(:,1)=y+(-b-s)*omega3;  % c when projected onto the plane of pw0
    C(:,2)=y+(-b+s)*omega3;  % c when projected onto the plane of pw0
    
    c(:,1)=C(:,1)+qV+r;  % Point c in new supbroblem
    c(:,2)=C(:,2)+qV+r;  % Point c in new supbroblem
    
    %Use subproblem 1
    th1_1=subproblem1(d,C(:,1),Q); % C and Q projected onto the plane on p0
    th1_2=subproblem1(d,C(:,2),Q); % C and Q projected onto the plane on p0
    sol=2;
end