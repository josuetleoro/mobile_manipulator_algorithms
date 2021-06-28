function [th1_1,th1_2,c,sol]=subproblem3_new_paper(d,p,q,r,params)
    % This is the original version in the paper
    omega3=params{1};
    
    r_prime = r + ((q - r)'*d)*d;
    v_prime = q - r_prime;
    p_prime = p + ((q - p)'*d)*d;
    y = p_prime - r_prime;
    
    % The quadratic equation coefficients
    b_qeq = omega3'*y;
    c_qeq = y'*y - v_prime'*v_prime;
    sq = sqrt(b_qeq*b_qeq-c_qeq);
    
    x1 = (-b_qeq+sq)*omega3;
    x2 = (-b_qeq-sq)*omega3;
    
    c(:,1) = p_prime + x1;
    c(:,2) = p_prime + x2;
    
    %Use subproblem 1
    th1_1=subproblem1(d,c(:,1),q,r);
    th1_2=subproblem1(d,c(:,2),q,r);
    sol=2;    
    
end