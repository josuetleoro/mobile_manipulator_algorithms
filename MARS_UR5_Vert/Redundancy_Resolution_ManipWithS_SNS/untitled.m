A=[1,2,1;4,5,6;7,8,9];
detA=det(A)
W=eye(3,3);
W(2,2)=0;
B=A*W
T=B*B'
det(T)
T2=(A*A')*W
det(T2)