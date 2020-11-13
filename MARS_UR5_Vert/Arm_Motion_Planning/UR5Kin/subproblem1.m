function theta=subproblem1(d,p,q,r)
switch nargin
    case 3
        up=p-d*d'*p;
        vp=q-d*d'*q;
        theta=atan2(d'*cross(up,vp),up'*vp);
    case 4
        u=p-r;
        v=q-r;
        up=u-d*d'*u;
        vp=v-d*d'*v;
        theta=atan2(d'*cross(up,vp),up'*vp);
    otherwise
        error('Wrong number of input arguments')
end
    
end