function q=quatMult(q1,q2)
q1s=q1(1);
q1v=q1(2:4);

q2s=q2(1);
q2v=q2(2:4);

s=q1s*q2s-dot(q1v,q2v);
v=q1s*q2v+q2s*q1v+cross(q1v,q2v);
q=[s;v];  %Create the resulting quaternion using s and v
end