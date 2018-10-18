function [dJdq1,dJdq2,dJdq3,dJdq4,dJdq5]=evaluateUR5dJdq(q1,q2,q3,q4,q5)
dJdq1 = [-0.2125000000e0 * cos(q1 - q2) - 0.2125000000e0 * cos(q1 + q2) + 0.4732500000e-1 * sin(q4 + q3 + q1 + q2) - 0.4732500000e-1 * sin(-q4 - q3 + q1 - q2) - 0.1961250000e0 * cos(q3 + q1 + q2) - 0.1961250000e0 * cos(-q3 + q1 - q2) - 0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) + 0.4115000000e-1 * sin(q1 + q5) + 0.4115000000e-1 * sin(q1 - q5) + 0.10915e0 * sin(q1) 0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) + 0.4732500000e-1 * sin(-q4 - q3 + q1 - q2) + 0.4732500000e-1 * sin(q4 + q3 + q1 + q2) + 0.1961250000e0 * cos(-q3 + q1 - q2) - 0.1961250000e0 * cos(q3 + q1 + q2) - 0.2125000000e0 * cos(q1 + q2) + 0.2125000000e0 * cos(q1 - q2) 0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) + 0.4732500000e-1 * sin(-q4 - q3 + q1 - q2) + 0.4732500000e-1 * sin(q4 + q3 + q1 + q2) + 0.1961250000e0 * cos(-q3 + q1 - q2) - 0.1961250000e0 * cos(q3 + q1 + q2) 0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) + 0.4732500000e-1 * sin(-q4 - q3 + q1 - q2) + 0.4732500000e-1 * sin(q4 + q3 + q1 + q2) 0.4115000000e-1 * sin(q1 + q5) - 0.4115000000e-1 * sin(q1 - q5) - 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) 0; -0.2125000000e0 * sin(q1 + q2) - 0.2125000000e0 * sin(q1 - q2) - 0.1961250000e0 * sin(q3 + q1 + q2) - 0.1961250000e0 * sin(-q3 + q1 - q2) - 0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) - 0.4115000000e-1 * cos(q1 - q5) - 0.4115000000e-1 * cos(q1 + q5) + 0.4732500000e-1 * cos(-q4 - q3 + q1 - q2) - 0.4732500000e-1 * cos(q4 + q3 + q1 + q2) - 0.10915e0 * cos(q1) -0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) - 0.4732500000e-1 * cos(-q4 - q3 + q1 - q2) - 0.4732500000e-1 * cos(q4 + q3 + q1 + q2) + 0.1961250000e0 * sin(-q3 + q1 - q2) - 0.1961250000e0 * sin(q3 + q1 + q2) - 0.2125000000e0 * sin(q1 + q2) + 0.2125000000e0 * sin(q1 - q2) -0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) - 0.4732500000e-1 * cos(-q4 - q3 + q1 - q2) - 0.4732500000e-1 * cos(q4 + q3 + q1 + q2) + 0.1961250000e0 * sin(-q3 + q1 - q2) - 0.1961250000e0 * sin(q3 + q1 + q2) -0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) - 0.4732500000e-1 * cos(-q4 - q3 + q1 - q2) - 0.4732500000e-1 * cos(q4 + q3 + q1 + q2) -0.4115000000e-1 * cos(q1 + q5) + 0.4115000000e-1 * cos(q1 - q5) + 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) 0; 0 0 0 0 0 0; 0 -cos(q1) -cos(q1) -cos(q1) cos(-q4 - q3 + q1 - q2) / 0.2e1 - cos(q4 + q3 + q1 + q2) / 0.2e1 cos(q5 - q4 - q3 + q1 - q2) / 0.4e1 - cos(-q5 - q4 - q3 + q1 - q2) / 0.4e1 + cos(q5 + q4 + q3 + q1 + q2) / 0.4e1 - cos(-q5 + q4 + q3 + q1 + q2) / 0.4e1 - cos(q1 + q5) / 0.2e1 - cos(q1 - q5) / 0.2e1; 0 -sin(q1) -sin(q1) -sin(q1) sin(-q4 - q3 + q1 - q2) / 0.2e1 - sin(q4 + q3 + q1 + q2) / 0.2e1 sin(q5 - q4 - q3 + q1 - q2) / 0.4e1 - sin(-q5 - q4 - q3 + q1 - q2) / 0.4e1 + sin(q5 + q4 + q3 + q1 + q2) / 0.4e1 - sin(-q5 + q4 + q3 + q1 + q2) / 0.4e1 - sin(q1 + q5) / 0.2e1 - sin(q1 - q5) / 0.2e1; 0 0 0 0 0 0;];
dJdq2 = [-0.2125000000e0 * cos(q1 - q2) - 0.2125000000e0 * cos(q1 + q2) + 0.4732500000e-1 * sin(q4 + q3 + q1 + q2) - 0.4732500000e-1 * sin(-q4 - q3 + q1 - q2) - 0.1961250000e0 * cos(q3 + q1 + q2) - 0.1961250000e0 * cos(-q3 + q1 - q2) - 0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) + 0.4115000000e-1 * sin(q1 + q5) + 0.4115000000e-1 * sin(q1 - q5) + 0.10915e0 * sin(q1) 0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) + 0.4732500000e-1 * sin(-q4 - q3 + q1 - q2) + 0.4732500000e-1 * sin(q4 + q3 + q1 + q2) + 0.1961250000e0 * cos(-q3 + q1 - q2) - 0.1961250000e0 * cos(q3 + q1 + q2) - 0.2125000000e0 * cos(q1 + q2) + 0.2125000000e0 * cos(q1 - q2) 0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) + 0.4732500000e-1 * sin(-q4 - q3 + q1 - q2) + 0.4732500000e-1 * sin(q4 + q3 + q1 + q2) + 0.1961250000e0 * cos(-q3 + q1 - q2) - 0.1961250000e0 * cos(q3 + q1 + q2) 0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) + 0.4732500000e-1 * sin(-q4 - q3 + q1 - q2) + 0.4732500000e-1 * sin(q4 + q3 + q1 + q2) 0.4115000000e-1 * sin(q1 + q5) - 0.4115000000e-1 * sin(q1 - q5) - 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) 0; -0.2125000000e0 * sin(q1 + q2) - 0.2125000000e0 * sin(q1 - q2) - 0.1961250000e0 * sin(q3 + q1 + q2) - 0.1961250000e0 * sin(-q3 + q1 - q2) - 0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) - 0.4115000000e-1 * cos(q1 - q5) - 0.4115000000e-1 * cos(q1 + q5) + 0.4732500000e-1 * cos(-q4 - q3 + q1 - q2) - 0.4732500000e-1 * cos(q4 + q3 + q1 + q2) - 0.10915e0 * cos(q1) -0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) - 0.4732500000e-1 * cos(-q4 - q3 + q1 - q2) - 0.4732500000e-1 * cos(q4 + q3 + q1 + q2) + 0.1961250000e0 * sin(-q3 + q1 - q2) - 0.1961250000e0 * sin(q3 + q1 + q2) - 0.2125000000e0 * sin(q1 + q2) + 0.2125000000e0 * sin(q1 - q2) -0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) - 0.4732500000e-1 * cos(-q4 - q3 + q1 - q2) - 0.4732500000e-1 * cos(q4 + q3 + q1 + q2) + 0.1961250000e0 * sin(-q3 + q1 - q2) - 0.1961250000e0 * sin(q3 + q1 + q2) -0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) - 0.4732500000e-1 * cos(-q4 - q3 + q1 - q2) - 0.4732500000e-1 * cos(q4 + q3 + q1 + q2) -0.4115000000e-1 * cos(q1 + q5) + 0.4115000000e-1 * cos(q1 - q5) + 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) 0; 0 0 0 0 0 0; 0 -cos(q1) -cos(q1) -cos(q1) cos(-q4 - q3 + q1 - q2) / 0.2e1 - cos(q4 + q3 + q1 + q2) / 0.2e1 cos(q5 - q4 - q3 + q1 - q2) / 0.4e1 - cos(-q5 - q4 - q3 + q1 - q2) / 0.4e1 + cos(q5 + q4 + q3 + q1 + q2) / 0.4e1 - cos(-q5 + q4 + q3 + q1 + q2) / 0.4e1 - cos(q1 + q5) / 0.2e1 - cos(q1 - q5) / 0.2e1; 0 -sin(q1) -sin(q1) -sin(q1) sin(-q4 - q3 + q1 - q2) / 0.2e1 - sin(q4 + q3 + q1 + q2) / 0.2e1 sin(q5 - q4 - q3 + q1 - q2) / 0.4e1 - sin(-q5 - q4 - q3 + q1 - q2) / 0.4e1 + sin(q5 + q4 + q3 + q1 + q2) / 0.4e1 - sin(-q5 + q4 + q3 + q1 + q2) / 0.4e1 - sin(q1 + q5) / 0.2e1 - sin(q1 - q5) / 0.2e1; 0 0 0 0 0 0;];
dJdq3 = [0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) + 0.4732500000e-1 * sin(-q4 - q3 + q1 - q2) + 0.4732500000e-1 * sin(q4 + q3 + q1 + q2) + 0.1961250000e0 * cos(-q3 + q1 - q2) - 0.1961250000e0 * cos(q3 + q1 + q2) -0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) - 0.4732500000e-1 * sin(-q4 - q3 + q1 - q2) + 0.4732500000e-1 * sin(q4 + q3 + q1 + q2) - 0.1961250000e0 * cos(-q3 + q1 - q2) - 0.1961250000e0 * cos(q3 + q1 + q2) -0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) - 0.4732500000e-1 * sin(-q4 - q3 + q1 - q2) + 0.4732500000e-1 * sin(q4 + q3 + q1 + q2) - 0.1961250000e0 * cos(-q3 + q1 - q2) - 0.1961250000e0 * cos(q3 + q1 + q2) -0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) - 0.4732500000e-1 * sin(-q4 - q3 + q1 - q2) + 0.4732500000e-1 * sin(q4 + q3 + q1 + q2) 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) 0; -0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) - 0.4732500000e-1 * cos(-q4 - q3 + q1 - q2) - 0.4732500000e-1 * cos(q4 + q3 + q1 + q2) + 0.1961250000e0 * sin(-q3 + q1 - q2) - 0.1961250000e0 * sin(q3 + q1 + q2) -0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) - 0.4732500000e-1 * cos(q4 + q3 + q1 + q2) + 0.4732500000e-1 * cos(-q4 - q3 + q1 - q2) - 0.1961250000e0 * sin(q3 + q1 + q2) - 0.1961250000e0 * sin(-q3 + q1 - q2) -0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) - 0.4732500000e-1 * cos(q4 + q3 + q1 + q2) + 0.4732500000e-1 * cos(-q4 - q3 + q1 - q2) - 0.1961250000e0 * sin(q3 + q1 + q2) - 0.1961250000e0 * sin(-q3 + q1 - q2) -0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) - 0.4732500000e-1 * cos(q4 + q3 + q1 + q2) + 0.4732500000e-1 * cos(-q4 - q3 + q1 - q2) 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) 0; 0 0.4115000000e-1 * cos(q2 + q3 + q4 - q5) - 0.4115000000e-1 * cos(q2 + q3 + q4 + q5) + 0.9465000000e-1 * cos(q2 + q3 + q4) + 0.3922500000e0 * sin(q2 + q3) 0.4115000000e-1 * cos(q2 + q3 + q4 - q5) - 0.4115000000e-1 * cos(q2 + q3 + q4 + q5) + 0.9465000000e-1 * cos(q2 + q3 + q4) + 0.3922500000e0 * sin(q2 + q3) 0.4115000000e-1 * cos(q2 + q3 + q4 - q5) - 0.4115000000e-1 * cos(q2 + q3 + q4 + q5) + 0.9465000000e-1 * cos(q2 + q3 + q4) -0.4115000000e-1 * cos(q2 + q3 + q4 - q5) - 0.4115000000e-1 * cos(q2 + q3 + q4 + q5) 0; 0 0 0 0 -cos(-q4 - q3 + q1 - q2) / 0.2e1 - cos(q4 + q3 + q1 + q2) / 0.2e1 -cos(q5 - q4 - q3 + q1 - q2) / 0.4e1 + cos(-q5 - q4 - q3 + q1 - q2) / 0.4e1 + cos(q5 + q4 + q3 + q1 + q2) / 0.4e1 - cos(-q5 + q4 + q3 + q1 + q2) / 0.4e1; 0 0 0 0 -sin(q4 + q3 + q1 + q2) / 0.2e1 - sin(-q4 - q3 + q1 - q2) / 0.2e1 -sin(-q5 + q4 + q3 + q1 + q2) / 0.4e1 + sin(q5 + q4 + q3 + q1 + q2) / 0.4e1 + sin(-q5 - q4 - q3 + q1 - q2) / 0.4e1 - sin(q5 - q4 - q3 + q1 - q2) / 0.4e1; 0 0 0 0 sin(q2 + q3 + q4) sin(q2 + q3 + q4 - q5) / 0.2e1 - sin(q2 + q3 + q4 + q5) / 0.2e1;];
dJdq4 = [0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) + 0.4732500000e-1 * sin(-q4 - q3 + q1 - q2) + 0.4732500000e-1 * sin(q4 + q3 + q1 + q2) -0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) - 0.4732500000e-1 * sin(-q4 - q3 + q1 - q2) + 0.4732500000e-1 * sin(q4 + q3 + q1 + q2) -0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) - 0.4732500000e-1 * sin(-q4 - q3 + q1 - q2) + 0.4732500000e-1 * sin(q4 + q3 + q1 + q2) -0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) - 0.4732500000e-1 * sin(-q4 - q3 + q1 - q2) + 0.4732500000e-1 * sin(q4 + q3 + q1 + q2) 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) 0; -0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) - 0.4732500000e-1 * cos(-q4 - q3 + q1 - q2) - 0.4732500000e-1 * cos(q4 + q3 + q1 + q2) -0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) - 0.4732500000e-1 * cos(q4 + q3 + q1 + q2) + 0.4732500000e-1 * cos(-q4 - q3 + q1 - q2) -0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) - 0.4732500000e-1 * cos(q4 + q3 + q1 + q2) + 0.4732500000e-1 * cos(-q4 - q3 + q1 - q2) -0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) - 0.4732500000e-1 * cos(q4 + q3 + q1 + q2) + 0.4732500000e-1 * cos(-q4 - q3 + q1 - q2) 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) 0; 0 0.4115000000e-1 * cos(q2 + q3 + q4 - q5) - 0.4115000000e-1 * cos(q2 + q3 + q4 + q5) + 0.9465000000e-1 * cos(q2 + q3 + q4) 0.4115000000e-1 * cos(q2 + q3 + q4 - q5) - 0.4115000000e-1 * cos(q2 + q3 + q4 + q5) + 0.9465000000e-1 * cos(q2 + q3 + q4) 0.4115000000e-1 * cos(q2 + q3 + q4 - q5) - 0.4115000000e-1 * cos(q2 + q3 + q4 + q5) + 0.9465000000e-1 * cos(q2 + q3 + q4) -0.4115000000e-1 * cos(q2 + q3 + q4 - q5) - 0.4115000000e-1 * cos(q2 + q3 + q4 + q5) 0; 0 0 0 0 -cos(-q4 - q3 + q1 - q2) / 0.2e1 - cos(q4 + q3 + q1 + q2) / 0.2e1 -cos(q5 - q4 - q3 + q1 - q2) / 0.4e1 + cos(-q5 - q4 - q3 + q1 - q2) / 0.4e1 + cos(q5 + q4 + q3 + q1 + q2) / 0.4e1 - cos(-q5 + q4 + q3 + q1 + q2) / 0.4e1; 0 0 0 0 -sin(q4 + q3 + q1 + q2) / 0.2e1 - sin(-q4 - q3 + q1 - q2) / 0.2e1 -sin(-q5 + q4 + q3 + q1 + q2) / 0.4e1 + sin(q5 + q4 + q3 + q1 + q2) / 0.4e1 + sin(-q5 - q4 - q3 + q1 - q2) / 0.4e1 - sin(q5 - q4 - q3 + q1 - q2) / 0.4e1; 0 0 0 0 sin(q2 + q3 + q4) sin(q2 + q3 + q4 - q5) / 0.2e1 - sin(q2 + q3 + q4 + q5) / 0.2e1;];
dJdq5 = [0.4115000000e-1 * sin(q1 + q5) - 0.4115000000e-1 * sin(q1 - q5) - 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) 0.4115000000e-1 * sin(q1 + q5) + 0.4115000000e-1 * sin(q1 - q5) + 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) 0; -0.4115000000e-1 * cos(q1 + q5) + 0.4115000000e-1 * cos(q1 - q5) + 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) -0.4115000000e-1 * cos(q1 + q5) - 0.4115000000e-1 * cos(q1 - q5) + 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) 0; 0 -0.4115000000e-1 * cos(q2 + q3 + q4 - q5) - 0.4115000000e-1 * cos(q2 + q3 + q4 + q5) -0.4115000000e-1 * cos(q2 + q3 + q4 - q5) - 0.4115000000e-1 * cos(q2 + q3 + q4 + q5) -0.4115000000e-1 * cos(q2 + q3 + q4 - q5) - 0.4115000000e-1 * cos(q2 + q3 + q4 + q5) 0.4115000000e-1 * cos(q2 + q3 + q4 - q5) - 0.4115000000e-1 * cos(q2 + q3 + q4 + q5) 0; 0 0 0 0 0 cos(q5 - q4 - q3 + q1 - q2) / 0.4e1 + cos(-q5 - q4 - q3 + q1 - q2) / 0.4e1 + cos(q5 + q4 + q3 + q1 + q2) / 0.4e1 + cos(-q5 + q4 + q3 + q1 + q2) / 0.4e1 - cos(q1 + q5) / 0.2e1 + cos(q1 - q5) / 0.2e1; 0 0 0 0 0 sin(-q5 + q4 + q3 + q1 + q2) / 0.4e1 + sin(q5 + q4 + q3 + q1 + q2) / 0.4e1 + sin(-q5 - q4 - q3 + q1 - q2) / 0.4e1 + sin(q5 - q4 - q3 + q1 - q2) / 0.4e1 + sin(q1 - q5) / 0.2e1 - sin(q1 + q5) / 0.2e1; 0 0 0 0 0 -sin(q2 + q3 + q4 - q5) / 0.2e1 - sin(q2 + q3 + q4 + q5) / 0.2e1;];
end
