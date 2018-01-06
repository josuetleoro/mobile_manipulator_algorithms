function J=evaluateJ(q1,q2,q3,q4,q5)
J = [0 -0.4250000000e0 * cos(q2) - 0.4115000000e-1 * sin(q2 + q3 + q4 + q5) + 0.4115000000e-1 * sin(q2 + q3 + q4 - q5) - 0.3922500000e0 * cos(q2 + q3) + 0.9465000000e-1 * sin(q2 + q3 + q4) -0.4115000000e-1 * sin(q2 + q3 + q4 + q5) + 0.4115000000e-1 * sin(q2 + q3 + q4 - q5) - 0.3922500000e0 * cos(q2 + q3) + 0.9465000000e-1 * sin(q2 + q3 + q4) 0.4115000000e-1 * sin(q2 + q3 + q4 - q5) - 0.4115000000e-1 * sin(q2 + q3 + q4 + q5) + 0.9465000000e-1 * sin(q2 + q3 + q4) -0.4115000000e-1 * sin(q2 + q3 + q4 + q5) - 0.4115000000e-1 * sin(q2 + q3 + q4 - q5) 0; -0.10915e0 * sin(q1) + 0.4732500000e-1 * sin(-q4 - q3 + q1 - q2) - 0.4732500000e-1 * sin(q4 + q3 + q1 + q2) - 0.4115000000e-1 * sin(q1 + q5) - 0.4115000000e-1 * sin(q1 - q5) + 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) + 0.1961250000e0 * cos(q3 + q1 + q2) + 0.2125000000e0 * cos(q1 - q2) + 0.2125000000e0 * cos(q1 + q2) + 0.1961250000e0 * cos(-q3 + q1 - q2) -0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) - 0.4732500000e-1 * sin(q4 + q3 + q1 + q2) - 0.4732500000e-1 * sin(-q4 - q3 + q1 - q2) + 0.1961250000e0 * cos(q3 + q1 + q2) - 0.1961250000e0 * cos(-q3 + q1 - q2) - 0.2125000000e0 * cos(q1 - q2) + 0.2125000000e0 * cos(q1 + q2) -0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) - 0.4732500000e-1 * sin(q4 + q3 + q1 + q2) - 0.4732500000e-1 * sin(-q4 - q3 + q1 - q2) + 0.1961250000e0 * cos(q3 + q1 + q2) - 0.1961250000e0 * cos(-q3 + q1 - q2) -0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) - 0.4732500000e-1 * sin(q4 + q3 + q1 + q2) - 0.4732500000e-1 * sin(-q4 - q3 + q1 - q2) 0.4115000000e-1 * sin(q1 - q5) - 0.4115000000e-1 * sin(q1 + q5) + 0.2057500000e-1 * sin(q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * sin(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * sin(-q5 + q4 + q3 + q1 + q2) + 0.2057500000e-1 * sin(-q5 - q4 - q3 + q1 - q2) 0; 0.10915e0 * cos(q1) + 0.4115000000e-1 * cos(q1 + q5) + 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) + 0.4115000000e-1 * cos(q1 - q5) + 0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) + 0.1961250000e0 * sin(-q3 + q1 - q2) + 0.1961250000e0 * sin(q3 + q1 + q2) - 0.4732500000e-1 * cos(-q4 - q3 + q1 - q2) + 0.4732500000e-1 * cos(q4 + q3 + q1 + q2) + 0.2125000000e0 * sin(q1 + q2) + 0.2125000000e0 * sin(q1 - q2) 0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) + 0.4732500000e-1 * cos(-q4 - q3 + q1 - q2) + 0.4732500000e-1 * cos(q4 + q3 + q1 + q2) + 0.1961250000e0 * sin(q3 + q1 + q2) - 0.1961250000e0 * sin(-q3 + q1 - q2) + 0.2125000000e0 * sin(q1 + q2) - 0.2125000000e0 * sin(q1 - q2) 0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) + 0.4732500000e-1 * cos(-q4 - q3 + q1 - q2) + 0.4732500000e-1 * cos(q4 + q3 + q1 + q2) + 0.1961250000e0 * sin(q3 + q1 + q2) - 0.1961250000e0 * sin(-q3 + q1 - q2) 0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) + 0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) + 0.4732500000e-1 * cos(-q4 - q3 + q1 - q2) + 0.4732500000e-1 * cos(q4 + q3 + q1 + q2) -0.4115000000e-1 * cos(q1 - q5) + 0.4115000000e-1 * cos(q1 + q5) - 0.2057500000e-1 * cos(-q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * cos(-q5 + q4 + q3 + q1 + q2) - 0.2057500000e-1 * cos(q5 - q4 - q3 + q1 - q2) - 0.2057500000e-1 * cos(q5 + q4 + q3 + q1 + q2) 0; 1 0 0 0 -cos(q2 + q3 + q4) -cos(q2 + q3 + q4 - q5) / 0.2e1 + cos(q2 + q3 + q4 + q5) / 0.2e1; 0 cos(q1) cos(q1) cos(q1) cos(q4 + q3 + q1 + q2) / 0.2e1 - cos(-q4 - q3 + q1 - q2) / 0.2e1 cos(-q5 + q4 + q3 + q1 + q2) / 0.4e1 - cos(q5 + q4 + q3 + q1 + q2) / 0.4e1 + cos(-q5 - q4 - q3 + q1 - q2) / 0.4e1 - cos(q5 - q4 - q3 + q1 - q2) / 0.4e1 + cos(q1 - q5) / 0.2e1 + cos(q1 + q5) / 0.2e1; 0 sin(q1) sin(q1) sin(q1) -sin(-q4 - q3 + q1 - q2) / 0.2e1 + sin(q4 + q3 + q1 + q2) / 0.2e1 -sin(q5 - q4 - q3 + q1 - q2) / 0.4e1 + sin(-q5 - q4 - q3 + q1 - q2) / 0.4e1 - sin(q5 + q4 + q3 + q1 + q2) / 0.4e1 + sin(-q5 + q4 + q3 + q1 + q2) / 0.4e1 + sin(q1 + q5) / 0.2e1 + sin(q1 - q5) / 0.2e1;];
end