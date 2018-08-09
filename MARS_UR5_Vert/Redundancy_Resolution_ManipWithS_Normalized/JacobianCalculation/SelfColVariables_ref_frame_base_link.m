> Matlab(p_elbow, resultname = "pos_elbow");
pos_elbow = 0.645359e0 - 0.425e0 * sin(q2) + zmp;
> Matlab(p_wrist, resultname = "pos_wrist");
pos_wrist = [0.39225e0 * cos(q1) * cos(q2) * cos(q3) - 0.39225e0 * cos(q1) * sin(q2) * sin(q3) - 0.49e-1 + 0.425e0 * cos(q1) * cos(q2) 0.39225e0 * sin(q1) * cos(q2) * cos(q3) - 0.39225e0 * sin(q1) * sin(q2) * sin(q3) + 0.425e0 * sin(q1) * cos(q2) -0.39225e0 * sin(q2) * cos(q3) - 0.39225e0 * cos(q2) * sin(q3) + 0.645359e0 - 0.425e0 * sin(q2) + zmp];
> Matlab(Jelbow, resultname = "Jelbow");
Jelbow = [0 0 1 0 -0.4250000000e0 * cos(q2) 0 0 0 0];
> Matlab(Jwrist, resultname = "Jwrist");
Jwrist = [0 0 0 -0.1961250000e0 * sin(q3 + q1 + q2) - 0.1961250000e0 * sin(-q3 + q1 - q2) - 0.2125000000e0 * sin(q1 + q2) - 0.2125000000e0 * sin(q1 - q2) -0.1961250000e0 * sin(q3 + q1 + q2) + 0.1961250000e0 * sin(-q3 + q1 - q2) - 0.2125000000e0 * sin(q1 + q2) + 0.2125000000e0 * sin(q1 - q2) -0.1961250000e0 * sin(q3 + q1 + q2) + 0.1961250000e0 * sin(-q3 + q1 - q2) 0 0 0];
> writeto(terminal);
