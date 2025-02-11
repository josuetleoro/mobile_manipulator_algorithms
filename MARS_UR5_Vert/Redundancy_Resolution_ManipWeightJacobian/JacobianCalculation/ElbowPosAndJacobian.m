> Matlab(p[elbow], resultname = "pos_elbow");
pos_elbow = [-0.425e0 * (-cos(phi) * cos(q1) + sin(phi) * sin(q1)) * cos(q2) - 0.49e-1 * cos(phi) -0.425e0 * (-cos(phi) * sin(q1) - sin(phi) * cos(q1)) * cos(q2) - 0.49e-1 * sin(phi) 0.645359e0 - 0.425e0 * sin(q2) + zmp];
> Matlab(JelbowBar, resultname = "Jelbow");
Jelbow = [cos(phi) -0.2125000000e0 * sin(phi + q1 + q2) - 0.2125000000e0 * sin(phi + q1 - q2) + 0.49e-1 * sin(phi) 0 -0.2125000000e0 * sin(phi + q1 + q2) - 0.2125000000e0 * sin(phi + q1 - q2) -0.2125000000e0 * sin(phi + q1 + q2) + 0.2125000000e0 * sin(phi + q1 - q2) 0 0 0 0; sin(phi) 0.2125000000e0 * cos(phi + q1 - q2) + 0.2125000000e0 * cos(phi + q1 + q2) - 0.49e-1 * cos(phi) 0 0.2125000000e0 * cos(phi + q1 - q2) + 0.2125000000e0 * cos(phi + q1 + q2) -0.2125000000e0 * cos(phi + q1 - q2) + 0.2125000000e0 * cos(phi + q1 + q2) 0 0 0 0; 0 0 0.1e1 0 -0.4250000000e0 * cos(q2) 0 0 0 0;];
> writeto(terminal);
