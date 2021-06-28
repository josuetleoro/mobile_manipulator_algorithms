function quat_vel=quatVel(w)
quat_vel = Quat(cos(norm(w)/2), w/norm(w)*sin(norm(w)/2));
end