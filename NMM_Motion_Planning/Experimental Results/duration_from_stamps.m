function secs=duration_from_stamps(stamp1, stamp2)
sec_part = stamp1(1) - stamp2(1);
nsec_part = stamp1(2) - stamp2(2);
while (nsec_part > 1000000000)
    nsec_part = nsec_part - 1000000000;
    sec_part = sec_part + 1;
end
while (nsec_part < 0)
    nsec_part = nsec_part + 1000000000;
    sec_part = sec_part - 1;
end
secs = sec_part + 1e-9 * nsec_part;
end