function dxyh = compute_tracking_err_at_target(xyh, xyh1)
dh   = xyh(3) - xyh1(3);
bx   = xyh(1) - xyh1(1);
by   = xyh(2) - xyh1(2);
dx   =  bx * cos(xyh(3)) + by * sin(xyh(3));
dy   = -bx * sin(xyh(3)) + by * cos(xyh(3));
dxyh = [dx; dy; dh];
end