tt = truckTrailerOnAxle();
strctrl  = lqrSteeringControl(tt); strctrl.reset();
spdctrl  = simpleSpeedControl(tt);

wp = [1 1 0 0];

for t = 0:0.1:10
    sw = strctrl.compute(wp);
    vm = spdctrl.compute(wp);
    tt.set_speed(vm);
    tt.set_steering(sw);
    tt.drive();
    fprintf("%f, %f, %f \n", tt.x_m, tt.y_m, tt.v_mps);
end