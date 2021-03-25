tt = truckTrailerOnAxle();
tt_plot =  truck_trailer_plot_simple();
strctrl  = lqrSteeringControl(tt); strctrl.reset();
spdctrl  = simpleSpeedControl(tt);

wp = [1 5 0 0];

for t = 0:0.1:10    
    sw = strctrl.compute(wp);
    vm = spdctrl.compute(wp);
    tt.set_speed(vm);
    tt.set_steering(sw);
    tt_plot.updateFig([tt.x_m; tt.y_m; tt.h_rad; tt.g_rad], tt.s_deg, 0);
    drawnow;
    tt.drive();    
    fprintf("%f, %f, %f \n", tt.x_m, tt.y_m, tt.s_deg);
end