p = get_default_truck_trailer_params();
ic = [0 1 -0.2 0.3];
figure(1)
tp(1) = truck_trailer_plot_simple(p, gca);
tp(1).updateFig(ic);


figure(2)
p = get_default_truck_trailer_params();
ic = [20 3.2 0.5 -0.3];
tp(1) = truck_trailer_plot_simple(p, gca);
tp(1).updateFig(ic);

figure(3)
p = get_default_truck_trailer_params();
ic = [20 0 0 0];
tp(1) = truck_trailer_plot_simple(p, gca);
tp(1).updateFig(ic);
