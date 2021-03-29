function plot_trajectory(trj, params)

if nargin < 2
    params = get_default_truck_trailer_params();
end

trj = truck_trailer_trajectory(trj);
[~,ia] = unique(trj.cumDistLeft);
ia = sort(ia);

dist_step_to_plot = 1;
dist_to_plot = max(trj.cumDistLeft):-dist_step_to_plot:0;

trj_resampled = interp1(-trj.cumDistLeft(ia), trj.Data(ia,:), -dist_to_plot);

figure()
for ct = 1:numel(dist_to_plot)
    [xx,yy] = local_get_truck_points(trj_resampled(ct,1), ...
        trj_resampled(ct,2), ...
        trj_resampled(ct,3), ...
        params);
    polygon = polyshape(xx,yy);
    plot(polygon, 'FaceColor', 'red', ...
        'EdgeColor', 'red', ...
        'FaceAlpha',0.01)
    [xx,yy] = local_get_trailer_points(trj_resampled(ct,1), ...
        trj_resampled(ct,2), ...
        trj_resampled(ct,3), ...
        trj_resampled(ct,4), ...
        params);
    polygon = polyshape(xx,yy);
    plot(polygon, 'FaceColor','green', ...
        'EdgeColor', 'green', ...
        'FaceAlpha',0.01)
    hold on
end
axis equal
hold off

end

function [xx, yy] = local_get_truck_points(x,y,h, p)
x0 = [-p.truckMargin;
       p.truckMargin+p.truckWheelbase
       p.truckMargin+p.truckWheelbase
      -p.truckMargin];
y0 = [ p.truckWidth
       p.truckWidth
      -p.truckWidth
      -p.truckWidth]/2;
xx =  x0 * cos(h) - y0 * sin(h) + x;
yy =  x0 * sin(h) + y0 * cos(h) + y;
end

function [xx, yy] = local_get_trailer_points(x,y,h, g, p)
x0 = [ ...
      -p.trailerMarginBack-p.trailerWheelbase
       p.trailerMarginFront
       p.trailerMarginFront
      -p.trailerMarginBack-p.trailerWheelbase];
y0 = [ p.trailerWidth
       p.trailerWidth
      -p.trailerWidth
      -p.trailerWidth]/2;
xx =  x0 * cos(h+g) - y0 * sin(h+g) + x;
yy =  x0 * sin(h+g) + y0 * cos(h+g) + y;
end