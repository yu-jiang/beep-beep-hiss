classdef truck_trailer_plot_simple < handle
    
    properties
        %fig
        ax
        truckShape
        trailerShape
        truckPlot
        trailerPlot
        flTirePlot
        frTirePlot
        x = 0
        y = 0
        h = 0
        sa = 0
        g = 0
        t = 0
        params
        xlimit = [-15 15]
        ylimit = [-20 30]
        obstaclePlot
        targetAreaPlot
    end
    
    methods
        function self = truck_trailer_plot_simple(params, ax)
            if nargin < 1
                self.params = get_default_truck_trailer_params();
            else
                self.params = params;
            end
            
            if nargin < 2
                fig = figure(101);
                clf
                self.ax = axes('parent', fig);
            else
                self.ax = ax;
            end
             
            [x1,y1] = local_get_truck_points(self.x, self.y, self.h, self.params);
            [x2,y2] = local_get_trailer_points(self.x, self.y, self.h, self.g, self.params);
            [xfl, yfl, xfr, yfr] = local_get_tire_points( ...
                self.x, self.y, self.h, self.sa, self.params);
            
            self.truckShape   = polyshape(x1, y1);
            self.trailerShape = polyshape(x2, y2);
            
            
            % draw
            self.truckPlot   = plot(self.truckShape);
            hold on
            obstacleColor = 'k';

%             self.obstaclesPlot(1) = plot(polyshape([self.xlimit([1 1 2 2])], ...
%                                       [25 30 30 25]), ...
%                                       'FaceColor', obstacleColor);
%             self.obstaclesPlot(2) = plot(polyshape([self.xlimit([1 1 2 2])], ...
%                                       [-20 -15 -15 -20]), ...
%                                       'FaceColor', obstacleColor);
%             self.obstaclesPlot(3) = plot(polyshape(...
%             [-self.params.truckWidth/2 ...
%              -self.params.truckWidth/2 ...
%              self.params.truckWidth/2 ...
%              self.params.truckWidth/2]*1.1, ...
%             [-15 -self.params.trailerWheelbase-self.params.trailerMarginBack ...
%             -self.params.trailerWheelbase-self.params.trailerMarginBack -15]), ...
%                                       'FaceColor', 'k', ...
%                                       'FaceAlpha', 0.95);
%             self.linesPlot(1) = line( ...
%                 [-self.params.truckWidth/2 -self.params.truckWidth/2]*1.1, ...
%                 [-self.params.trailerWheelbase-self.params.trailerMarginBack ...
%                   self.params.truckWheelbase + self.params.truckMargin], ...
%                 'Color', '#c5c9c9', 'LineWidth', 1.2, 'LineStyle', '-');
%             self.linesPlot(2) = line( ...
%                 [self.params.truckWidth/2 self.params.truckWidth/2]*1.1, ...
%                 [-self.params.trailerWheelbase-self.params.trailerMarginBack ...
%                   self.params.truckWheelbase + self.params.truckMargin], ...
%                 'Color', '#c5c9c9', 'LineWidth', 1.2, 'LineStyle', '-');

            self.trailerPlot = plot(self.trailerShape);
            self.flTirePlot  = plot(polyshape(xfl, yfl), ...
                                    'FaceColor', 'k', ...
                                    'FaceAlpha', 0.9);
            self.frTirePlot  = plot(polyshape(xfr, yfr), ...
                                    'FaceColor', 'k', ...
                                    'FaceAlpha', 0.9);
            hold off
            set(self.ax, 'xlim', self.xlimit);
            set(self.ax, 'ylim', self.ylimit);
            xtklb = get(self.ax, 'XTickLabel');
            set(self.ax, 'XTickLabel', xtklb(end:-1:1));
            xlabel('y (m)')
            ylabel('x (m)')
            grid on
        end
        
        function updateFig(self, xyhp, sa_deg, t)
            if nargin < 4
                t = 0;
            end
            
            self.x = xyhp(1);
            self.y = xyhp(2);
            self.h = xyhp(3);
            self.g = xyhp(4);
            self.sa = sa_deg;
            self.t = t;
            
            [x1,y1] = local_get_truck_points(self.x, self.y, self.h, self.params);
            [x2,y2] = local_get_trailer_points(self.x, self.y, self.h, self.g, self.params);
            [flxx, flyy, frxx, fryy] = local_get_tire_points( ...
                self.x, self.y, self.h, self.sa, self.params);            
            
            self.truckPlot.Shape.Vertices = [x1 y1];
            self.trailerPlot.Shape.Vertices = [x2 y2];
            self.flTirePlot.Shape.Vertices = [flxx flyy];
            self.frTirePlot.Shape.Vertices = [frxx fryy];
            
            if self.x > 10
                obstacleColor = 'k';
            elseif abs(self.y) < 0.1 && abs(self.h+self.g) < 0.05 
                obstacleColor = 'g';
            else
                obstacleColor = 'r';
            end
            
%            set(self.obstaclesPlot(1), 'FaceColor', obstacleColor);
%            set(self.obstaclesPlot(2), 'FaceColor', obstacleColor);
            
        end
    end
    
end

%%
function [flxx, flyy, frxx, fryy] = local_get_tire_points(x, y, h, sa, p)

MAX_STEERING_DEG = 25;

sa   = max(sa, -MAX_STEERING_DEG);
sa   = min(sa, MAX_STEERING_DEG);
swa = deg2rad(sa);

h = h + pi/2;
tmp_y = y;
y = x;
x = -tmp_y;

% Front left
x0 = [-p.tireLen/2 -p.tireLen/2 p.tireLen/2 p.tireLen/2]';
y0 = [-p.tireWidth/2 p.tireWidth/2 p.tireWidth/2 -p.tireWidth/2]';
x01 =  x0 * cos(swa) - y0 * sin(swa) + p.truckWheelbase;
y01 =  x0 * sin(swa) + y0 * cos(swa) + p.truckWidth/2;
flxx =  x01 * cos(h) - y01 * sin(h) + x;
flyy =  x01 * sin(h) + y01 * cos(h) + y;

% Front right
x02 =  x0 * cos(swa) - y0 * sin(swa) + p.truckWheelbase;
y02 =  x0 * sin(swa) + y0 * cos(swa) - p.truckWidth/2;
frxx =  x02 * cos(h) - y02 * sin(h) + x;
fryy =  x02 * sin(h) + y02 * cos(h) + y;
end

function [xx, yy] = local_get_truck_points(x,y,h, p)
h = h + pi/2;
tmp_y = y;
y = x;
x = -tmp_y;

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
h = h + pi/2;
tmp_y = y;
y = x;
x = -tmp_y;

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