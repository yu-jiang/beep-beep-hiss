%% Setup vehicle
p = get_default_truck_trailer_params();
p.noiseLevel = 0.01;
p.trailerWheelbase = 11;
forwardTarget = [20 2 0.3];

% compute optimal gain
A = [  0    -1     0;
    0     0     0;
    0     0    1/p.trailerWheelbase];
B = [0;
    -1/p.truckWheelbase;
    1/p.truckWheelbase];
p.feedbackGain = lqr(A, B, p.Q, p.R);

xx = [-2 0 0 0] + [ 20 0 0 0; 
       20 -1 0 0; 
      20 0 0.3 -0.3;   
      20 0 0 0.03;
      20 2.3 -0.15 0.24;
      20 3.2 0.5 -0.3;
      ];
%xx = [20 -2 0.3 -0.35];
opt = odeset('Events', @obstacleEvents);
dt = 0.2;

% setup viz
[nSim, ~] = size(xx);
nRows = floor(sqrt(nSim));
nCols = ceil(nSim/nRows);
figure(101)

for ct = 1:nSim
    ax(ct) = subplot(nRows, nCols, ct);
    tp(ct) = truck_trailer_plot_simple(p, ax(ct));
    tp(ct).updateFig(xx(ct,:))
end
%% No learning
clear Ts Ys
for ct = 1:nSim
    tsave = 0;
    ysave = xx(ct,:);
    
    %backward
    p.velocity = -1;
    [t,y] = ode45(@(t, x)sys_truck_trailer_wrapper(t, x, p), ...
        tsave(end)+[0 40], [ysave(end,1:4) zeros(1,10)], opt);
    
    tsave = [tsave; t];
    ysave = [ysave; y(:,1:4)];
    
    
    ts = (tsave(1):dt:tsave(end))';
    [~, ix] = unique(tsave);
    ys = interp1(tsave(ix), ysave(ix,:), ts, 'linear', 'extrap');
    
    Ts{ct} = ts;
    Ys{ct} = ys;
end

% Parallel plot
%v = VideoWriter('newfile.mp4','MPEG-4');
%open(v)
create_gif = 1;
h = figure(101);
if create_gif == 1
    filename = 'convergence.gif';
end
for jj = 1:numel(Ts{ct})+20
    for ct = 1:nSim
        if jj <= numel(Ts{ct})
        tp(ct).updateFig(Ys{ct}(jj,:))
        end
    end
    
    drawnow
    % Write to the GIF File
    if create_gif == 1
        frame = getframe(h);
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        if jj == 1
            imwrite(imind,cm,filename,'gif', 'Loopcount',inf, 'DelayTime',2);
        else
            imwrite(imind,cm,filename,'gif','WriteMode','append', 'DelayTime',0.05);
        end
    end
end
