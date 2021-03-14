%% Setup vehicle
p = get_default_truck_trailer_params();
p.noiseLevel = 0.01;
p.trailerWheelbase = 11;
forwardTarget = [20 2 0.3];

%ic = [0 3 0 0];
ic = [0 1 -0.2 0.3];

% compute optimal gain
A = [  0    -1     0;
       0     0     0;
       0     0    1/p.trailerWheelbase];
B = [0;
    -1/p.truckWheelbase;
    1/p.truckWheelbase];
p.feedbackGain = lqr(A, B, p.Q, p.R);
t = gen_forward_fix_traj(p, ic, 20);

figure(102)
ax(1) = subplot(121);
tp(1) = truck_trailer_plot_simple(p, ax(1));
tp(1).updateFig(ic);
title('Straightline Tracking')
ax(2) = subplot(122);
tp(2) = truck_trailer_plot_simple(p, ax(2));
tp(2).updateFig(ic);
title('Control-based Planning')

%% Straight line
numPullups = 3;

tsave = 0;
ysave = ic;

for ct = 1:numPullups

p.forwardTarget = [20 0 0];


p.velocity = 1;
[t,y] = ode45(@(t, x)sys_truck_trailer_wrapper(t, x, p), ...
    tsave(end)+[0 40], [ysave(end,1:4) zeros(1,10)], opt);

tsave = [tsave; t];
ysave = [ysave; y(:,1:4)];

%backward
p.velocity = -1;
[t,y] = ode45(@(t, x)sys_truck_trailer_wrapper(t, x, p), ...
    tsave(end)+[0 40], [ysave(end,1:4) zeros(1,10)], opt);

tsave = [tsave; t];
ysave = [ysave; y(:,1:4)];

end


ts = (tsave(1):dt:tsave(end))';
[~, ix] = unique(tsave);
ys = interp1(tsave(ix), ysave(ix,:), ts, 'linear', 'extrap');
ts1 = ts;
ys1 = ys;

%% With Control-based planning
numPullups = 1;

tsave = 0;
ysave = ic;

for ct = 1:numPullups

p.forwardTarget = [20 0 0];


p.velocity = 1;


trj = gen_forward_fix_traj(p, ysave(end,1:4), p.forwardTarget(1)); 
[nwp, ~] = size(trj);
t = tsave(end)+ (0:nwp-1)*0.1;

tsave = [tsave; t(:)];
ysave = [ysave; trj(:,1:4)];

%backward
p.velocity = -1;
[t,y] = ode45(@(t, x)sys_truck_trailer_wrapper(t, x, p), ...
    tsave(end)+[0 40], [ysave(end,1:4) zeros(1,10)], opt);

tsave = [tsave; t];
ysave = [ysave; y(:,1:4)];

end


ts = (tsave(1):dt:tsave(end))';
[~, ix] = unique(tsave);
ys = interp1(tsave(ix), ysave(ix,:), ts, 'linear', 'extrap');
ts2 = ts;
ys2 = ys;
%% Parallel plot
%v = VideoWriter('newfile.mp4','MPEG-4');
%open(v)
h = gcf;
%axis tight manual % this ensures that getframe() returns a consistent size
filename = 'temp.gif';
for jj = 1:numel(ts1)
    tp(1).updateFig(ys1(jj,:))
    if jj <= numel(ts2)
        tp(2).updateFig(ys2(jj,:))
    end
    drawnow
    % Write to the GIF File
    frame = getframe(h);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if jj == 1
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf, 'DelayTime',0.5);
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append', 'DelayTime',0.1);
    end
end
%close(v)

%%
figure(1)


