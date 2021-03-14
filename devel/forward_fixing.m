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
t = gen_forward_fix_traj(p, [0 1 0 0], 20);

figure(102)
ax = gca;
tp = truck_trailer_plot_simple(p, ax);

[nSteps,~] = size(t);
for ct = 1:nSteps
    tp.updateFig(t(ct,:))    
    drawnow
end
