%%
wp = [1 0 0 0 0 1
      5 1 0 0 0 1];

tt = truckTrailerOnAxle();
tckr = trajectory_tracker(tt);
trj = truck_trailer_trajectory(wp);
tckr.set_ref_trajectotry(trj);
tckr.track()