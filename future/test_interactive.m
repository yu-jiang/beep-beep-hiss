tt = truckTrailerOnAxle();

taskType = 2; 
vehicle = tt;
speedController = simpleSpeedControl();
[A,B] = tt.get_forward_AB();
steeringController = lqrSteeringControl(A,B);
wps = [1 0 0 0 0 1; 5 1 0 0 0 1];
taskTrajectory = truck_trailer_trajectory(wps);
termCond.distanceLeft = 0.1;

ct = controlTask(vehicle, ...
    taskType, ...
    taskTrajectory, ...
    speedController, ...
    steeringController, ...
    termCond);

