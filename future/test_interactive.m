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

xsave = zeros(0,6);

while ct.tasksStatus == 1
    ct.execute()
    
    xx = [ct.vehicle.get_state()' ct.vehicle.get_steering_angle() ct.vehicle.v_mps];
    xsave = [xsave; xx];
end

