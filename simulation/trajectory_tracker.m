classdef trajectory_tracker < handle
    properties
        taskMgnr = taskManager();
        vehicle
        sampleTime = 0.1;
        speedController
        steeringController
        trajectory_ref
        nearestWP_idx = 1;
        trajectory_actual = zeros(0,6);
    end
    
    methods
        function self = trajectory_tracker(vhcl)
            self.vehicle = vhcl;
            self.speedController = simpleSpeedControl(vhcl);
            self.steeringController = lqrSteeringControl(vhcl); 
            self.steeringController.reset();
        end
        
        function set_ref_trajectotry(self, traj)
            self.trajectory_ref = traj;            
        end
        
        function track(self)
            for t = 0:self.sampleTime:1000
                track_one_step(self);
                self.trajectory_actual = ...
                    [self.trajectory_actual;
                     self.vehicle.get_state' self.vehicle.get_steering_angle ...
                     self.vehicle.v_mps];
            end
        end
        
        function track_one_step(self)            
            compute_neareset_waypoint(self);  
            wp = self.trajectory_ref.Data(self.nearestWP_idx, :);
            d_m = get_distance_left(self);
            fprintf("State: %.2f, %.2f, %.2f \n", self.vehicle.x_m, self.vehicle.y_m, self.vehicle.s_deg);
            fprintf("Tracking waypoint #%d of %d\n", self.nearestWP_idx, self.trajectory_ref.numWayPoints);
            fprintf("Distance left %.2f \n", d_m);            
            sw = self.steeringController.compute(wp);            
            vm = self.speedController.compute(wp(end), d_m);
            self.vehicle.set_steering(sw);
            self.vehicle.set_speed(vm);
            self.vehicle.drive();            
        end
        
        function wpIdx = compute_neareset_waypoint(self)
            dist = self.trajectory_ref.Data(:,1:2) - ...
                [self.vehicle.x_m self.vehicle.y_m];
            dist = dist(:,1).^2 + dist(:,2).^2; 
            [~, wpIdx] = min(dist);
            self.nearestWP_idx = wpIdx;
        end
        
        function d_m = get_distance_left(self)
            xyhg = self.vehicle.get_state();
            wp = self.trajectory_ref.Data(self.nearestWP_idx, :);
            dxyh = compute_tracking_err_at_target(xyhg(1:3), wp(1:3));
            % add to get distance left
            
            d1 = self.trajectory_ref.cumDistLeft(self.nearestWP_idx);
            d2 = dxyh(1);
            if self.taskMgnr.taskType == 1
                d_m = d1 - d2;
            else
                d_m = d1 + d2;
            end
        end
    end
end