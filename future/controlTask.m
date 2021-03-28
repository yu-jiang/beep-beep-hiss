classdef controlTask < handle
    properties
        taskType % 1 forward without trailer, 2 forward with trailer
                 % 3 backward without trailer , 4 backward with trailer
        taskTrajectory
        tasksStatus = 1; % 1 executing, 0 stopped
        distanceLeft = 0;
        debugInfo
        
        % termination condition
        terminationCondition
        
        % tracking error
        nearestWP_idx = 1;
        trackingErr
        
        speedController
        steeringController
        
        vehicle        
    end
    
    methods
        function self = controlTask(vhcl, ...
                                    type, ...
                                    traj, ...
                                    spdCtrl, ...
                                    strCtrl, ...
                                    termCond)
            self.vehicle = vhcl;
            self.taskType = type;
            self.taskTrajectory = traj;
            self.speedController = spdCtrl;
            self.steeringController = strCtrl;
            self.terminationCondition = termCond;
                        
            updateStatus(self);
        end
        
        function updateStatus(self)
            compute_tracking_err(self);  
            if self.trackingErr.distanceLeft >= ...
                    self.terminationCondition.distanceLeft
                self.tasksStatus = 1;
            else
                self.tasksStatus = 0;
            end
        end
        
        function execute(self, dt)
            if nargin < 2
                dt = 0.1;
            end
            
            updateStatus(self);
            fprintf("State: %.2f, %.2f, %.2f \n", ...
                self.vehicle.x_m, ...
                self.vehicle.y_m, ...
                self.vehicle.s_deg);
            fprintf("Tracking waypoint #%d of %d\n", ...
                self.nearestWP_idx, ...
                self.taskTrajectory.numWayPoints);
            fprintf("Distance left %.2f \n", ...
                self.trackingErr.distanceLeft);
            
            if self.tasksStatus == 1
                if self.vehicle.shifting == 0
                    switch self.taskType
                        case {1,2}
                            self.vehicle.shifting = 1;
                        case {3,4}
                            self.vehicle.shifting = -1;
                    end
                else
                    wp = self.taskTrajectory.Data(self.nearestWP_idx, :);                    
                    sw = self.steeringController.compute(self.vehicle.get_state, wp);
                    vm = self.speedController.compute(self.vehicle.spd_mps, wp(end), ...
                        self.trackingErr.distanceLeft);
                    self.vehicle.set_steering(sw);
                    self.vehicle.set_speed(vm);
                    self.vehicle.drive(dt);
                end
            else
                % Task is done
                if self.vehicle.shifting ~= 0 && self.vehicle.spd_mps == 0
                    self.vehicle.shifting = 0;
                end
            end
        end
    end
    
    methods (Access = private)
        function compute_tracking_err(self)
            compute_neareset_waypoint(self);
            self.trackingErr.distanceLeft = get_distance_left(self);
        end
        
        function wpIdx = compute_neareset_waypoint(self)
            dist = self.taskTrajectory.Data(:,1:2) - ...
                [self.vehicle.x_m self.vehicle.y_m];
            dist = dist(:,1).^2 + dist(:,2).^2;
            [~, wpIdx] = min(dist);
            self.nearestWP_idx = wpIdx;
        end
        
        function d_m = get_distance_left(self)
            xyhg = self.vehicle.get_state();
            wp = self.taskTrajectory.Data(self.nearestWP_idx, :);
            dxyh = compute_tracking_err_at_target(xyhg(1:3), wp(1:3));
            % add to get distance left
            
            d1 = self.taskTrajectory.cumDistLeft(self.nearestWP_idx);
            d2 = dxyh(1);
            if self.taskType == 1 || self.taskType == 2
                d_m = d1 - d2;
            elseif self.taskType == 3 || self.taskType ==  4
                d_m = d1 + d2;
            end
        end
    end
    
end