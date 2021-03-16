classdef simpleSpeedControl < handle
    properties
        vehicle
    end
    
    methods
        function self = simpleSpeedControl(vehicle)
            self.vehicle = vehicle;
        end
        
        function vm = compute(self, waypoint)
             xyhg            = self.vehicle.get_state();
             currentVelociy  = self.vehicle.get_velocity();
             dxyh = compute_tracking_err_at_target(xyhg(1:3), waypoint(1:3));    
             if (dxyh(1) >0 && currentVelociy>0) || (dxyh(1) <0 && currentVelociy<0)
                 vm = 0;
             else
                 vm = 1;
             end                 
        end
    end
end