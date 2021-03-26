classdef simpleSpeedControl < handle
    properties
        vehicle
        threshold = 0.1;
    end
    
    methods
        function self = simpleSpeedControl(vehicle)
            self.vehicle = vehicle;
        end
        
        function spd_cmd = compute(self, ref_spd, distance_left)
            spd_cmd = feedback(self, ref_spd);
            if distance_left <= self.threshold
                spd_cmd = 0;
            end
        end
        
        function y = feedback(self, ref_spd)
            y = 0.5*abs(ref_spd) + 0.5*abs(self.vehicle.v_mps);
        end
    end
end