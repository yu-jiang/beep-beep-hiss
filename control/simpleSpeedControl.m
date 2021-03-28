classdef simpleSpeedControl < handle
    properties
        direction = 1; % 1 forward, -1 backward
        threshold = 0.1;
    end
    
    methods
        function self = simpleSpeedControl()            
        end
        
        function spd_cmd = compute(self, spd, ref_spd, distance_left)
            spd_cmd = feedback(self, spd, ref_spd);
            if distance_left <= self.threshold
                spd_cmd = 0;
            end
        end
        
        function y = feedback(self, cur_sped, ref_spd)
            y = 0.5*abs(ref_spd) + 0.5*abs(cur_sped);
        end
    end
end