classdef lqrSteeringControl < abstractSteeringController
    properties
        Q
        R               
    end
    
    methods
        function self = lqrSteeringControl()
        end
        
        function reset(self)
        end
        
        function y = getSteeringCommand(self)
        end
        
        function compute(self)
        end
    end
end