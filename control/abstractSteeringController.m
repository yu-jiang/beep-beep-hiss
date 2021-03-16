classdef (Abstract) abstractSteeringController < handle
    properties
        controlSig = 0;
    end
    
    methods
        function y = getSteeringCommand(self)
            y = self.controlSig;
        end
    end

    methods (Abstract)
        compute(self)        
    end
end