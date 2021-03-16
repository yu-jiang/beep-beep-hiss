classdef trajectory_tracker < handle
    properties
        vehicle
        trajectory
    end
    
    methods 
        function this = trajectory_tracker(vhcl, trjc)
            this.vehicle = vhcl;
            this.trajectory = trjc;
        end
        
        function track(this)
        end
    end
end