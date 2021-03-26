classdef truck_trailer_trajectory < handle
    properties
        Data
        cumDistLeft
        numWayPoints
    end
    
    methods
        function self = truck_trailer_trajectory(trajData)
            % trajData is an n by 6 matrix
            self.Data = trajData;
            compute_cumDistLeft(self);
        end
        
        function compute_cumDistLeft(self)
            [n, ~] = size(self.Data);
            self.cumDistLeft = zeros(n,1);
            for ct = n-1:-1:1
                self.cumDistLeft(ct) = ...
                    norm(self.Data(ct+1,1:2) - self.Data(ct,1:2)) ...
                    + self.cumDistLeft(ct+1);
            end
            self.numWayPoints = n;
        end
    end
end