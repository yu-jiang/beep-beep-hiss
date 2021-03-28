classdef lqrSteeringControl < abstractSteeringController
    properties
        A = zeros(3);
        B = zeros(3,1);
        Q = [1 0 0; 
             0 1 0; 
             0 0 1];           
        R = 100;
        K = [0 0 0];        
        err = [0;0;0];
        
        vehicle
    end
    
    methods
        function self = lqrSteeringControl(A, B)
            self.A = A;
            self.B = B;
            reset(self);
        end
        
        function reset(self)
            self.K = lqr(self.A, self.B, self.Q, self.R);
        end
                
        function y = compute(self, ego_state, waypoint)
            xyhg = ego_state;
            dxyh = compute_tracking_err_at_target(xyhg(1:3), waypoint(1:3));
            dg   = xyhg(4) - waypoint(4);
            self.err = [dxyh; dg];
            self.controlSig = -self.K * self.err(2:4) + waypoint(6);
            y = self.getSteeringCommand();
        end
        
    end
end