classdef truck_trailer_on_axle < handle
    % truck and trailer model where the fifth wheel is at the rear center
    % of the truck, and it is also the reference point of the model.    
    properties
        % Params
        maxSteeringDeg     = 25;
        truckWheelbase     = 3.0;
        trailerWheelbase   = 11.0;
        
        % Dynamics
        v_mps              = 1;
        x_m                = 0;
        y_m                = 0;
        h_rad              = 0;
        g_rad              = 0; 
        s_deg              = 0; % Steering angle  
        shifting           = 1;
        
        % Visualization
        truckWidth         = 3;
        truckMargin        = 1;
        trailerWidth       = 2.8;
        trailerMarginFront = 1;
        trailerMarginBack  = 3;
        tireLen            = 1.2;
        tireWidth          = 0.3;       
    end
    
    methods
        function self = truck_trailer_on_axle()
        end
        
        function set_state(self, xyhg)
            self.x_m   = xyhg(1);
            self.y_m   = xyhg(2);
            self.h_rad = xyhg(3);
            self.g_rad = xyhg(4);
        end
        
        function set_input(self, swa_deg)
            swa_deg = min(swa_deg,  self.maxSteeringDeg);
            swa_deg = max(swa_deg, -self.maxSteeringDeg);
            self.s_deg = swa_deg;
        end
        
        function swa = get_steering_angle(self)
            swa = deg2rad(self.s_deg);
        end
        
        function state = get_state(self)
            state = [self.x_m; self.y_m; self.h_rad; self.g_rad];
        end
        
        function drive(self, dt)
            if nargin < 2
                dt = 0.1;
            end
            swa = self.get_steering_angle();
            d_state = [cos(self.h_rad);
                sin(self.h_rad);
                tan(swa)/self.truckWheelbase;
                -sin(self.g_rad)/self.trailerWheelbase - tan(swa)/self.truckWheelbase];
            
            old_state = self.get_state();
            new_state = old_state + dt * self.v_mps * d_state;
            self.set_state(new_state);
        end
        
    end
end