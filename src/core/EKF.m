classdef EKF < FilterBase
    methods (Access = public)
        function obj = EKF(x0, P0, state_model, obs_model)
            obj@FilterBase(x0, P0, state_model, obs_model);
        end
        
        function predict(obj, u, dt)
            [x_pred, F, Q] = obj.state_model.computePrediction(obj.x, u, dt);
            
            obj.x = x_pred;
            obj.P = F * obj.P * F' + Q;
        end
        
        function update(obj, z_meas)
            [z_pred, H, R] = obj.obs_model.computeObservation(obj.x, z_meas);
            r = z_meas - z_pred;
            S = H * obj.P * H' + R;
            K = obj.P * H' / S;
            
            obj.x = obj.x + K * r;
            obj.P = (eye(size(obj.P, 1)) - K * H) * obj.P;
        end
    end
end