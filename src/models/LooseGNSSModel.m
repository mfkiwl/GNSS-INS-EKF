classdef LooseGNSSModel < ObservationModel
    properties (Private)
        gnss_params;
    end
    
    methods
        function obj = GNSSModel(gnss_params)
            obj.gnss_params = gnss_params;
        end

        function [z_pred, H, R] = computeObservation(obj, x_pred, ~)
            z_pred = x_pred(1:3);
            H = [eye(3), zeros(3, 6)];
            % R = (HDOP * UERE)^2 I
            k = obj.s_UERE * obj.h_pdop;
            R = k^2 * eye(3);
        end
    end
end
