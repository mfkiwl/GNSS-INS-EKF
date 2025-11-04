classdef IMUModel < TransitionModel
    properties (Private)
        imu_params;
    end
    
    methods
        function obj = IMUModel(imu_params)
            obj.imu_params = imu_params;
        end
        
        function [x_pred, F, Q] = computePrediction(obj, x_prev, imu_data, dt)
            p_prev = x_prev(1:3);
            v_prev = x_prev(4:6);
            ba_prev = x_prev(7:9);
            
            % 补偿IMU加速度偏置
            a_B = imu_data.acc - ba_prev;
            R_GB = computeRotation(p_prev(1), p_prev(2), p_prev(3), imu_data(1), imu_data(2), imu_data(3), imu_data(4));
            a_G = R_GB * a_B;
            
            % 状态预测（欧拉积分）
            p_pred = p_prev + v_prev * dt;  % 位置更新
            v_pred = v_prev + a_G * dt;   % 速度更新
            ba_pred = ba_prev;
            
            % 输出
            x_pred = [p_pred; v_pred; ba_pred];
            F = obj.computeF(x_prev, a_G, dt);
            Q = obj.computeQ(x_prev);
        end
    end

    methods (Access = private)
        function R_GB = computeRotation(x_ecef, y_ecef, z_ecef, q_x, q_y, q_z, q_w)
            % R_GB = R_GL * R_LB
            % 输入：
            %   x_ecef, y_ecef, z_ecef - ECEF坐标 (m)
            %   q_x, q_y, q_z, q_w     - ROS orientation四元数分量（单位四元数）
            % 输出：
            %   R_GB                   - 3×3矩阵，Body→ECEF的转换矩阵
            
            % 计算R_GL矩阵
            a = 6378137.0;
            f = 1/298.257223563;
            e2 = 2*f - f^2;

            lon = atan2(y_ecef, x_ecef);
            rho = sqrt(x_ecef^2 + y_ecef^2);
            lat = atan2(z_ecef, rho);
            tol = 1e-8;
            for i = 1:10
                sin_lat = sin(lat);
                N = a / sqrt(1 - e2 * sin_lat^2);
                lat_new = atan2(z_ecef + N*e2*sin_lat, rho);
                if abs(lat_new - lat) < tol, lat = lat_new; break; end
                lat = lat_new;
            end

            sin_lon = sin(lon);
            cos_lon = cos(lon);
            sin_lat = sin(lat);
            cos_lat = cos(lat);
            R_GL = [
                -sin_lon, -sin_lat*cos_lon,  cos_lat*cos_lon;
                cos_lon,  -sin_lat*sin_lon,  cos_lat*sin_lon;
                0,        cos_lat,           sin_lat
            ];
            
            % R_LB
            q = [q_w, q_x, q_y, q_z];  % [w, x, y, z]
            q = q / norm(q);
            w = q(1); x = q(2); y = q(3); z = q(4);

            R_LB = [
                1-2*y^2-2*z^2,   2*x*y - 2*z*w,   2*x*z + 2*y*w;
                2*x*y + 2*z*w,   1-2*x^2-2*z^2,   2*y*z - 2*x*w;
                2*x*z - 2*y*w,   2*y*z + 2*x*w,   1-2*x^2-2*y^2
            ];
            
            % 计算R_GB
            R_GB = R_GL * R_LB;
        end

        function F = computeF(x_prev, a_ecef, dt)
            n = length(x_prev);
            F = eye(n);
            F(1:3, 4:6) = eye(3) * dt;
            F(4:6, 7:9) = diag(a_ecef * dt);
        end
        
        function Q = computeQ(x_prev)
            n = length(x_prev);
            Q = eye(n);
        end
    end
end
