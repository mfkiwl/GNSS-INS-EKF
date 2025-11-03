classdef imu < handle
% 过程模型（示例：简单的 [pos; vel; ba] 模型）
% 可按需要换成更高保真（重力、坐标系转换、姿态驱动等）

    methods
        function [xpred, Phi, Qd] = propagate(~, x, u, dt, cfg)
            % 状态: x = [pos(3); vel(3); ba(3)]
            pos = x(1:3); vel = x(4:6); ba = x(7:9);

            a_m = u.am;             % 加速度计测量（假设已是导航系/已去重力，仅示例）
            a   = a_m - ba;

            pos_n = pos + vel*dt;
            vel_n = vel + a*dt;
            ba_n  = ba;

            xpred = [pos_n; vel_n; ba_n];

            % 线性化（常速模型）
            Phi = eye(9);
            Phi(1:3,4:6) = eye(3)*dt;

            % Q 离散（示例：直接用配置块对角）
            Qd = blkdiag(cfg.Q_pos, cfg.Q_vel, cfg.Q_ba);
        end
    end
end
