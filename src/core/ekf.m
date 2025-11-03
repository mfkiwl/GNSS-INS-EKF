classdef ekf < handle
% 极简 EKF（不做 SPD/数值检查；完全按公式）
% x, P, predict(), update()；可配不同的过程/观测模型类

    properties
        x   % 列向量
        P   % 协方差
        cfg % 配置
    end

    methods
        function obj = ekf(cfg)
            obj.cfg = cfg;
            obj.x = cfg.x0(:);
            obj.P = cfg.P0;
        end

        function predict(obj, procModel, u, dt)
            % procModel: 实现 [xpred, Phi, Qd] = propagate(x,u,dt,cfg)
            [xpred, Phi, Qd] = procModel.propagate(obj.x, u, dt, obj.cfg);
            obj.x = xpred;
            obj.P = Phi*obj.P*Phi' + Qd;
        end

        function update(obj, measModel, z)
            % measModel: 实现 [r,H,R] = linearize(x,z,cfg)
            % 约定 r = z - h(x) （正残差）
            [r, H, R] = measModel.linearize(obj.x, z, obj.cfg);

            S = H*obj.P*H' + R;       % 创新
            K = (obj.P*H') / S;       % 增益（简洁起见，直接右除）
            dx = K * r;               % 状态修正
            obj.x = obj.x + dx;

            I = eye(size(obj.P));
            obj.P = (I - K*H) * obj.P * (I - K*H)' + K*R*K';  % Joseph 形式
        end
    end
end