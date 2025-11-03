classdef gnss_tight < handle
% TC：一历元多星时，每颗星独立调用本模型，然后把 (r,H,R) 逐行堆叠到一起
% 量测：z_i = 伪距_i
% 模型：h_i(x) = ||pos - rs_i|| + c*cb   （此处示例未包含钟差，可自行扩展）

    methods
        function [r, H, R] = linearize(~, x, z_one, cfg, sat)
            % x: [pos; vel; ba ...]（此处只用到 pos）
            % z_one: 标量伪距
            % sat: struct，至少包含 sat.rs(3x1) 卫星 ECEF 位置、sat.elev 仰角（rad）、sat.cn0 (dBHz)
            pos = x(1:3);
            rho = norm(pos - sat.rs);
            h   = rho;                      % 若扩展钟差： h = rho + c*cb
            r   = z_one - h;                % 标量残差

            % 雅可比：∂h/∂pos = (pos - rs)/||pos-rs||
            if rho < 1e-8, los = [1;0;0]; else, los = (pos - sat.rs)/rho; end
            Hpos = los';                    % 1x3
            H    = [Hpos, zeros(1,6)];      % 对 [pos; vel; ba]

            % 方差：按仰角/信噪比加权（对角）
            sigma2 = cfg.tc.UERE^2;
            if cfg.tc.elev_weight.enable && isfield(sat,'elev')
                w_el  = cfg.tc.elev_weight.func( max(sat.elev, cfg.tc.elev_weight.min_el) );
                sigma2 = sigma2 * w_el^2;
            end
            if cfg.tc.cn0_weight.enable && isfield(sat,'cn0')
                w_cn0 = 1/(1 + cfg.tc.cn0_weight.k*exp(-sat.cn0));
                sigma2 = sigma2 * (1/w_cn0)^2;
            end
            R = sigma2;   % 标量
        end
    end
end
