classdef gnss_loose < handle
% LC：z 为 GNSS 位置量测（ECEF/ENU任选，但要与状态一致）
% 文章思路：R = (HDOP * UERE)^2 I

    methods
        function [r, H, R] = linearize(~, x, z, cfg)
            pos = x(1:3);
            r   = z - pos;                 % r = z - h(x) = z - pos
            H   = [eye(3), zeros(3,6)];    % 对 [pos; vel; ba] 的雅可比
            HDOP = cfg.lc.HDOP_default;    % 若你从 RTKLIB 读到 HDOP，这里替换
            sigma2 = (HDOP * cfg.lc.UERE)^2;
            R   = sigma2 * eye(3);
        end
    end
end
