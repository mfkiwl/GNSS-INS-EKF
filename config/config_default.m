function cfg = config_default()
% 1) 初始状态与协方差（向量化状态：x = [pos; vel; ba; (可选 cb)]）
cfg.x0 = [zeros(3,1); zeros(3,1); zeros(3,1)];   % 示例: [pos; vel; ba]
cfg.P0 = blkdiag(100^2*eye(3), 1^2*eye(3), 0.01^2*eye(3));

% 2) 过程噪声 Q 的计算参数（由过程模型离散化成 Qd）
cfg.Q_pos = (1e-3)^2*eye(3);
cfg.Q_vel = (1e-2)^2*eye(3);
cfg.Q_ba  = (1e-5)^2*eye(3);

% 3) R 阵的计算参数（文章中未直接给 R；这里给出可配置的权重参数）
%   3.1 松耦合（位置）—— R = (HDOP * UERE)^2 I
cfg.lc.UERE = 10;  % m
cfg.lc.HDOP_default = 1.0;

%   3.2 紧耦合（伪距）—— 对角阵 R_i：按仰角/信噪比衰减
cfg.tc.UERE = 3;    % 基础误差（可按设备/SV系统区分）
cfg.tc.elev_weight.enable = true;
cfg.tc.elev_weight.min_el = deg2rad(5);
cfg.tc.elev_weight.func   = @(el) 1./max(sin(el), 0.1);  % 仰角越低，方差越大
cfg.tc.cn0_weight.enable  = true;
cfg.tc.cn0_weight.k       = 0.05;  % 简单 1/(1+k*exp(-CN0)) 形式可自定义

% 4) 其他
cfg.dt_imu = 0.01;
end