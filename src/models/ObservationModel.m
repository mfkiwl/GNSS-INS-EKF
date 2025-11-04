classdef ObservationModel < handle
    methods (Abstract)
        % 计算预测观测值z_pred、观测矩阵H、观测噪声R
        % 输入：x_pred（预测状态）、z_meas（实测观测值，用于辅助计算R）
        % 输出：z_pred（预测观测值）、H（观测矩阵）、R（观测噪声协方差）
        [z_pred, H, R] = computeObservation(obj, x_pred, z_meas);
    end
end