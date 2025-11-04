classdef TransitionModel < handle
    methods (Abstract)
        % 计算状态预测x_pred、状态转移矩阵F、过程噪声Q
        % 输入：x_prev（上一时刻状态）、u（传感器输入，如IMU数据）、dt（时间差）
        % 输出：x_pred（预测状态）、F（Phi矩阵）、Q（过程噪声协方差）
        [x_pred, F, Q] = computePrediction(obj, x_prev, u, dt);
    end
end