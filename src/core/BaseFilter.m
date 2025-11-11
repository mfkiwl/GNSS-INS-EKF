classdef BaseFilter < handle
    % 滤波器基类
    properties (Access = protected)
        x;              % 状态向量（维度由模型和配置决定）
        P;              % 状态协方差矩阵
        state_model;    % 状态转移模型（必须实现TransitionModel接口）
        obs_model;      % 观测模型（必须实现ObservationModel接口）
    end
    
    % 接口方法
    methods (Abstract, Access = public)
        % 预测步；输入：u-系统输入（如IMU数据）；dt-两个epoch的时间间隔
        predict(obj, u, dt);
        
        % 更新步；输入：meas-观测数据（如GNSS数据）；z-观测量（如伪距）
        update(obj, meas, z);
    end
    

    % 通用功能
    methods (Access = public)
        function obj = BaseFilter(x0, P0, state_model, obs_model)
            % 校验模型是否实现接口
            if ~isa(state_model, 'TransitionModel')
                error('状态模型必须实现TransitionModel接口');
            end
            if ~isa(obs_model, 'ObservationModel')
                error('观测模型必须实现ObservationModel接口');
            end
            % 初始化状态
            obj.reset(x0, P0);
            obj.state_model = state_model;
            obj.obs_model = obs_model;
        end
        
        % 重置状态
        function reset(obj, x0, P0)
            if length(x0) ~= size(P0, 1)
                error('初始状态x0与协方差P0维度不匹配');
            end
            obj.x = x0;
            obj.P = P0;
        end
        
        % 获取当前状态
        function [x, P] = getState(obj)
            x = obj.x;
            P = obj.P;
        end
    end
end