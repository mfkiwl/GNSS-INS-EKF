classdef FilterBase < handle
    % 滤波器基类：定义统一接口，封装状态和模型依赖，支持EKF/IEKF等扩展
    
    % ------------------------------
    % 保护属性：状态和模型（子类可访问）
    % ------------------------------
    properties (Access = protected)
        x;              % 状态向量（内部维护，维度由模型和配置决定）
        P;              % 状态协方差矩阵（内部维护）
        state_model;    % 状态转移模型（必须实现TransitionModel接口）
        obs_model;      % 观测模型（必须实现ObservationModel接口）
    end
    
    % ------------------------------
    % 抽象方法：子类必须实现具体逻辑
    % ------------------------------
    methods (Abstract, Access = public)
        % 预测步骤：基于传感器输入更新状态和协方差
        % 输入：u - 状态转移输入（如IMU数据）；dt - 时间间隔
        predict(obj, u, dt);
        
        % 更新步骤：基于观测值更新状态和协方差
        % 输入：z_meas - 观测数据（如GNSS伪距/位置）
        update(obj, z_meas);
    end
    
    % ------------------------------
    % 公共方法：通用功能（状态管理）
    % ------------------------------
    methods (Access = public)
        % 构造函数：注入模型实例（依赖接口，不依赖具体实现）
        function obj = FilterBase(x0, P0, state_model, obs_model)
            % 校验模型是否实现接口
            if ~isa(state_model, 'TransitionModel')
                error('状态模型必须实现TransitionModel接口');
            end
            if ~isa(obs_model, 'ObservationModel')
                error('观测模型必须实现ObservationModel接口');
            end
            % 初始化状态
            obj.reset(x0, P0);
            % 绑定模型（通过接口约束，支持任意实现类）
            obj.state_model = state_model;
            obj.obs_model = obs_model;
        end
        
        % 重置状态（初始化或重启动时使用）
        function reset(obj, x0, P0)
            % 基本校验：维度匹配且P0正定
            if length(x0) ~= size(P0, 1)
                error('初始状态x0与协方差P0维度不匹配');
            end
            if ~isposdef(P0)
                error('协方差P0非正定');
            end
            obj.x = x0;
            obj.P = P0;
        end
        
        % 获取当前状态（只读，供外部读取结果）
        function [x, P] = getState(obj)
            x = obj.x;
            P = obj.P;
        end
    end
end