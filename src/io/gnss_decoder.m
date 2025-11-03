classdef gnss_decoder < handle
    methods
        function gnssRaw = loadLoose(~, obsFile, navFile)
            % TODO: 使用 MatRTKLIB 实现
            % 这里用 mock 占位
            N = 1000;
            for k=1:N
                gnssRaw(k).t = k*0.1;
                gnssRaw(k).pos = [0.1*k; 0.0; 0.0] + 0.5*randn(3,1);
                gnssRaw(k).h_pdop = 1.0 + 0.1*rand();
            end
        end

        function gnssRaw = loadTight(~, obsFile, navFile)
            % TODO: 使用 MatRTKLIB 解析多星几何与伪距
            N = 1000;
            for k=1:N
                gnssRaw(k).t = k*0.1;
                M = 8; % 每历元8颗星（示例）
                gnssRaw(k).sats = repmat(struct('rs',[20200000;0;0],'elev',deg2rad(45),'cn0',45), 1, M);
                gnssRaw(k).pr   = 2.02e7 + randn(M,1); % 粗略伪距
            end
        end
    end
end
