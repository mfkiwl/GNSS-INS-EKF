classdef GNSSData < handle
    % 简洁版 GNSSData：
    % - 紧耦合量测使用 P* = rho + resPc - TGD
    %   其中 resPc 已在 MatRTKLIB 中去除了 dts/ion/trp（不含 TGD）
    % - 钟差统一用"米"单位；EKF 的钟差状态按"米"处理
    % - 默认星座：GPS + BeiDou；可切换为 GPS-only
    % - 提供仰角/信噪比筛选
    %
    % 依赖：MatRTKLIB (gt.Gobs, gt.Gnav, gt.Gsat, gt.Gopt, gt.Gfun)
    properties
        obs    % gt.Gobs
        nav    % gt.Gnav
        xyz    % [N x 3] 参考接收机位置(ECEF, m); 用于仰角/遮罩/权重(可由SPP或预测给)
        sysMode = "GPS"   % "GPS" 或 "GPS+BDS"
        elMaskDeg = 10        % 仰角阈值(度)
        cn0Min   = 20         % C/N0最小阈值(dB-Hz)
    end

    methods
        function obj = GNSSData(obs_file, nav_file)
            obj.obs = gt.Gobs(obs_file);
            obj.nav = gt.Gnav(nav_file);
        end

        function set_sys_mode(obj, mode)
            mode = upper(string(mode));
            if mode ~= "GPS" && mode ~= "GPS+BDS"
                error('sysMode 仅支持 "GPS" 或 "GPS+BDS"');
            end
            obj.sysMode = mode;
        end

        function n = get_epoch(obj)
            n = obj.obs.n;
        end

        function time = get_time_series(obj)
            time = obj.obs.time;
        end

        function z_meas = get_tight_measurement(obj, k)
            % 1) 取该历元观测
            obsc_all = obj.obs.selectTime(k);
        
            % --- 星座筛选 ---
            switch upper(string(obj.sysMode))
                case "GPS"
                    mask = (obsc_all.sys == gt.C.SYS_GPS);
            end
            obsc = obsc_all.selectSat(mask);
        
            % 若筛完没有星，直接返回 skipped
            if obsc.nsat == 0
                z_meas = struct('P',[],'SV',[],'elev',[],'cn0',[],'tgd',[],'sys',[], 'skipped', true);
                return;
            end
        
            % 2) 由筛好的 obsc 生成卫星参数
            sat  = gt.Gsat(obsc, obj.nav);
        
            % 3) 参考位置(只用于仰角/遮罩)
            Xref = pick_xref(obj, k, sat, obsc);         % 见文末的子函数
            haveElev = all(isfinite(Xref));              % 是否可可靠计算仰角
            sat.setRcvPos(gt.Gpos(Xref.', "xyz"));
        
            % 4) 生成 resPc（已去 dts/ion/trp，不含 TGD）
            obsc = obsc.residuals(sat);
        
            % 5) 取原始字段
            SV    = [sat.x; sat.y; sat.z];        % 3×M
            el    = sat.el(:);                    % M×1 (rad)
            Sraw  = obsc.L1.S(:);                 % M×1
            cn0   = Sraw;  if any(cn0>100), cn0 = cn0/4; end
            resPc = obsc.L1.resPc(:);             % M×1 (m)
            tgd   = obj.nav.getTGD(sat.sat(:));   % M×1 (m)
            sys   = sat.sys(:);                   % M×1 (int)
        
            % 6) 仰角/CN0筛选（仰角只在有参考时启用）
            use_idx = ~isnan(SV(1,:)).' & ~isnan(resPc) & (cn0 >= obj.cn0Min);
            if haveElev
                use_idx = use_idx & (el >= deg2rad(obj.elMaskDeg));
            end
        
            SV    = SV(:,use_idx);
            el    = el(use_idx);
            cn0   = cn0(use_idx);
            resPc = resPc(use_idx);
            tgd   = tgd(use_idx);
            sys   = sys(use_idx);
        
            if isempty(resPc)
                z_meas = struct('P',[],'SV',[],'elev',[],'cn0',[],'tgd',[],'sys',[], 'skipped', true);
                return;
            end
        
            % 7) 几何距离 rho（简单范数；高阶修正可后续加）
            d   = SV - Xref;                      
            rho = sqrt(sum(d.^2,1)).';            % M×1
        
            % 8) 构造紧耦合量测：P* = rho + resPc - TGD
            pstar = rho + resPc - tgd';
        
            % 9) 打包输出
            z_meas.P    = pstar;     % (m)
            z_meas.SV   = SV;        % (m)
            z_meas.elev = el;        % (rad)
            z_meas.cn0  = cn0;       % (dB-Hz)
            z_meas.tgd  = tgd;       % (m)
            z_meas.sys  = sys;       % (int)
            z_meas.skipped = false;
        end
        
        % ===== 子函数：选择参考位置（多级兜底） =====
        function Xref = pick_xref(obj, k, sat, obsc)
            % 1) 当前 SPP
            if ~isempty(obj.xyz) && size(obj.xyz,1) >= k && all(isfinite(obj.xyz(k,:)))
                Xref = obj.xyz(k,:).';   return;
            end
            % 2) 近期历史 SPP
            if ~isempty(obj.xyz)
                idx = find(all(isfinite(obj.xyz),2), 1, 'last');
                if ~isempty(idx)
                    Xref = obj.xyz(idx,:).';  return;
                end
            end
            % 3) Bancroft 粗解（仅当该历元可用卫星与伪距足够时）
            P = obsc.L1.P(:);
            SV = [sat.x; sat.y; sat.z];
            ok = isfinite(P) & isfinite(SV(1,:)).';
            if nnz(ok) >= 4
                try
                    [x0, ~] = bancroft_init(SV(:,ok), P(ok));
                    Xref = x0;  return;
                catch
                end
            end
            % 4) 最后兜底：地心；上层将禁用仰角筛选
            Xref = [0;0;0];
        end
        
        % ===== 极简 Bancroft（仅作兜底，够用即可） =====
        function [x0, b0] = bancroft_init(SV, P)
            M = size(SV,2);  assert(M>=4,'Bancroft needs >=4 sats');
            A = [SV', ones(M,1)];
            y = 0.5*(sum(SV'.^2,2) - P.^2);
            u = A\y;  xb = u(1:3);  bb = u(4);
            % 两支解的选择（简化版）：
            rho = vecnorm(SV - xb, 2, 1)';   % M×1
            res = [norm(P - (rho + bb));  norm(P - (rho - bb))];
            if res(1) <= res(2), x0 = xb; b0 = bb; else, x0 = xb; b0 = -bb; end
        end


        % 调 RTKLIB 的 SPP
        function spp(obj)
            classFullPath = mfilename('fullpath');
            classDir = fileparts(classFullPath);
            opt = gt.Gopt(fullfile(classDir, "spp.conf"));
            [sol, ~] = gt.Gfun.pntpos(obj.obs, obj.nav, opt);
            obj.xyz = sol.pos.xyz;   % Nx3 作为参考位置（仰角/遮罩/权重/可视化）
            sol.plot()
        end

        % 简单可视化
        function show(obj)
            obj.obs.plot;
            obj.obs.plot("L1");
            obj.obs.plotNSat("L1");
        end
    end
end
