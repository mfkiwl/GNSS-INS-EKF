classdef GNSSData < handle
    properties
        obs;
        nav;
        xyzt;
    end
    
    methods
        function obj = GNSSData(obs_file, nav_file)
            obj.obs = gt.Gobs(obs_file);
            obj.nav = gt.Gnav(nav_file);
            obj.single_point_positioning();

            obj.obs.plot
            posest = gt.Gpos(obj.xyzt(:,1:3),'xyz');
            posest.plot
        end

        function obs = get_obs(obj)
            obs = obj.obs;
        end

        function single_point_positioning(obj)
            % Select only GPS
            tmp_obs = obj.obs;
            tmp_obs = tmp_obs.selectSat(tmp_obs.sys==gt.C.SYS_GPS);
            
            % Select observations for position computation
            SNR_TH = 20;    % SNR threshold (dBHz)
            EL_TH = 10;     % Elevation angle threshold (deg)
            tmp_obs.maskP(tmp_obs.L1.S<SNR_TH);
            
            % Initials
            nx = 3+1; % Position in ECEF and receiver clock [x,y,z,dtr]'
            x = zeros(nx, 1); % Initial position is center of the Earth
            xlog = zeros(tmp_obs.n, nx); % For logging solution 
            
            % Point positioning
            for i = 1 : tmp_obs.n
                obsc = tmp_obs.selectTime(i);       % Current observation
                sat = gt.Gsat(obsc, obj.nav);       % Compute satellite position

                % Repeat until convergence
                for iter = 1:10
                    sat.setRcvPos(gt.Gpos(x(1:3)',"xyz"));  % Set current receiver position
                    obsc = obsc.residuals(sat);             % Compute pseudorange residuals at current position

                    % resP = obs.P-(rng-dts+ion+trp)-dtr-tgd
                    resP = obsc.L1.resPc - x(4) - obj.nav.getTGD(sat.sat);

                    idx = ~isnan(resP) & sat.el>EL_TH;      % Index not NaN
                    nobs = nnz(idx);                        % Number of current observation

                    % Simple elevation angle dependent weight model
                    varP90 = 0.5^2;
                    w = 1./(varP90./sind(sat.el(idx))); 
                    % sys = obsc.sys(idx);

                    % Design matrix
                    H = zeros(nobs,nx);
                    H(:,1) = -sat.ex(idx)'; % LOS vector in ECEF X
                    H(:,2) = -sat.ey(idx)'; % LOS vector in ECEF Y
                    H(:,3) = -sat.ez(idx)'; % LOS vector in ECEF Z
                    H(:,4) = 1.0;           % Reciever clock

                    % Weighted least square
                    % (y-H*x)'*diag(w)*(y-H*x)
                    y = resP(idx)';
                    dx = lscov(H,y,w); % position/clock error

                    % Solution correction
                    x = x+dx;

                    % Exit loop after convergence 
                    if norm(dx)<1e-3
                        break;
                    end
                end
                xlog(i,:) = x';
            end
            obj.xyzt = xlog;
        end
    end
end
