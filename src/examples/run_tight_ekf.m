function out = run_tight_ekf()
cfg = config_default();

F    = EKF(cfg);
proc = models.imu.ProcessIMU();
meas = models.gnss_tight.MeasPseudorange();
io   = io.RTKLibAdapter();

imu      = load_imu_csv(cfg.dt_imu);
gnss_raw = io.loadTight('data/rover.obs','data/rover.nav');
[imu, gnss_raw] = io.sync_streams(imu, gnss_raw);

traj = [];
for k = 1:numel(imu)-1
    dt = imu(k+1).t - imu(k).t;
    F.predict(proc, imu(k), dt);

    if k <= numel(gnss_raw)
        sats = gnss_raw(k).sats;  % 1×M struct
        pr   = gnss_raw(k).pr;    % M×1

        % 批量组装 r,H,R
        r_all = []; H_all = []; R_diag = [];
        for i=1:numel(sats)
            [ri, Hi, Ri] = meas.linearize(F.x, pr(i), cfg, sats(i));
            r_all = [r_all; ri];
            H_all = [H_all; Hi];
            R_diag = [R_diag; Ri];
        end

        % 为了复用 EKF.update() 的接口，我们做个"临时观测模型"把 (r,H,R) 返回
        pseudo = struct(); 
        pseudo.linearize = @(x,z,cfg_) deal(r_all, H_all, diag(R_diag));
        F.update(pseudo, []);
    end

    traj(end+1,:) = [F.x(1:3).' F.x(4:6).']; %#ok<AGROW>
end

out.traj = traj;
end
