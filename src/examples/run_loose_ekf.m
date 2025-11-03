function out = run_loose_ekf()
cfg = config_default();

% 类实例化
F   = EKF(cfg);
proc= models.imu.ProcessIMU();
meas= models.gnss_loose.MeasPos();
io  = io.RTKLibAdapter();

% 数据
imu      = load_imu_csv(cfg.dt_imu);           % 你已有的读取函数/类
gnss_pos = io.loadLoose('data/rover.obs','data/rover.nav');
[imu, gnss_pos] = io.sync_streams(imu, gnss_pos); % 若你做成类方法

% 主循环：以 IMU 时间步驱动
traj = [];
for k = 1:numel(imu)-1
    dt = imu(k+1).t - imu(k).t;
    F.predict(proc, imu(k), dt);

    % 若此刻有 LC 位置量测（这里按索引对齐；真实项目请按时间戳查找）
    if k <= numel(gnss_pos)
        z = gnss_pos(k).pos;
        % 若要用 RTK 的 HDOP 替换 cfg.lc.HDOP_default，可将其塞给 meas（或 cfg）
        F.update(meas, z);
    end

    traj(end+1,:) = [F.x(1:3).' F.x(4:6).']; %#ok<AGROW>
end

out.traj = traj;
end
