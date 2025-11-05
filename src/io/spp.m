function [x,status] = spp(obsc,nav)
% input: obs: observation and nav: broadcast ephemeris
% output: ECEF position and receiver clock bias

x = zeros(4,1);
status = false;

nx = 4;
sat = gt.Gsat(obsc,nav); % Compute satellite position

EL_TH = 10; % Elevation angle threshold (deg)

for iter = 1:10
    sat.setRcvPos(gt.Gpos(x(1:3)',"xyz")); % Set current receiver position
    obsc = obsc.residuals(sat); % Compute pseudorange residuals at current position

    % resP = obs.P-(rng-dts+ion+trp)-dtr-tgd
    resP = obsc.L1.resPc-x(4)-nav.getTGD(sat.sat);

    idx = ~isnan(resP) & sat.el>EL_TH; % Index not NaN
    nobs = nnz(idx); % Number of current observation

    % Simple elevation angle dependent weight model
    varP90 = 0.5^2;
    w = 1./(varP90./sind(sat.el(idx)));
    sys = obsc.sys(idx);

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
        status = true;
        break;
    end
end
% xlog(i,:) = x';
% fprintf("i=%d iter:%d\n",i,iter);
end

