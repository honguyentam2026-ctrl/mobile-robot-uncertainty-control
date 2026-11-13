function [ekfState, poseEst, etaEst, nis] = ekf_augmented( ...
    ekfState, odoMeas, obsList, cfg)
%EKF_AUGMENTED 5-state Augmented EKF estimating pose and slip/scale factors.
%   [EKFSTATE, POSEEST, ETAEST, NIS] = EKF_AUGMENTED(EKFSTATE, ODOMEAS,
%   OBSLIST, CFG) augments the standard pose state [x; y; theta] with
%   online velocity scale factors [eta_v; eta_w] to absorb systematic wheel
%   geometry mismatch and persistent wheel slip:
%     x_a = [x; y; theta; eta_v; eta_w]
%
%   Predict equations:
%     x_{k+1}     = x_k     + eta_v * d * cos(theta_k)
%     y_{k+1}     = y_k     + eta_v * d * sin(theta_k)
%     theta_{k+1} = wrap_to_pi(theta_k + eta_w * dTheta)
%     eta_{k+1}   = eta_k   + w_eta

    if isempty(ekfState)
        ekfState.x = [cfg.initialPose(:); cfg.augEkf.eta0(:)];
        ekfState.P = blkdiag(cfg.ekf.P0, cfg.augEkf.P0Eta);
        poseEst = ekfState.x(1:3);
        etaEst = ekfState.x(4:5);
        nis = NaN;
        return;
    end

    x = ekfState.x;
    P = ekfState.P;

    %% 1. Predict Step (5-state augmented kinematics)
    d = odoMeas(1);
    dTheta = odoMeas(2);
    theta = x(3);
    etaV = x(4);
    etaW = x(5);

    c = cos(theta);
    s = sin(theta);

    xPred = [x(1) + etaV * d * c;
             x(2) + etaV * d * s;
             wrap_to_pi(theta + etaW * dTheta);
             etaV;
             etaW];

    Fx = [1.0, 0.0, -etaV * d * s, d * c, 0.0;
          0.0, 1.0,  etaV * d * c, d * s, 0.0;
          0.0, 0.0,  1.0,          0.0,   dTheta;
          0.0, 0.0,  0.0,          1.0,   0.0;
          0.0, 0.0,  0.0,          0.0,   1.0];

    Fv = [etaV * c, 0.0;
          etaV * s, 0.0;
          0.0,      etaW;
          0.0,      0.0;
          0.0,      0.0];

    % Scale parameter random walk by motion observability so covariance
    % does not inflate unboundedly when stationary or moving straight.
    actV = double(abs(d) > 1e-4);
    actW = double(abs(dTheta) > 1e-4);
    qEtaStep = diag([cfg.augEkf.qEta(1, 1) * (0.05 + 0.95 * actV), ...
                     cfg.augEkf.qEta(2, 2) * (0.05 + 0.95 * actW)]);
    QAug = blkdiag(zeros(3, 3), qEtaStep);

    PPred = Fx * P * Fx' + Fv * cfg.ekf.Vhat * Fv' + QAug;
    PPred = 0.5 * (PPred + PPred');

    %% 2. Update Step (range-bearing landmarks)
    [xUpd, PUpd, nis] = ekf_joseph_update( ...
        xPred, PPred, obsList, cfg.landmarks, ...
        cfg.ekf.What, cfg.ekf.useGating, cfg.ekf.gateChi2);

    % Clamp estimated scale factors within physical bounds
    bMin = cfg.augEkf.etaBounds(1);
    bMax = cfg.augEkf.etaBounds(2);
    xUpd(4) = min(max(xUpd(4), bMin), bMax);
    xUpd(5) = min(max(xUpd(5), bMin), bMax);

    ekfState.x = xUpd;
    ekfState.P = PUpd;
    poseEst = xUpd(1:3);
    etaEst = xUpd(4:5);
end
