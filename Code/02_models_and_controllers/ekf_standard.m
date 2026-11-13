function [ekfState, poseEst, nis] = ekf_standard(ekfState, odoMeas, obsList, cfg)
%EKF_STANDARD 3-state Extended Kalman Filter ([x; y; theta]) per Ch.4_part 2.
%   [EKFSTATE, POSEEST, NIS] = EKF_STANDARD(EKFSTATE, ODOMEAS, OBSLIST, CFG)
%   performs odometry prediction with Jacobians (Fx, Fv) and sequential
%   range-bearing landmark updates via ekf_joseph_update.

    if isempty(ekfState)
        ekfState.x = cfg.initialPose(:);
        ekfState.P = cfg.ekf.P0;
        poseEst = ekfState.x;
        nis = NaN;
        return;
    end

    x = ekfState.x;
    P = ekfState.P;

    %% 1. Predict Step (odometry model, Chapter 4_part 2 p.12-17)
    d = odoMeas(1);
    dTheta = odoMeas(2);
    theta = x(3);

    c = cos(theta);
    s = sin(theta);

    xPred = [x(1) + d * c;
             x(2) + d * s;
             wrap_to_pi(theta + dTheta)];

    Fx = [1.0, 0.0, -d * s;
          0.0, 1.0,  d * c;
          0.0, 0.0,  1.0];

    Fv = [c,   0.0;
          s,   0.0;
          0.0, 1.0];

    PPred = Fx * P * Fx' + Fv * cfg.ekf.Vhat * Fv';
    PPred = 0.5 * (PPred + PPred');

    %% 2. Update Step (range-bearing landmarks)
    [xUpd, PUpd, nis] = ekf_joseph_update( ...
        xPred, PPred, obsList, cfg.landmarks, ...
        cfg.ekf.What, cfg.ekf.useGating, cfg.ekf.gateChi2);

    ekfState.x = xUpd;
    ekfState.P = PUpd;
    poseEst = xUpd;
end
