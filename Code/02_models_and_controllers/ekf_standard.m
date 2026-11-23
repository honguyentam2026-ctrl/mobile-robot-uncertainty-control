function [ekfState, poseEst, nis, slipEst] = ekf_standard(ekfState, odoMeas, obsList, cfg)
%EKF_STANDARD 3-state Extended Kalman Filter ([x; y; theta]) per Ch.4_part 2.
%   [EKFSTATE, POSEEST, NIS, SLIPEST] = EKF_STANDARD(EKFSTATE, ODOMEAS, OBSLIST, CFG)
%   performs odometry prediction with Jacobians (Fx, Fv), sequential
%   range-bearing landmark updates via ekf_joseph_update, and longitudinal
%   slip estimation for Fuzzy-PID scheduling (C5_ekf_fuzzy_pidf).

    if isempty(ekfState)
        ekfState.x = cfg.initialPose(:);
        ekfState.P = cfg.ekf.P0;
        ekfState.poseAtLastLm = cfg.initialPose(:);
        ekfState.odoDistAccum = 0.0;
        ekfState.odoTurnAccum = 0.0;
        ekfState.slipEst = 0.0;
        poseEst = ekfState.x;
        nis = NaN;
        slipEst = 0.0;
        return;
    end

    x = ekfState.x;
    P = ekfState.P;

    %% 1. Predict Step (odometry midpoint model)
    d = odoMeas(1);
    dTheta = odoMeas(2);
    theta = x(3);
    thetaMid = theta + 0.5 * dTheta;

    c = cos(thetaMid);
    s = sin(thetaMid);

    xPred = [x(1) + d * c;
             x(2) + d * s;
             wrap_to_pi(theta + dTheta)];

    ekfState.odoDistAccum = ekfState.odoDistAccum + abs(d);
    ekfState.odoTurnAccum = ekfState.odoTurnAccum + abs(dTheta);

    Fx = [1.0, 0.0, -d * s;
          0.0, 1.0,  d * c;
          0.0, 0.0,  1.0];

    Fv = [c,   -0.5 * d * s;
          s,    0.5 * d * c;
          0.0,  1.0];

    PPred = Fx * P * Fx' + Fv * cfg.ekf.Vhat * Fv';
    PPred = 0.5 * (PPred + PPred');

    %% 2. Update Step (range-bearing landmarks)
    [xUpd, PUpd, nis] = ekf_joseph_update( ...
        xPred, PPred, obsList, cfg.landmarks, ...
        cfg.ekf.What, cfg.ekf.useGating, cfg.ekf.gateChi2);

    if ~isempty(obsList)
        if ekfState.odoDistAccum > 0.12 && ekfState.odoTurnAccum < 0.70
            dispChord = hypot(xUpd(1) - ekfState.poseAtLastLm(1), ...
                              xUpd(2) - ekfState.poseAtLastLm(2));
            halfTurn = 0.5 * ekfState.odoTurnAccum;
            sincTurn = max(sin(halfTurn) / max(halfTurn, 1e-6), 0.85);
            dispArc  = dispChord / sincTurn;
            rawSlip = 1.0 - dispArc / ekfState.odoDistAccum;
            rawSlip = min(max(rawSlip, 0.0), 0.50);
            alphaSlip = 0.25 + 0.15 * (rawSlip < ekfState.slipEst);
            ekfState.slipEst = (1.0 - alphaSlip) * ekfState.slipEst + alphaSlip * rawSlip;
        end
        ekfState.poseAtLastLm = xUpd;
        ekfState.odoDistAccum = 0.0;
        ekfState.odoTurnAccum = 0.0;
    end

    ekfState.x = xUpd;
    ekfState.P = PUpd;
    poseEst = xUpd;
    slipEst = ekfState.slipEst;
end
