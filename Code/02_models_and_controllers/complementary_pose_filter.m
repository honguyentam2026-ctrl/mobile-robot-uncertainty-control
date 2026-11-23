function [compState, poseEst, slipEst] = complementary_pose_filter( ...
    compState, odoMeas, obsList, cfg)
%COMPLEMENTARY_POSE_FILTER 2D complementary pose fusion (odometry + landmarks).
%   [COMPSTATE, POSEEST, SLIPEST] = COMPLEMENTARY_POSE_FILTER(COMPSTATE,
%   ODOMEAS, OBSLIST, CFG) fuses high-rate wheel odometry (10 Hz) with
%   periodic range-bearing landmark observations (2 Hz) using a fixed-gain
%   complementary filter (lightweight algebraic fusion for C2..C4).
%
%   Also tracks a smoothed apparent longitudinal slip/scale indicator
%   SLIPEST in [0, 0.6] by comparing landmark-corrected displacement against
%   raw encoder odometry over each landmark interval.

    if isempty(compState)
        compState.poseEst = cfg.initialPose(:);
        compState.poseAtLastLm = cfg.initialPose(:);
        compState.odoDistAccum = 0.0;
        compState.odoTurnAccum = 0.0;
        compState.slipEst = 0.0;
        poseEst = compState.poseEst;
        slipEst = compState.slipEst;
        return;
    end

    alphaPos = cfg.compFilter.alphaPos;
    alphaTh  = cfg.compFilter.alphaTheta;
    maxInnov = cfg.compFilter.maxInnovation;

    %% 1. Odometry dead-reckoning prediction step (10 Hz)
    p = compState.poseEst;
    dS = odoMeas(1);
    dTh = odoMeas(2);

    xPred = p(1) + dS * cos(p(3));
    yPred = p(2) + dS * sin(p(3));
    thPred = wrap_to_pi(p(3) + dTh);
    compState.odoDistAccum = compState.odoDistAccum + abs(dS);
    compState.odoTurnAccum = compState.odoTurnAccum + abs(dTh);

    %% 2. Periodic landmark complementary correction (2 Hz)
    if ~isempty(obsList)
        nObs = size(obsList, 1);

        % Heading estimate from landmark pairs (closed-form relative geometry)
        thCorr = thPred;
        if nObs >= 2
            sinSum = 0.0;
            cosSum = 0.0;
            pairCount = 0;
            for i = 1:(nObs - 1)
                id1 = obsList(i, 1);
                r1 = obsList(i, 2);
                b1 = obsList(i, 3);
                L1 = cfg.landmarks(id1, :);
                pb1 = [r1 * cos(b1); r1 * sin(b1)];

                for j = (i + 1):nObs
                    id2 = obsList(j, 1);
                    r2 = obsList(j, 2);
                    b2 = obsList(j, 3);
                    L2 = cfg.landmarks(id2, :);
                    pb2 = [r2 * cos(b2); r2 * sin(b2)];

                    dW = L2(:) - L1(:);
                    dB = pb2 - pb1;
                    if norm(dW) > 0.5 && norm(dB) > 0.5
                        thPair = wrap_to_pi(atan2(dW(2), dW(1)) - atan2(dB(2), dB(1)));
                        sinSum = sinSum + sin(thPair);
                        cosSum = cosSum + cos(thPair);
                        pairCount = pairCount + 1;
                    end
                end
            end

            if pairCount > 0
                thMeas = atan2(sinSum, cosSum);
                thErr = wrap_to_pi(thMeas - thPred);
                if abs(thErr) < deg2rad(35.0)
                    thCorr = wrap_to_pi(thPred + alphaTh * thErr);
                end
            end
        end

        % Position estimate weighted by inverse squared range
        wSum = 0.0;
        xSum = 0.0;
        ySum = 0.0;
        for i = 1:nObs
            lmId = obsList(i, 1);
            rMeas = obsList(i, 2);
            bMeas = obsList(i, 3);
            L = cfg.landmarks(lmId, :);

            xLm = L(1) - rMeas * cos(thCorr + bMeas);
            yLm = L(2) - rMeas * sin(thCorr + bMeas);
            w = 1.0 / max(rMeas, 0.5)^2;
            xSum = xSum + w * xLm;
            ySum = ySum + w * yLm;
            wSum = wSum + w;
        end

        if wSum > 0
            xMeas = xSum / wSum;
            yMeas = ySum / wSum;
            innov = hypot(xMeas - xPred, yMeas - yPred);
            if innov < maxInnov
                xPred = (1.0 - alphaPos) * xPred + alphaPos * xMeas;
                yPred = (1.0 - alphaPos) * yPred + alphaPos * yMeas;
                thPred = thCorr;
            end
        end

        % Update smoothed apparent longitudinal slip indicator (using arc-chord correction)
        if compState.odoDistAccum > 0.12 && compState.odoTurnAccum < 0.70
            dispChord = hypot(xPred - compState.poseAtLastLm(1), ...
                              yPred - compState.poseAtLastLm(2));
            halfTurn = 0.5 * compState.odoTurnAccum;
            sincTurn = max(sin(halfTurn) / max(halfTurn, 1e-6), 0.85);
            dispArc  = dispChord / sincTurn;
            rawSlip = 1.0 - dispArc / compState.odoDistAccum;
            rawSlip = min(max(rawSlip, 0.0), 0.50);
            alphaSlip = 0.25 + 0.15 * (rawSlip < compState.slipEst);
            compState.slipEst = (1.0 - alphaSlip) * compState.slipEst + alphaSlip * rawSlip;
        end
        compState.poseAtLastLm = [xPred; yPred; thPred];
        compState.odoDistAccum = 0.0;
        compState.odoTurnAccum = 0.0;
    end

    compState.poseEst = [xPred; yPred; thPred];
    poseEst = compState.poseEst;
    slipEst = compState.slipEst;
end
