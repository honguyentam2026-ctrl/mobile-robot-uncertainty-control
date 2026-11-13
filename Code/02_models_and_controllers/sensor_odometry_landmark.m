function [odoMeas, obsList, shaftVelMeas] = sensor_odometry_landmark( ...
    stepIdx, poseTrue, encoderDelta, plantState, cfg)
%SENSOR_ODOMETRY_LANDMARK Simulate noisy odometry and range-bearing landmarks.
%   [ODOMEAS, OBSLIST, SHAFTVELMEAS] = SENSOR_ODOMETRY_LANDMARK(STEPIDX,
%   POSETRUE, ENCODERDELTA, PLANTSTATE, CFG) corrupts the encoder delta with
%   zero-mean Gaussian noise V_true = diag(sigmaOdoD^2, sigmaOdoTheta^2) and,
%   every cfg.landmarkPeriod steps, generates noisy range-bearing measurements
%   to all known landmarks within cfg.landmarkMaxRange.
%
%   Output obsList is an M-by-3 matrix with rows [landmarkId, range, bearing].

    scen = cfg.scenario;

    %% 1. Odometry measurement
    odoMeas = encoderDelta + [scen.sigmaOdoD * randn();
                              scen.sigmaOdoTheta * randn()];

    %% 2. Inner shaft velocity measurement (for PI-SIMC feedback)
    shaftVelMeas = [plantState.vShaft; plantState.wShaft] + ...
        scen.sigmaEncVel * randn(2, 1);

    %% 3. Range-bearing landmark observations (periodic)
    obsList = zeros(0, 3);
    if mod(stepIdx, cfg.landmarkPeriod) ~= 0
        return;
    end

    numLm = size(cfg.landmarks, 1);
    buf = zeros(numLm, 3);
    count = 0;

    for i = 1:numLm
        [zClean, ~] = landmark_measurement_model(poseTrue, cfg.landmarks(i, :)');
        if zClean(1) <= cfg.landmarkMaxRange
            rMeas = zClean(1) + scen.sigmaRange * randn();
            bMeas = wrap_to_pi(zClean(2) + scen.sigmaBearing * randn());
            count = count + 1;
            buf(count, :) = [i, rMeas, bMeas];
        end
    end

    obsList = buf(1:count, :);
end
