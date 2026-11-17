function res = simulate_closed_loop(mode, cfg)
%SIMULATE_CLOSED_LOOP Run one closed-loop differential-drive simulation.
%   RES = SIMULATE_CLOSED_LOOP(MODE, CFG) simulates path following under the
%   uncertainty scenario in CFG using one of five non-Kalman controller/
%   estimator modes:
%     'C0_ideal'          - Pure Pursuit using true pose (ideal reference)
%     'C1_noisy'          - Pure Pursuit using raw noisy odometry (no filter)
%     'C2_comp'           - Pure Pursuit + Complementary Filter (open-loop inner)
%     'C3_comp_pidf'      - Pure Pursuit + Complementary Filter + fixed PIDF
%     'C4_comp_fuzzy_pidf'- Pure Pursuit + Complementary Filter + Fuzzy-PIDF
%
%   Note: Ground-truth pose is NEVER passed to any controller/estimator in
%   modes C1..C4; only C0_ideal uses true pose as the upper-bound benchmark.

    validModes = {'C0_ideal', 'C1_noisy', 'C2_comp', 'C3_comp_pidf', 'C4_comp_fuzzy_pidf'};
    if ~ismember(mode, validModes)
        error('simulate_closed_loop:InvalidMode', ...
            'Unknown mode "%s". Must be one of: %s', mode, strjoin(validModes, ', '));
    end

    % Set reproducible RNG seed per run so all controllers see identical noise
    rng(cfg.seed, 'twister');

    dt = cfg.dt;
    maxSteps = ceil(cfg.maxSimTime / dt) + 1;
    goalXY = cfg.path(end, :);

    %% 1. Initialize Pure Pursuit Controller
    pp = controllerPurePursuit( ...
        'Waypoints', cfg.path, ...
        'DesiredLinearVelocity', cfg.pp.DesiredLinearVelocity, ...
        'MaxAngularVelocity', cfg.pp.MaxAngularVelocity, ...
        'LookaheadDistance', cfg.pp.LookaheadDistance);

    %% 2. Initialize Plant, Complementary Filter, PIDF, and Fuzzy Scheduler
    plantState.poseTrue = cfg.initialPose(:);
    plantState.vShaft = 0.0;
    plantState.wShaft = 0.0;
    plantState.uHistV = [];
    plantState.uHistW = [];

    poseTrue = plantState.poseTrue;
    poseOdo = cfg.initialPose(:);
    shaftVelMeas = [0.0; 0.0];
    pidState = [];
    fisState = [];
    compState = [];
    slipEst = 0.0;

    if strcmp(mode, 'C0_ideal') || strcmp(mode, 'C1_noisy')
        poseEst = cfg.initialPose(:);
    else
        [compState, poseEst, slipEst] = complementary_pose_filter([], [], [], cfg);
    end

    %% 3. Preallocate Logs
    tHist = zeros(maxSteps, 1);
    poseTrueHist = zeros(maxSteps, 3);
    poseEstHist = zeros(maxSteps, 3);
    slipEstHist = zeros(maxSteps, 1);
    fuzzyGainHist = ones(maxSteps, 4); % [sKp, sKi, sKd, slipBoost]
    vRefHist = zeros(maxSteps, 1);
    wRefHist = zeros(maxSteps, 1);
    uCmdHist = zeros(maxSteps, 2);

    stepIdx = 1;
    poseTrueHist(stepIdx, :) = poseTrue';
    poseEstHist(stepIdx, :) = poseEst';
    slipEstHist(stepIdx) = slipEst;

    distanceToGoal = norm(poseTrue(1:2)' - goalXY);
    vPPPrev = 0.0;
    wPPPrev = 0.0;

    %% 4. Closed-Loop Simulation
    while (distanceToGoal > cfg.goalRadius) && (stepIdx < maxSteps)
        tCurr = (stepIdx - 1) * dt;

        % Select state feedback available to outer controller
        if strcmp(mode, 'C0_ideal')
            poseForCtrl = poseTrue;
        else
            poseForCtrl = poseEst;
        end

        % Optional Fuzzy adaptation of Pure Pursuit lookahead + PIDF gains (C4)
        gainScale = [1.0, 1.0, 1.0];
        slipBoost = 1.0;
        if strcmp(mode, 'C4_comp_fuzzy_pidf')
            cteEst = local_point_to_polyline_dist(poseForCtrl(1:2)', cfg.path);
            velErr = hypot(vPPPrev - shaftVelMeas(1), 0.5 * (wPPPrev - shaftVelMeas(2)));
            errNorm = (0.65 * cteEst + 0.35 * velErr) / cfg.fuzzyPid.errScale;
            slipNorm = slipEst / cfg.fuzzyPid.slipScale;

            [gainScale, slipBoost, fisState] = fuzzy_pid_scheduler( ...
                fisState, errNorm, slipNorm, cfg);

            if cfg.fuzzyPid.adaptLookahead
                % Tighten lookahead only when tracking error is significant
                errExcess = max(cteEst - 0.08, 0.0) / 0.25;
                adaptFactor = 1.0 - 0.22 * min(errExcess, 1.0);
                pp.LookaheadDistance = max(0.30, cfg.pp.LookaheadDistance * adaptFactor);
            end
        end

        % Outer guidance (Pure Pursuit)
        [vPP, wPP] = pp(poseForCtrl);
        vPPPrev = vPP;
        wPPPrev = wPP;

        if strcmp(mode, 'C4_comp_fuzzy_pidf')
            vRef = min(max(vPP * slipBoost, cfg.actuator.vLim(1)), cfg.actuator.vLim(2));
            wRef = wPP;
        else
            vRef = vPP;
            wRef = wPP;
        end

        % Inner velocity controller (open-loop vs fixed PIDF vs Fuzzy-PIDF)
        if strcmp(mode, 'C3_comp_pidf') || strcmp(mode, 'C4_comp_fuzzy_pidf')
            [pidState, uCmd] = pid_speed_filtered( ...
                pidState, [vRef; wRef], shaftVelMeas, cfg, gainScale);
        else
            uCmd = [vRef; wRef];
        end

        vRefHist(stepIdx) = vPP;
        wRefHist(stepIdx) = wPP;
        uCmdHist(stepIdx, :) = uCmd';
        fuzzyGainHist(stepIdx, :) = [gainScale, slipBoost];

        % True physical robot step
        [plantState, poseTrue, encoderDelta, ~] = robot_plant_step( ...
            plantState, uCmd, tCurr, cfg);

        % Simulate noisy odometry + periodic landmarks
        [odoMeas, obsList, shaftVelMeas] = sensor_odometry_landmark( ...
            stepIdx, poseTrue, encoderDelta, plantState, cfg);

        % State estimation update (none vs raw odometry vs complementary filter)
        if strcmp(mode, 'C0_ideal')
            poseEst = poseTrue;
            slipEst = 0.0;
        elseif strcmp(mode, 'C1_noisy')
            poseOdo = [poseOdo(1) + odoMeas(1) * cos(poseOdo(3));
                       poseOdo(2) + odoMeas(1) * sin(poseOdo(3));
                       wrap_to_pi(poseOdo(3) + odoMeas(2))];
            poseEst = poseOdo;
            slipEst = 0.0;
        else
            [compState, poseEst, slipEst] = complementary_pose_filter( ...
                compState, odoMeas, obsList, cfg);
        end

        stepIdx = stepIdx + 1;
        tHist(stepIdx) = (stepIdx - 1) * dt;
        poseTrueHist(stepIdx, :) = poseTrue';
        poseEstHist(stepIdx, :) = poseEst';
        slipEstHist(stepIdx) = slipEst;

        distanceToGoal = norm(poseTrue(1:2)' - goalXY);
    end

    %% 5. Trim Logs & Compute Quantitative Metrics
    tHist = tHist(1:stepIdx);
    poseTrueHist = poseTrueHist(1:stepIdx, :);
    poseEstHist = poseEstHist(1:stepIdx, :);
    slipEstHist = slipEstHist(1:stepIdx);
    fuzzyGainHist = fuzzyGainHist(1:stepIdx, :);
    vRefHist = vRefHist(1:stepIdx);
    wRefHist = wRefHist(1:stepIdx);
    uCmdHist = uCmdHist(1:stepIdx, :);

    metrics = compute_path_metrics( ...
        tHist, poseTrueHist, cfg.path, uCmdHist(1:end-1, 1), uCmdHist(1:end-1, 2));
    metrics.reachedGoal = (metrics.finalGoalError <= cfg.goalRadius);

    % Estimation error metrics (from poseEst vs poseTrue)
    posErr = sqrt(sum((poseEstHist(:, 1:2) - poseTrueHist(:, 1:2)).^2, 2));
    headErr = wrap_to_pi(poseEstHist(:, 3) - poseTrueHist(:, 3));

    estMetrics.posRmse = sqrt(mean(posErr.^2));
    estMetrics.headingRmseDeg = rad2deg(sqrt(mean(headErr.^2)));

    %% 6. Pack Result Struct
    res.mode = mode;
    res.scenarioName = cfg.scenario.name;
    res.seed = cfg.seed;
    res.t = tHist;
    res.poseTrue = poseTrueHist;
    res.poseEst = poseEstHist;
    res.slipEst = slipEstHist;
    res.fuzzyGains = fuzzyGainHist;
    res.vRef = vRefHist;
    res.wRef = wRefHist;
    res.uCmd = uCmdHist;
    res.metrics = metrics;
    res.estMetrics = estMetrics;
end

function dMin = local_point_to_polyline_dist(pt, poly)
%LOCAL_POINT_TO_POLYLINE_DIST Minimum Euclidean distance from 2D point to path.
    dMin = inf;
    for k = 1:(size(poly, 1) - 1)
        a = poly(k, :);
        b = poly(k + 1, :);
        ab = b - a;
        lenSq = dot(ab, ab);
        if lenSq <= 1e-12
            d = norm(pt - a);
        else
            tProj = min(max(dot(pt - a, ab) / lenSq, 0.0), 1.0);
            proj = a + tProj * ab;
            d = norm(pt - proj);
        end
        if d < dMin
            dMin = d;
        end
    end
end
