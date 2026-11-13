function res = simulate_closed_loop(mode, cfg)
%SIMULATE_CLOSED_LOOP Run one closed-loop differential-drive simulation.
%   RES = SIMULATE_CLOSED_LOOP(MODE, CFG) simulates path following under the
%   uncertainty scenario in CFG using one of five controller/estimator modes:
%     'C0_ideal'        - Pure Pursuit using true pose (ideal reference)
%     'C1_noisy'        - Pure Pursuit using raw noisy odometry (no filter)
%     'C2_ekf'          - Pure Pursuit + 3-state EKF (odo + landmarks)
%     'C3_ekf_simc'     - Pure Pursuit + 3-state EKF + inner PI-SIMC loop
%     'C4_aug_ekf_simc' - Pure Pursuit + 5-state Augmented EKF (eta_v, eta_w)
%                         + inner PI-SIMC loop + slip/scale compensation
%
%   Note: Ground-truth pose is NEVER passed to any controller/estimator in
%   modes C1..C4; only C0_ideal uses true pose as the upper-bound benchmark.

    validModes = {'C0_ideal', 'C1_noisy', 'C2_ekf', 'C3_ekf_simc', 'C4_aug_ekf_simc'};
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

    %% 2. Initialize Plant, Estimator, and Inner PI-SIMC Controller
    plantState.poseTrue = cfg.initialPose(:);
    plantState.vShaft = 0.0;
    plantState.wShaft = 0.0;
    plantState.uHistV = [];
    plantState.uHistW = [];

    poseTrue = plantState.poseTrue;
    poseOdo = cfg.initialPose(:);
    shaftVelMeas = [0.0; 0.0];
    piState = [];

    ekfState = [];
    etaEst = cfg.augEkf.eta0(:);
    if strcmp(mode, 'C0_ideal') || strcmp(mode, 'C1_noisy')
        poseEst = cfg.initialPose(:);
        pDiagInit = nan(1, 3);
    elseif strcmp(mode, 'C4_aug_ekf_simc')
        [ekfState, poseEst, etaEst, ~] = ekf_augmented([], [], [], cfg);
        pDiagInit = diag(ekfState.P)';
    else
        [ekfState, poseEst, ~] = ekf_standard([], [], [], cfg);
        pDiagInit = diag(ekfState.P)';
    end

    %% 3. Preallocate Logs
    tHist = zeros(maxSteps, 1);
    poseTrueHist = zeros(maxSteps, 3);
    poseEstHist = zeros(maxSteps, 3);
    pDiagHist = nan(maxSteps, length(pDiagInit));
    etaTrueHist = ones(maxSteps, 2);
    etaEstHist = ones(maxSteps, 2);
    vRefHist = zeros(maxSteps, 1);
    wRefHist = zeros(maxSteps, 1);
    uCmdHist = zeros(maxSteps, 2);
    nisHist = nan(maxSteps, 1);

    stepIdx = 1;
    poseTrueHist(stepIdx, :) = poseTrue';
    poseEstHist(stepIdx, :) = poseEst';
    pDiagHist(stepIdx, :) = pDiagInit;
    etaTrueHist(stepIdx, :) = [cfg.scenario.etaVBase, cfg.scenario.etaWBase];
    etaEstHist(stepIdx, :) = etaEst';

    distanceToGoal = norm(poseTrue(1:2)' - goalXY);

    %% 4. Closed-Loop Simulation
    while (distanceToGoal > cfg.goalRadius) && (stepIdx < maxSteps)
        tCurr = (stepIdx - 1) * dt;

        % Select state feedback available to outer controller
        if strcmp(mode, 'C0_ideal')
            poseForCtrl = poseTrue;
        else
            poseForCtrl = poseEst;
        end

        % Outer guidance (Pure Pursuit)
        [vPP, wPP] = pp(poseForCtrl);
        vRef = vPP;
        wRef = wPP;

        % Augmented-EKF inverse scale/slip feedforward compensation
        if strcmp(mode, 'C4_aug_ekf_simc') && cfg.augEkf.compensateControl
            bMin = cfg.augEkf.etaBounds(1);
            bMax = cfg.augEkf.etaBounds(2);
            etaVC = min(max(etaEst(1), bMin), bMax);
            etaWC = min(max(etaEst(2), bMin), bMax);
            vRef = min(max(vPP / etaVC, cfg.actuator.vLim(1)), cfg.actuator.vLim(2));
            wRef = min(max(wPP / etaWC, cfg.actuator.wLim(1)), cfg.actuator.wLim(2));
        end

        % Inner velocity controller (open-loop vs PI-SIMC)
        if strcmp(mode, 'C3_ekf_simc') || strcmp(mode, 'C4_aug_ekf_simc')
            [piState, uCmd] = pi_speed_simc(piState, [vRef; wRef], shaftVelMeas, cfg);
        else
            uCmd = [vRef; wRef];
        end

        vRefHist(stepIdx) = vPP;
        wRefHist(stepIdx) = wPP;
        uCmdHist(stepIdx, :) = uCmd';

        % True physical robot step
        [plantState, poseTrue, encoderDelta, etaTrue] = robot_plant_step( ...
            plantState, uCmd, tCurr, cfg);

        % Simulate noisy odometry + periodic landmarks
        [odoMeas, obsList, shaftVelMeas] = sensor_odometry_landmark( ...
            stepIdx, poseTrue, encoderDelta, plantState, cfg);

        % State estimation update
        nis = NaN;
        if strcmp(mode, 'C0_ideal')
            poseEst = poseTrue;
            pDiagCurr = nan(1, 3);
        elseif strcmp(mode, 'C1_noisy')
            poseOdo = [poseOdo(1) + odoMeas(1) * cos(poseOdo(3));
                       poseOdo(2) + odoMeas(1) * sin(poseOdo(3));
                       wrap_to_pi(poseOdo(3) + odoMeas(2))];
            poseEst = poseOdo;
            pDiagCurr = nan(1, 3);
        elseif strcmp(mode, 'C4_aug_ekf_simc')
            [ekfState, poseEst, etaEst, nis] = ekf_augmented( ...
                ekfState, odoMeas, obsList, cfg);
            pDiagCurr = diag(ekfState.P)';
        else
            [ekfState, poseEst, nis] = ekf_standard( ...
                ekfState, odoMeas, obsList, cfg);
            pDiagCurr = diag(ekfState.P)';
        end

        stepIdx = stepIdx + 1;
        tHist(stepIdx) = (stepIdx - 1) * dt;
        poseTrueHist(stepIdx, :) = poseTrue';
        poseEstHist(stepIdx, :) = poseEst';
        pDiagHist(stepIdx, :) = pDiagCurr;
        etaTrueHist(stepIdx, :) = etaTrue';
        etaEstHist(stepIdx, :) = etaEst';
        nisHist(stepIdx) = nis;

        distanceToGoal = norm(poseTrue(1:2)' - goalXY);
    end

    %% 5. Trim Logs & Compute Quantitative Metrics
    tHist = tHist(1:stepIdx);
    poseTrueHist = poseTrueHist(1:stepIdx, :);
    poseEstHist = poseEstHist(1:stepIdx, :);
    pDiagHist = pDiagHist(1:stepIdx, :);
    etaTrueHist = etaTrueHist(1:stepIdx, :);
    etaEstHist = etaEstHist(1:stepIdx, :);
    vRefHist = vRefHist(1:stepIdx);
    wRefHist = wRefHist(1:stepIdx);
    uCmdHist = uCmdHist(1:stepIdx, :);
    nisHist = nisHist(1:stepIdx);

    metrics = compute_path_metrics( ...
        tHist, poseTrueHist, cfg.path, uCmdHist(1:end-1, 1), uCmdHist(1:end-1, 2));
    metrics.reachedGoal = (metrics.finalGoalError <= cfg.goalRadius);

    % Estimation error & 2-sigma consistency metrics
    posErr = sqrt(sum((poseEstHist(:, 1:2) - poseTrueHist(:, 1:2)).^2, 2));
    headErr = wrap_to_pi(poseEstHist(:, 3) - poseTrueHist(:, 3));

    estMetrics.posRmse = sqrt(mean(posErr.^2));
    estMetrics.headingRmseDeg = rad2deg(sqrt(mean(headErr.^2)));

    if all(isnan(pDiagHist(:, 1)))
        estMetrics.within2SigmaPos = [NaN, NaN, NaN];
    else
        errXYZ = [poseEstHist(:, 1:2) - poseTrueHist(:, 1:2), headErr];
        sig2 = 2.0 * sqrt(max(pDiagHist(:, 1:3), 0.0));
        estMetrics.within2SigmaPos = mean(abs(errXYZ) <= sig2, 1);
    end

    estMetrics.etaRmse = sqrt(mean((etaEstHist - etaTrueHist).^2, 1));
    validNis = nisHist(~isnan(nisHist));
    estMetrics.meanNis = mean(validNis);

    %% 6. Pack Result Struct
    res.mode = mode;
    res.scenarioName = cfg.scenario.name;
    res.seed = cfg.seed;
    res.t = tHist;
    res.poseTrue = poseTrueHist;
    res.poseEst = poseEstHist;
    res.pDiag = pDiagHist;
    res.etaTrue = etaTrueHist;
    res.etaEst = etaEstHist;
    res.vRef = vRefHist;
    res.wRef = wRefHist;
    res.uCmd = uCmdHist;
    res.nis = nisHist;
    res.metrics = metrics;
    res.estMetrics = estMetrics;
end
