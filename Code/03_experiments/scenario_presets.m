function cfg = scenario_presets(scenarioId)
%SCENARIO_PRESETS Build simulation configuration for benchmark scenarios S0..S5.
%   CFG = SCENARIO_PRESETS(SCENARIOID) returns a default_sim_config() struct
%   configured for one of the standard evaluation scenarios:
%     'S0_ideal'          - Ideal baseline (no noise, no mismatch, no slip, nominal plant)
%     'S1_sensor_noise'   - Zero-mean Gaussian odometry & landmark noise
%     'S2_wheel_slip'     - Sensor noise + severe wet-floor slip zone
%     'S3_model_mismatch' - Sensor noise + systematic wheel radius / track mismatch
%     'S4_load_step'      - Sensor noise + mid-run heavy payload step change (K, T)
%     'S5_worst_case'     - Combined noise + mismatch + slip zone + load step

    cfg = default_sim_config();
    cfg.scenario.name = scenarioId;

    if strcmp(scenarioId, 'S0_ideal')
        return;
    end

    % Baseline realistic sensor noise shared across S1..S5
    cfg.scenario.sigmaOdoD = 0.02;             % [m/step]
    cfg.scenario.sigmaOdoTheta = deg2rad(1.5); % [rad/step]
    cfg.scenario.sigmaRange = 0.08;            % [m]
    cfg.scenario.sigmaBearing = deg2rad(2.5);  % [rad]
    cfg.scenario.sigmaEncVel = 0.005;          % [m/s, rad/s]

    switch scenarioId
        case 'S1_sensor_noise'
            % Pure zero-mean noise only

        case 'S2_wheel_slip'
            cfg.scenario.slipZone.enabled = true;
            cfg.scenario.slipZone.xRange = [4.0, 9.5];
            cfg.scenario.slipZone.yRange = [5.0, 9.8];
            cfg.scenario.slipZone.slipV = 0.30;
            cfg.scenario.slipZone.slipW = 0.30;

        case 'S3_model_mismatch'
            % -20% effective wheel radius, +20% yaw scaling (narrower effective track)
            cfg.scenario.etaVBase = 0.80;
            cfg.scenario.etaWBase = 1.20;

        case 'S4_load_step'
            cfg.scenario.loadStep.enabled = true;
            cfg.scenario.loadStep.time = 20.0;
            cfg.scenario.loadStep.K = 0.65;
            cfg.scenario.loadStep.T = 0.70;
            cfg.scenario.loadStep.tau = 0.10;

        case 'S5_worst_case'
            cfg.scenario.etaVBase = 0.85;
            cfg.scenario.etaWBase = 0.80;
            cfg.scenario.slipZone.enabled = true;
            cfg.scenario.slipZone.xRange = [4.0, 9.5];
            cfg.scenario.slipZone.yRange = [5.0, 9.8];
            cfg.scenario.slipZone.slipV = 0.25;
            cfg.scenario.slipZone.slipW = 0.25;
            cfg.scenario.loadStep.enabled = true;
            cfg.scenario.loadStep.time = 20.0;
            cfg.scenario.loadStep.K = 0.70;
            cfg.scenario.loadStep.T = 0.65;
            cfg.scenario.loadStep.tau = 0.10;

        otherwise
            error('scenario_presets:UnknownScenario', ...
                'Unknown scenario "%s".', scenarioId);
    end
end
