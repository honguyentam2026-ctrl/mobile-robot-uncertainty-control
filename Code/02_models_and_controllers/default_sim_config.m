function cfg = default_sim_config()
%DEFAULT_SIM_CONFIG Return default simulation configuration for Topic 4.
%   CFG = DEFAULT_SIM_CONFIG() builds a structured configuration matching the
%   MathWorks baseline path while adding warehouse landmarks, FOPDT speed
%   actuator parameters, EKF covariances, and uncertainty scenario defaults
%   (defaulting to S0 ideal).

    %% Timing & Termination
    cfg.dt = 0.1;               % Sampling period [s]
    cfg.maxSimTime = 120.0;     % Maximum simulation horizon [s]
    cfg.goalRadius = 0.15;      % Goal arrival tolerance [m]
    cfg.seed = 42;              % Default RNG seed

    %% Reference Path & Initial Pose (matches 01_baseline_mathworks)
    cfg.path = [2.00  1.00;
                1.25  1.75;
                5.25  8.25;
                7.25  8.75;
                11.75 10.75;
                12.00 10.00];
    cfg.initialPose = [2.00; 1.00; 0.0]; % [x; y; theta]

    %% Known Warehouse Landmarks [x, y] in meters
    cfg.landmarks = [0.5,  3.5;
                     4.5,  2.5;
                     2.5,  8.5;
                     6.5,  5.5;
                     6.0, 11.0;
                     9.5,  7.0;
                     9.5, 12.0;
                    13.0,  8.5];
    cfg.landmarkPeriod = 5;     % Landmark measurement every N steps (0.5 s)
    cfg.landmarkMaxRange = 10.0;% Sensor maximum range [m]

    %% Pure Pursuit Outer Guidance Controller
    cfg.pp.DesiredLinearVelocity = 0.3; % [m/s]
    cfg.pp.MaxAngularVelocity = 2.0;    % [rad/s]
    cfg.pp.LookaheadDistance = 0.4;     % [m]

    %% Nominal Inner FOPDT Velocity Loop & S-IMC Tuning (Chapter 3_part 2)
    cfg.actuator.K0 = 1.0;      % Nominal DC gain [-]
    cfg.actuator.T0 = 0.30;     % Nominal time constant [s]
    cfg.actuator.tau0 = 0.10;   % Nominal dead-time [s]
    cfg.actuator.Tc = 0.25;     % Desired closed-loop time constant for SIMC [s]
    cfg.actuator.vLim = [-1.0, 1.0];    % Linear drive command limits [m/s]
    cfg.actuator.wLim = [-3.0, 3.0];    % Angular drive command limits [rad/s]
    cfg.actuator.useNominalDynamics = true; % If false, instantaneous kinematics

    %% Complementary Pose Filter Parameters (Non-Kalman, C2..C4)
    cfg.compFilter.alphaPos = 0.50;       % Landmark position blend gain [-]
    cfg.compFilter.alphaTheta = 0.40;     % Landmark heading blend gain [-]
    cfg.compFilter.maxInnovation = 1.50;  % Outlier rejection gate [m]

    %% Inner Filtered PID (PIDF) Velocity Loop (C3, C4)
    % Fixed PIDF tuned for robustness across nominal & loaded FOPDT plant
    cfg.pidf.Kp = 0.75;                   % Proportional gain [-]
    cfg.pidf.Ti = 0.50;                   % Integral time constant [s]
    cfg.pidf.Td = 0.08;                   % Derivative time constant [s]
    cfg.pidf.Tf = 0.20;                   % Derivative low-pass filter [s]

    %% Fuzzy-PID Gain Scheduler Parameters (C4, Fuzzy Logic Toolbox)
    cfg.fuzzyPid.errScale = 0.25;         % Speed/tracking error normalization
    cfg.fuzzyPid.slipScale = 0.30;        % Apparent slip normalization
    cfg.fuzzyPid.kpLevels = [0.75, 0.95, 1.15];
    cfg.fuzzyPid.kiLevels = [0.85, 1.05, 1.30];
    cfg.fuzzyPid.kdLevels = [0.70, 1.00, 1.35];
    cfg.fuzzyPid.boostLevels = [1.00, 1.12, 1.24];
    cfg.fuzzyPid.adaptLookahead = true;   % Tighten lookahead when CTE is large

    %% Uncertainty & Disturbance Scenario Parameters (Default = S0 Ideal)
    cfg.scenario.name = 'S0_ideal';
    % True sensor noise stds
    cfg.scenario.sigmaOdoD = 0.0;           % [m/step]
    cfg.scenario.sigmaOdoTheta = 0.0;       % [rad/step]
    cfg.scenario.sigmaRange = 0.0;          % [m]
    cfg.scenario.sigmaBearing = 0.0;        % [rad]
    cfg.scenario.sigmaEncVel = 0.0;         % Inner encoder vel noise [m/s, rad/s]
    % Systematic geometric mismatch (wheel radius / track width errors)
    cfg.scenario.etaVBase = 1.0;            % True linear scale factor [-]
    cfg.scenario.etaWBase = 1.0;            % True angular scale factor [-]
    cfg.scenario.biasW = 0.0;               % Constant yaw-rate disturbance [rad/s]
    % Slip patch zone: struct with xRange, yRange, slipV, slipW (fractional loss)
    cfg.scenario.slipZone.enabled = false;
    cfg.scenario.slipZone.xRange = [4.0, 9.0];
    cfg.scenario.slipZone.yRange = [5.0, 9.5];
    cfg.scenario.slipZone.slipV = 0.25;     % 25% linear slip loss
    cfg.scenario.slipZone.slipW = 0.30;     % 30% angular slip loss
    % Mid-run payload change on FOPDT actuator (activated when t >= loadStepTime)
    cfg.scenario.loadStep.enabled = false;
    cfg.scenario.loadStep.time = 20.0;      % [s]
    cfg.scenario.loadStep.K = 0.70;         % Reduced gain under heavy load
    cfg.scenario.loadStep.T = 0.65;         % Slower time constant [s]
    cfg.scenario.loadStep.tau = 0.10;       % Dead-time [s]
end
