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

    %% EKF Filter Tuning Covariances (Vhat, What, P0)
    cfg.ekf.P0 = diag([0.05^2, 0.05^2, (2*pi/180)^2]);
    cfg.ekf.Vhat = diag([0.02^2, (1.5*pi/180)^2]);  % Assumed odo noise [m^2, rad^2]
    cfg.ekf.What = diag([0.08^2, (2.5*pi/180)^2]);  % Assumed landmark noise [m^2, rad^2]
    cfg.ekf.gateChi2 = chi2inv(0.99, 2);            % Mahalanobis gating threshold (99%)
    cfg.ekf.useGating = true;

    %% Augmented EKF (5-state: [x; y; theta; eta_v; eta_w]) Parameters
    cfg.augEkf.eta0 = [1.0; 1.0];
    cfg.augEkf.P0Eta = diag([0.25^2, 0.25^2]);
    % Random-walk process noise on [eta_v; eta_w] per active step
    cfg.augEkf.qEta = diag([0.015^2, 0.025^2]);
    cfg.augEkf.etaBounds = [0.40, 1.60];
    cfg.augEkf.compensateControl = true;

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
