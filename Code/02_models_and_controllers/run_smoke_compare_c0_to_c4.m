%% Smoke Test & Preliminary Comparison of C0..C4 (Non-Kalman Architecture)
% Runs modes C0_ideal through C4_comp_fuzzy_pidf on a stressful warehouse
% scenario (odometry + landmark noise, wheel radius/track mismatch, floor
% slip patch, and mid-run payload step change) using the same RNG seed.

clear; clc; close all;

scriptDir = fileparts(mfilename('fullpath'));
codeRoot = fileparts(scriptDir);
addpath(scriptDir);
addpath(fullfile(codeRoot, 'utils'));

resultsDir = fullfile(scriptDir, 'results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

%% 1. Configure Stress Test Scenario (Noise + Mismatch + Slip Zone + Load Step)
cfg = default_sim_config();
cfg.seed = 2026;
cfg.scenario.name = 'Smoke_Combined_Uncertainty';

% Sensor noises
cfg.scenario.sigmaOdoD = 0.02;            % [m/step]
cfg.scenario.sigmaOdoTheta = deg2rad(1.5);% [rad/step]
cfg.scenario.sigmaRange = 0.08;           % [m]
cfg.scenario.sigmaBearing = deg2rad(2.5); % [rad]
cfg.scenario.sigmaEncVel = 0.005;         % inner encoder noise

% Systematic geometric mismatch (-15% effective linear, -20% effective yaw)
cfg.scenario.etaVBase = 0.85;
cfg.scenario.etaWBase = 0.80;

% Wet floor slip patch across middle warehouse corridor
cfg.scenario.slipZone.enabled = true;
cfg.scenario.slipZone.xRange = [4.0, 9.5];
cfg.scenario.slipZone.yRange = [5.0, 9.8];
cfg.scenario.slipZone.slipV = 0.25;
cfg.scenario.slipZone.slipW = 0.25;

% Mid-run payload pickup at t = 20 s
cfg.scenario.loadStep.enabled = true;
cfg.scenario.loadStep.time = 20.0;
cfg.scenario.loadStep.K = 0.70;
cfg.scenario.loadStep.T = 0.65;
cfg.scenario.loadStep.tau = 0.10;

%% 2. Run C0..C4
modes = {'C0_ideal', 'C1_noisy', 'C2_comp', 'C3_comp_pidf', 'C4_comp_fuzzy_pidf'};
labels = {'C0 Ideal', 'C1 Noisy Odo', 'C2 CompFilter', 'C3 Comp+PIDF', 'C4 Comp+FuzzyPID'};
results = cell(size(modes));

fprintf('=== Smoke Comparison on %s (seed=%d) ===\n', cfg.scenario.name, cfg.seed);
fprintf('%-17s | %5s | %7s | %8s | %8s | %8s | %7s\n', ...
    'Controller', 'Goal', 'Time[s]', 'CTE_RMSE', 'CTE_Max', 'Est_RMSE', 'w_TV');
fprintf('%s\n', repmat('-', 1, 78));

for m = 1:numel(modes)
    cfgRun = cfg;
    if strcmp(modes{m}, 'C0_ideal')
        cfgIdeal = default_sim_config();
        cfgIdeal.seed = cfg.seed;
        res = simulate_closed_loop(modes{m}, cfgIdeal);
    else
        res = simulate_closed_loop(modes{m}, cfgRun);
    end
    results{m} = res;

    fprintf('%-17s | %5d | %7.2f | %8.4f | %8.4f | %8.4f | %7.2f\n', ...
        labels{m}, res.metrics.reachedGoal, res.metrics.completionTime, ...
        res.metrics.rmseCrossTrack, res.metrics.maxCrossTrack, ...
        res.estMetrics.posRmse, res.metrics.wTotalVariation);
end

%% 3. Diagnostic Plot (Trajectories, Cross-Track Error, Fuzzy-PID Adaptation)
figVisible = 'on';
if batchStartupOptionUsed
    figVisible = 'off';
end

fig = figure('Visible', figVisible, 'Position', [80 80 1280 430], 'Color', 'w');
colors = lines(numel(modes));

% Panel 1: Warehouse Trajectories + Slip Patch + Landmarks
subplot(1, 3, 1); hold on; grid on; axis equal;
sx = cfg.scenario.slipZone.xRange;
sy = cfg.scenario.slipZone.yRange;
patch([sx(1) sx(2) sx(2) sx(1)], [sy(1) sy(1) sy(2) sy(2)], [0.85 0.93 1.0], ...
    'EdgeColor', [0.4 0.7 0.9], 'FaceAlpha', 0.45, 'DisplayName', 'Slip Zone');
plot(cfg.landmarks(:, 1), cfg.landmarks(:, 2), 'k^', 'MarkerSize', 6, ...
    'MarkerFaceColor', [0.8 0.8 0.8], 'DisplayName', 'Landmarks');
plot(cfg.path(:, 1), cfg.path(:, 2), 'k--o', 'LineWidth', 1.3, 'MarkerSize', 4, ...
    'DisplayName', 'Ref Path');
for m = 1:numel(modes)
    r = results{m};
    plot(r.poseTrue(:, 1), r.poseTrue(:, 2), 'LineWidth', 1.6, ...
        'Color', colors(m, :), 'DisplayName', labels{m});
end
xlim([-0.5 14]); ylim([-0.5 13]);
xlabel('x [m]'); ylabel('y [m]');
title('True Trajectories (Non-Kalman C0..C4)');
legend('Location', 'southeast', 'FontSize', 7.5);

% Panel 2: Cross-Track Error vs Time
subplot(1, 3, 2); hold on; grid on;
for m = 1:numel(modes)
    r = results{m};
    plot(r.t, r.metrics.crossTrackError, 'LineWidth', 1.5, ...
        'Color', colors(m, :), 'DisplayName', labels{m});
end
xline(cfg.scenario.loadStep.time, ':k', 'Load Step', 'LabelVerticalAlignment', 'bottom');
xlabel('Time [s]'); ylabel('Cross-Track Error [m]');
title('Path Tracking Error over Time');
legend('Location', 'northeast', 'FontSize', 7.5);

% Panel 3: Online Fuzzy-PID Gain Adaptation (C4)
subplot(1, 3, 3); hold on; grid on;
r4 = results{5};
t4 = r4.t(1:end-1);
plot(t4, r4.fuzzyGains(1:end-1, 1), 'b-',  'LineWidth', 1.5, 'DisplayName', 's_{Kp}');
plot(t4, r4.fuzzyGains(1:end-1, 2), 'r-',  'LineWidth', 1.5, 'DisplayName', 's_{Ki}');
plot(t4, r4.fuzzyGains(1:end-1, 3), 'g-',  'LineWidth', 1.5, 'DisplayName', 's_{Kd}');
plot(t4, r4.fuzzyGains(1:end-1, 4), 'm--', 'LineWidth', 1.5, 'DisplayName', 'Slip Boost');
plot(r4.t, r4.slipEst, 'k:', 'LineWidth', 1.4, 'DisplayName', 'Est. Slip');
xline(cfg.scenario.loadStep.time, ':k', 'Load Step', 'HandleVisibility', 'off');
xlabel('Time [s]'); ylabel('Multiplier / Indicator [-]');
ylim([0 2.4]);
title('C4 Online Fuzzy-PID Gain Scheduling');
legend('Location', 'best', 'FontSize', 8);

exportgraphics(fig, fullfile(resultsDir, 'smoke_compare_c0_to_c4.png'), 'Resolution', 200);
save(fullfile(resultsDir, 'smoke_compare_c0_to_c4.mat'), 'cfg', 'results', 'modes', 'labels');
fprintf('Saved smoke comparison to: %s\n', resultsDir);
