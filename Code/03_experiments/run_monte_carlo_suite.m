%% Monte Carlo Evaluation Suite across Scenarios S0..S5 (N = 30 Seeds)
% Runs C1_noisy, C2_ekf, C3_ekf_simc, and C4_aug_ekf_simc across scenarios
% S1..S5 using 30 shared RNG seeds per scenario, plus S0_ideal baseline
% (C0_ideal). Exports CSV summary tables, MAT data, boxplots, and 2-sigma
% EKF consistency plots to Code/03_experiments/results/.

clear; clc; close all;

expDir = fileparts(mfilename('fullpath'));
codeRoot = fileparts(expDir);
addpath(expDir);
addpath(fullfile(codeRoot, '02_models_and_controllers'));
addpath(fullfile(codeRoot, 'utils'));

resultsDir = fullfile(expDir, 'results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

%% 1. Experiment Settings
numSeeds = 30;
seeds = 1000 + (1:numSeeds);

scenarios = {'S1_sensor_noise', 'S2_wheel_slip', 'S3_model_mismatch', ...
             'S4_load_step', 'S5_worst_case'};
scenShort = {'S1 Noise', 'S2 Slip', 'S3 Mismatch', 'S4 LoadStep', 'S5 Worst'};

modes = {'C1_noisy', 'C2_ekf', 'C3_ekf_simc', 'C4_aug_ekf_simc'};
modeLabels = {'C1 Noisy', 'C2 StdEKF', 'C3 EKF+SIMC', 'C4 AugEKF+SIMC'};

numScen = numel(scenarios);
numModes = numel(modes);

%% 2. Ideal Reference Benchmark S0
cfgS0 = scenario_presets('S0_ideal');
cfgS0.seed = seeds(1);
resS0 = simulate_closed_loop('C0_ideal', cfgS0);

fprintf('=== S0 Ideal Reference ===\n');
fprintf('C0_ideal | Goal=%d | Time=%.2f s | CTE_RMSE=%.4f m | CTE_Max=%.4f m\n\n', ...
    resS0.metrics.reachedGoal, resS0.metrics.completionTime, ...
    resS0.metrics.rmseCrossTrack, resS0.metrics.maxCrossTrack);

%% 3. Preallocate Monte Carlo Metric Arrays [numScen x numModes x numSeeds]
cteRmse     = nan(numScen, numModes, numSeeds);
cteMax      = nan(numScen, numModes, numSeeds);
compTime    = nan(numScen, numModes, numSeeds);
goalReached = nan(numScen, numModes, numSeeds);
estPosRmse  = nan(numScen, numModes, numSeeds);
estHeadRmse = nan(numScen, numModes, numSeeds);
sig2XY      = nan(numScen, numModes, numSeeds);
wTV         = nan(numScen, numModes, numSeeds);

% Keep representative detailed trajectories (seed 1 of S5_worst_case)
repS5 = cell(1, numModes);

tStart = tic;
for s = 1:numScen
    cfgBase = scenario_presets(scenarios{s});
    tScen = tic;

    for m = 1:numModes
        for r = 1:numSeeds
            cfgRun = cfgBase;
            cfgRun.seed = seeds(r);
            res = simulate_closed_loop(modes{m}, cfgRun);

            cteRmse(s, m, r)     = res.metrics.rmseCrossTrack;
            cteMax(s, m, r)      = res.metrics.maxCrossTrack;
            compTime(s, m, r)    = res.metrics.completionTime;
            goalReached(s, m, r) = double(res.metrics.reachedGoal);
            estPosRmse(s, m, r)  = res.estMetrics.posRmse;
            estHeadRmse(s, m, r) = res.estMetrics.headingRmseDeg;
            sig2XY(s, m, r)      = mean(res.estMetrics.within2SigmaPos(1:2)) * 100.0;
            wTV(s, m, r)         = res.metrics.wTotalVariation;

            if strcmp(scenarios{s}, 'S5_worst_case') && r == 1
                repS5{m} = res;
            end
        end
    end

    fprintf('Completed %-18s (%d x %d runs) in %.2f s\n', ...
        scenarios{s}, numModes, numSeeds, toc(tScen));
end
fprintf('Total Monte Carlo time: %.2f s\n\n', toc(tStart));

%% 4. Build Summary Table & Export CSV
nRows = numScen * numModes;
colScen       = cell(nRows, 1);
colMode       = cell(nRows, 1);
colGoalPct    = zeros(nRows, 1);
colTimeMean   = zeros(nRows, 1);
colTimeStd    = zeros(nRows, 1);
colCteMean    = zeros(nRows, 1);
colCteStd     = zeros(nRows, 1);
colMaxMean    = zeros(nRows, 1);
colMaxStd     = zeros(nRows, 1);
colEstMean    = zeros(nRows, 1);
colEstStd     = zeros(nRows, 1);
colSig2Mean   = zeros(nRows, 1);
colSig2Std    = zeros(nRows, 1);
colWtvMean    = zeros(nRows, 1);

row = 0;
fprintf('=== Monte Carlo Summary (Mean +/- Std, N = %d seeds) ===\n', numSeeds);
fprintf('%-12s | %-14s | %6s | %11s | %15s | %15s | %7s\n', ...
    'Scenario', 'Controller', 'Goal%', 'Time [s]', 'CTE_RMSE [m]', 'Est_RMSE [m]', '2sig%');
fprintf('%s\n', repmat('-', 1, 95));

for s = 1:numScen
    for m = 1:numModes
        row = row + 1;
        colScen{row}     = scenarios{s};
        colMode{row}     = modes{m};
        colGoalPct(row)  = mean(goalReached(s, m, :)) * 100.0;
        colTimeMean(row) = mean(compTime(s, m, :));
        colTimeStd(row)  = std(compTime(s, m, :));
        colCteMean(row)  = mean(cteRmse(s, m, :));
        colCteStd(row)   = std(cteRmse(s, m, :));
        colMaxMean(row)  = mean(cteMax(s, m, :));
        colMaxStd(row)   = std(cteMax(s, m, :));
        colEstMean(row)  = mean(estPosRmse(s, m, :));
        colEstStd(row)   = std(estPosRmse(s, m, :));
        colSig2Mean(row) = mean(sig2XY(s, m, :));
        colSig2Std(row)  = std(sig2XY(s, m, :));
        colWtvMean(row)  = mean(wTV(s, m, :));

        fprintf('%-12s | %-14s | %5.1f%% | %5.2f+-%4.2f | %6.4f+-%6.4f | %6.4f+-%6.4f | %6.1f\n', ...
            scenShort{s}, modeLabels{m}, colGoalPct(row), ...
            colTimeMean(row), colTimeStd(row), ...
            colCteMean(row), colCteStd(row), ...
            colEstMean(row), colEstStd(row), ...
            colSig2Mean(row));
    end
    fprintf('%s\n', repmat('-', 1, 95));
end

summaryTbl = table(colScen, colMode, colGoalPct, colTimeMean, colTimeStd, ...
    colCteMean, colCteStd, colMaxMean, colMaxStd, colEstMean, colEstStd, ...
    colSig2Mean, colSig2Std, colWtvMean, ...
    'VariableNames', {'Scenario', 'Controller', 'GoalRatePct', ...
    'TimeMean_s', 'TimeStd_s', 'CTE_RmseMean_m', 'CTE_RmseStd_m', ...
    'CTE_MaxMean_m', 'CTE_MaxStd_m', 'EstRmseMean_m', 'EstRmseStd_m', ...
    'Within2SigmaPct', 'Within2SigmaStd', 'OmegaTotalVarMean'});

writetable(summaryTbl, fullfile(resultsDir, 'mc_summary_table.csv'));

%% 5. Figure 1: Monte Carlo Boxplots (C2..C4 vs S1..S5 + C1 failure summary)
figVisible = 'on';
if batchStartupOptionUsed
    figVisible = 'off';
end

fig1 = figure('Visible', figVisible, 'Position', [70 70 1280 500], 'Color', 'w');
colors3 = [0.85 0.55 0.10; 0.49 0.18 0.56; 0.15 0.62 0.25];

subplot(1, 2, 1); hold on; grid on;
% Plot CTE RMSE distributions for C2, C3, C4 across S1..S5
offsets = [-0.24, 0.0, 0.24];
boxWidth = 0.20;
for mIdx = 2:4
    dataMat = squeeze(cteRmse(:, mIdx, :))'; % [numSeeds x numScen]
    pos = (1:numScen) + offsets(mIdx - 1);
    b = boxplot(dataMat, 'Positions', pos, 'Widths', boxWidth, ...
        'Colors', colors3(mIdx - 1, :), 'Symbol', '+');
    set(b, 'LineWidth', 1.2);
end
yline(resS0.metrics.rmseCrossTrack, 'b--', 'LineWidth', 1.4, ...
    'DisplayName', 'C0 Ideal S0');
% Dummy legend handles for C2..C4
hLeg = gobjects(4, 1);
hLeg(1) = plot(NaN, NaN, 's', 'Color', colors3(1,:), 'MarkerFaceColor', colors3(1,:), 'DisplayName', 'C2 StdEKF');
hLeg(2) = plot(NaN, NaN, 's', 'Color', colors3(2,:), 'MarkerFaceColor', colors3(2,:), 'DisplayName', 'C3 EKF+SIMC');
hLeg(3) = plot(NaN, NaN, 's', 'Color', colors3(3,:), 'MarkerFaceColor', colors3(3,:), 'DisplayName', 'C4 AugEKF+SIMC');
hLeg(4) = plot(NaN, NaN, 'b--', 'LineWidth', 1.4, 'DisplayName', 'C0 Ideal Baseline');
set(gca, 'XTick', 1:numScen, 'XTickLabel', scenShort);
xlim([0.4, numScen + 0.6]);
ylim([0, max(cteRmse(:, 2:4, :), [], 'all') * 1.15]);
ylabel('Cross-Track RMSE [m]');
title(sprintf('Cross-Track RMSE (N=%d seeds; C1 diverges 1.2-3.7 m)', numSeeds));
legend(hLeg, 'Location', 'northwest');

subplot(1, 2, 2); hold on; grid on;
% Grouped bar chart of EKF 2-sigma consistency (C2, C3, C4)
sigMeans = squeeze(mean(sig2XY(:, 2:4, :), 3));
hb = bar(sigMeans);
for k = 1:3
    hb(k).FaceColor = colors3(k, :);
end
yline(95.0, 'r--', 'LineWidth', 1.5, 'DisplayName', '95% Nominal Consistency');
set(gca, 'XTick', 1:numScen, 'XTickLabel', scenShort);
ylim([50 105]);
ylabel('Samples Within \pm2\sigma Bound [%]');
title('Estimator Covariance Consistency (95% Target)');
legend([hb(1), hb(2), hb(3)], {'C2 StdEKF', 'C3 EKF+SIMC', 'C4 AugEKF+SIMC'}, ...
    'Location', 'southwest');

exportgraphics(fig1, fullfile(resultsDir, 'mc_cte_and_consistency.png'), 'Resolution', 200);

%% 6. Figure 2: Representative S5 Run (Trajectories, Zoomed CTE, 2-sigma Bounds)
fig2 = figure('Visible', figVisible, 'Position', [70 70 1280 760], 'Color', 'w');

% (a) Trajectories C0, C2, C3, C4
subplot(2, 2, 1); hold on; grid on; axis equal;
cfgS5 = scenario_presets('S5_worst_case');
sx = cfgS5.scenario.slipZone.xRange; sy = cfgS5.scenario.slipZone.yRange;
patch([sx(1) sx(2) sx(2) sx(1)], [sy(1) sy(1) sy(2) sy(2)], [0.85 0.93 1.0], ...
    'EdgeColor', [0.4 0.7 0.9], 'FaceAlpha', 0.45, 'DisplayName', 'Slip Zone');
plot(cfgS5.landmarks(:,1), cfgS5.landmarks(:,2), 'k^', 'MarkerSize', 5, ...
    'MarkerFaceColor', [0.85 0.85 0.85], 'DisplayName', 'Landmarks');
plot(resS0.poseTrue(:,1), resS0.poseTrue(:,2), 'b--', 'LineWidth', 1.4, 'DisplayName', 'C0 Ideal');
for mIdx = 2:4
    r = repS5{mIdx};
    plot(r.poseTrue(:,1), r.poseTrue(:,2), 'LineWidth', 1.6, ...
        'Color', colors3(mIdx - 1, :), 'DisplayName', modeLabels{mIdx});
end
xlim([0 14]); ylim([0 13]);
xlabel('x [m]'); ylabel('y [m]');
title(sprintf('S5 Worst-Case Trajectories (seed=%d)', seeds(1)));
legend('Location', 'southeast', 'FontSize', 7.5);

% (b) Zoomed CTE vs Time (C0, C2, C3, C4)
subplot(2, 2, 2); hold on; grid on;
plot(resS0.t, resS0.metrics.crossTrackError, 'b--', 'LineWidth', 1.3, 'DisplayName', 'C0 Ideal');
for mIdx = 2:4
    r = repS5{mIdx};
    plot(r.t, r.metrics.crossTrackError, 'LineWidth', 1.5, ...
        'Color', colors3(mIdx - 1, :), 'DisplayName', modeLabels{mIdx});
end
xline(cfgS5.scenario.loadStep.time, ':k', 'Load Step', 'HandleVisibility', 'off');
xlabel('Time [s]'); ylabel('Cross-Track Error [m]');
title('S5 Cross-Track Error Comparison (C2..C4)');
legend('Location', 'northeast', 'FontSize', 8);

% (c) S5 Estimation Error with +-2sigma Envelope: C3 StdEKF vs C4 AugEKF
subplot(2, 2, 3); hold on; grid on;
r3 = repS5{3};
r4 = repS5{4};
errX3 = r3.poseEst(:, 1) - r3.poseTrue(:, 1);
errX4 = r4.poseEst(:, 1) - r4.poseTrue(:, 1);
sig2X3 = 2.0 * sqrt(r3.pDiag(:, 1));
sig2X4 = 2.0 * sqrt(r4.pDiag(:, 1));
plot(r3.t, errX3, 'Color', colors3(2,:), 'LineWidth', 1.2, 'DisplayName', 'C3 x-err');
plot(r3.t,  sig2X3, '--', 'Color', colors3(2,:), 'HandleVisibility', 'off');
plot(r3.t, -sig2X3, '--', 'Color', colors3(2,:), 'HandleVisibility', 'off');
plot(r4.t, errX4, 'Color', colors3(3,:), 'LineWidth', 1.4, 'DisplayName', 'C4 x-err');
plot(r4.t,  sig2X4, ':', 'Color', colors3(3,:), 'LineWidth', 1.3, 'DisplayName', 'C4 \pm2\sigma');
plot(r4.t, -sig2X4, ':', 'Color', colors3(3,:), 'LineWidth', 1.3, 'HandleVisibility', 'off');
xlabel('Time [s]'); ylabel('X Estimation Error [m]');
title('S5 Consistency Check (x-error vs \pm2\sigma)');
legend('Location', 'northeast', 'FontSize', 8);

% (d) C4 Online Scale/Slip Parameter Adaptation on S5
subplot(2, 2, 4); hold on; grid on;
plot(r4.t, r4.etaTrue(:, 1), 'b--', 'LineWidth', 1.3, 'DisplayName', '\eta_v true');
plot(r4.t, r4.etaEst(:, 1),  'b-',  'LineWidth', 1.6, 'DisplayName', '\eta_v est');
plot(r4.t, r4.etaTrue(:, 2), 'r--', 'LineWidth', 1.3, 'DisplayName', '\eta_\omega true');
plot(r4.t, r4.etaEst(:, 2),  'r-',  'LineWidth', 1.6, 'DisplayName', '\eta_\omega est');
xlabel('Time [s]'); ylabel('Scale Factor [-]');
ylim([0.4 1.3]);
title(sprintf('C4 Online Scale Adaptation (RMSE=%.3f, %.3f)', ...
    r4.estMetrics.etaRmse(1), r4.estMetrics.etaRmse(2)));
legend('Location', 'best', 'FontSize', 8);

exportgraphics(fig2, fullfile(resultsDir, 'mc_s5_detailed_analysis.png'), 'Resolution', 200);

%% 7. Save Full Workspace Results
save(fullfile(resultsDir, 'mc_suite_results.mat'), ...
    'scenarios', 'scenShort', 'modes', 'modeLabels', 'seeds', 'resS0', ...
    'cteRmse', 'cteMax', 'compTime', 'goalReached', 'estPosRmse', ...
    'estHeadRmse', 'sig2XY', 'wTV', 'summaryTbl', 'repS5');

fprintf('Saved Monte Carlo results to: %s\n', resultsDir);
