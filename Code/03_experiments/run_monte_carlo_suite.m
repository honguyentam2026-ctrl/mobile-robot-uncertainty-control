%% Monte Carlo Evaluation Suite across Scenarios S0..S5 (N = 30 Seeds)
% Runs C1_noisy, C2_comp, C3_comp_pidf, and C4_comp_fuzzy_pidf across S1..S5
% using 30 shared RNG seeds per scenario, plus S0_ideal baseline (C0_ideal).
% Exports CSV summary tables, MAT data, boxplots, and S5 diagnostic plots
% to Code/03_experiments/results/.

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

modes = {'C1_noisy', 'C2_comp', 'C3_comp_pidf', 'C4_comp_fuzzy_pidf', 'C5_ekf_fuzzy_pidf'};
modeLabels = {'C1 Noisy', 'C2 CompFilter', 'C3 Comp+PIDF', 'C4 Comp+FuzzyPID', 'C5 EKF+FuzzyPID'};

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
wTV         = nan(numScen, numModes, numSeeds);

repS5 = cell(1, numModes);

% Reuse cached C1..C4 results if available with identical seeds
matPath = fullfile(resultsDir, 'mc_suite_results.mat');
startMode = 1;
if exist(matPath, 'file')
    prev = load(matPath);
    if isfield(prev, 'cteRmse') && size(prev.cteRmse, 1) == numScen && ...
       size(prev.cteRmse, 3) == numSeeds && size(prev.cteRmse, 2) >= 4
        cteRmse(:, 1:4, :)     = prev.cteRmse(:, 1:4, :);
        cteMax(:, 1:4, :)      = prev.cteMax(:, 1:4, :);
        compTime(:, 1:4, :)    = prev.compTime(:, 1:4, :);
        goalReached(:, 1:4, :) = prev.goalReached(:, 1:4, :);
        estPosRmse(:, 1:4, :)  = prev.estPosRmse(:, 1:4, :);
        estHeadRmse(:, 1:4, :) = prev.estHeadRmse(:, 1:4, :);
        wTV(:, 1:4, :)         = prev.wTV(:, 1:4, :);
        for m = 1:4
            repS5{m} = prev.repS5{m};
        end
        startMode = 5;
        fprintf('Loaded cached C1..C4 Monte Carlo results; running C5_ekf_fuzzy_pidf...\n');
    end
end

tStart = tic;
for s = 1:numScen
    cfgBase = scenario_presets(scenarios{s});
    tScen = tic;

    for m = startMode:numModes
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
colWtvMean    = zeros(nRows, 1);

row = 0;
fprintf('=== Monte Carlo Summary (Mean +/- Std, N = %d seeds) ===\n', numSeeds);
fprintf('%-12s | %-16s | %6s | %11s | %15s | %15s | %7s\n', ...
    'Scenario', 'Controller', 'Goal%', 'Time [s]', 'CTE_RMSE [m]', 'Est_RMSE [m]', 'w_TV');
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
        colWtvMean(row)  = mean(wTV(s, m, :));

        fprintf('%-12s | %-16s | %5.1f%% | %5.2f+-%4.2f | %6.4f+-%6.4f | %6.4f+-%6.4f | %7.1f\n', ...
            scenShort{s}, modeLabels{m}, colGoalPct(row), ...
            colTimeMean(row), colTimeStd(row), ...
            colCteMean(row), colCteStd(row), ...
            colEstMean(row), colEstStd(row), ...
            colWtvMean(row));
    end
    fprintf('%s\n', repmat('-', 1, 95));
end

summaryTbl = table(colScen, colMode, colGoalPct, colTimeMean, colTimeStd, ...
    colCteMean, colCteStd, colMaxMean, colMaxStd, colEstMean, colEstStd, colWtvMean, ...
    'VariableNames', {'Scenario', 'Controller', 'GoalRatePct', ...
    'TimeMean_s', 'TimeStd_s', 'CTE_RmseMean_m', 'CTE_RmseStd_m', ...
    'CTE_MaxMean_m', 'CTE_MaxStd_m', 'EstRmseMean_m', 'EstRmseStd_m', ...
    'OmegaTotalVarMean'});

writetable(summaryTbl, fullfile(resultsDir, 'mc_summary_table.csv'));

%% 5. Figure 1: Monte Carlo Boxplots (CTE RMSE & Completion Time)
figVisible = 'on';
if batchStartupOptionUsed
    figVisible = 'off';
end

fig1 = figure('Visible', figVisible, 'Position', [70 70 1280 500], 'Color', 'w');
colors4 = [0.85 0.55 0.10; 0.49 0.18 0.56; 0.15 0.62 0.25; 0.00 0.45 0.74];

subplot(1, 2, 1); hold on; grid on;
offsets = [-0.27, -0.09, 0.09, 0.27];
boxWidth = 0.15;
for mIdx = 2:5
    dataMat = squeeze(cteRmse(:, mIdx, :))'; % [numSeeds x numScen]
    pos = (1:numScen) + offsets(mIdx - 1);
    b = boxplot(dataMat, 'Positions', pos, 'Widths', boxWidth, ...
        'Colors', colors4(mIdx - 1, :), 'Symbol', '+');
    set(b, 'LineWidth', 1.2);
end
yline(resS0.metrics.rmseCrossTrack, 'k--', 'LineWidth', 1.4);
hLeg = gobjects(5, 1);
hLeg(1) = plot(NaN, NaN, 's', 'Color', colors4(1,:), 'MarkerFaceColor', colors4(1,:), 'DisplayName', 'C2 CompFilter');
hLeg(2) = plot(NaN, NaN, 's', 'Color', colors4(2,:), 'MarkerFaceColor', colors4(2,:), 'DisplayName', 'C3 Comp+PIDF');
hLeg(3) = plot(NaN, NaN, 's', 'Color', colors4(3,:), 'MarkerFaceColor', colors4(3,:), 'DisplayName', 'C4 Comp+FuzzyPID');
hLeg(4) = plot(NaN, NaN, 's', 'Color', colors4(4,:), 'MarkerFaceColor', colors4(4,:), 'DisplayName', 'C5 EKF+FuzzyPID');
hLeg(5) = plot(NaN, NaN, 'k--', 'LineWidth', 1.4, 'DisplayName', 'C0 Ideal Baseline');
set(gca, 'XTick', 1:numScen, 'XTickLabel', scenShort);
xlim([0.4, numScen + 0.6]);
ylim([0, max(cteRmse(:, 2:5, :), [], 'all') * 1.15]);
ylabel('Cross-Track RMSE [m]');
title(sprintf('Cross-Track RMSE (N=%d seeds; C1 diverges 1.2-3.7 m)', numSeeds));
legend(hLeg, 'Location', 'northwest');

subplot(1, 2, 2); hold on; grid on;
timeMeans = squeeze(mean(compTime(:, 2:5, :), 3));
hb = bar(timeMeans);
for k = 1:4
    hb(k).FaceColor = colors4(k, :);
end
yline(resS0.metrics.completionTime, 'k--', 'LineWidth', 1.5, 'DisplayName', 'C0 Ideal Time');
set(gca, 'XTick', 1:numScen, 'XTickLabel', scenShort);
ylim([0 115]);
ylabel('Mean Completion Time [s]');
title('Completion Time Comparison (Lower is Faster)');
legend([hb(1), hb(2), hb(3), hb(4)], {'C2 CompFilter', 'C3 Comp+PIDF', 'C4 Comp+FuzzyPID', 'C5 EKF+FuzzyPID'}, ...
    'Location', 'northwest');

exportgraphics(fig1, fullfile(resultsDir, 'mc_cte_and_time.png'), 'Resolution', 200);

%% 6. Figure 2: Representative S5 Run (Trajectories, CTE, Est Error, Fuzzy Gains)
fig2 = figure('Visible', figVisible, 'Position', [70 70 1280 760], 'Color', 'w');

% (a) Trajectories C0, C2, C3, C4, C5
subplot(2, 2, 1); hold on; grid on; axis equal;
cfgS5 = scenario_presets('S5_worst_case');
sx = cfgS5.scenario.slipZone.xRange; sy = cfgS5.scenario.slipZone.yRange;
patch([sx(1) sx(2) sx(2) sx(1)], [sy(1) sy(1) sy(2) sy(2)], [0.85 0.93 1.0], ...
    'EdgeColor', [0.4 0.7 0.9], 'FaceAlpha', 0.45, 'DisplayName', 'Slip Zone');
plot(cfgS5.landmarks(:,1), cfgS5.landmarks(:,2), 'k^', 'MarkerSize', 5, ...
    'MarkerFaceColor', [0.85 0.85 0.85], 'DisplayName', 'Landmarks');
plot(resS0.poseTrue(:,1), resS0.poseTrue(:,2), 'k--', 'LineWidth', 1.4, 'DisplayName', 'C0 Ideal');
for mIdx = 2:5
    r = repS5{mIdx};
    plot(r.poseTrue(:,1), r.poseTrue(:,2), 'LineWidth', 1.6, ...
        'Color', colors4(mIdx - 1, :), 'DisplayName', modeLabels{mIdx});
end
xlim([0 14]); ylim([0 13]);
xlabel('x [m]'); ylabel('y [m]');
title(sprintf('S5 Worst-Case Trajectories (seed=%d)', seeds(1)));
legend('Location', 'southeast', 'FontSize', 7.5);

% (b) Zoomed CTE vs Time (C0, C2, C3, C4, C5)
subplot(2, 2, 2); hold on; grid on;
plot(resS0.t, resS0.metrics.crossTrackError, 'k--', 'LineWidth', 1.3, 'DisplayName', 'C0 Ideal');
for mIdx = 2:5
    r = repS5{mIdx};
    plot(r.t, r.metrics.crossTrackError, 'LineWidth', 1.5, ...
        'Color', colors4(mIdx - 1, :), 'DisplayName', modeLabels{mIdx});
end
xline(cfgS5.scenario.loadStep.time, ':k', 'Load Step', 'HandleVisibility', 'off');
xlabel('Time [s]'); ylabel('Cross-Track Error [m]');
title('S5 Cross-Track Error Comparison (C2..C5)');
legend('Location', 'northeast', 'FontSize', 8);

% (c) S5 Localization Position Error (C2..C5)
subplot(2, 2, 3); hold on; grid on;
for mIdx = 2:5
    r = repS5{mIdx};
    posErr = hypot(r.poseEst(:, 1) - r.poseTrue(:, 1), r.poseEst(:, 2) - r.poseTrue(:, 2));
    plot(r.t, posErr, 'Color', colors4(mIdx - 1, :), 'LineWidth', 1.4, ...
        'DisplayName', modeLabels{mIdx});
end
xlabel('Time [s]'); ylabel('Pose Estimation Error [m]');
title('S5 Position Estimation Error (CompFilter C2..C4 vs EKF C5)');
legend('Location', 'northeast', 'FontSize', 8);

% (d) C4 Online Fuzzy-PID Gain Adaptation on S5
subplot(2, 2, 4); hold on; grid on;
r4 = repS5{4};
t4 = r4.t(1:end-1);
plot(t4, r4.fuzzyGains(1:end-1, 1), 'b-',  'LineWidth', 1.5, 'DisplayName', 's_{Kp}');
plot(t4, r4.fuzzyGains(1:end-1, 2), 'r-',  'LineWidth', 1.5, 'DisplayName', 's_{Ki}');
plot(t4, r4.fuzzyGains(1:end-1, 3), 'g-',  'LineWidth', 1.5, 'DisplayName', 's_{Kd}');
plot(t4, r4.fuzzyGains(1:end-1, 4), 'm--', 'LineWidth', 1.5, 'DisplayName', 'Slip Boost');
plot(r4.t, r4.slipEst, 'k:', 'LineWidth', 1.4, 'DisplayName', 'Est. Slip');
xline(cfgS5.scenario.loadStep.time, ':k', 'Load Step', 'HandleVisibility', 'off');
xlabel('Time [s]'); ylabel('Multiplier / Indicator [-]');
ylim([0 2.0]);
title('C4 Online Fuzzy-PID Gain Adaptation on S5');
legend('Location', 'best', 'FontSize', 8);

exportgraphics(fig2, fullfile(resultsDir, 'mc_s5_detailed_analysis.png'), 'Resolution', 200);

%% 7. Save Full Workspace Results
save(fullfile(resultsDir, 'mc_suite_results.mat'), ...
    'scenarios', 'scenShort', 'modes', 'modeLabels', 'seeds', 'resS0', ...
    'cteRmse', 'cteMax', 'compTime', 'goalReached', 'estPosRmse', ...
    'estHeadRmse', 'wTV', 'summaryTbl', 'repS5');

fprintf('Saved Monte Carlo results to: %s\n', resultsDir);
