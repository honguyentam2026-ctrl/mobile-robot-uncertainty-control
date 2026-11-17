%% Parameter Sensitivity Analysis (Lookahead Distance & Complementary Gain)
% Study A: Pure Pursuit LookaheadDistance sweep (0.25 .. 1.40 m) comparing
%          corner-cutting (CTE RMSE) vs steering smoothness (wTotalVariation)
%          on S0_ideal and S5_worst_case (10 seeds for C4_comp_fuzzy_pidf).
% Study B: Complementary Filter position blend gain alphaPos sweep
%          (0.10 .. 0.85) on S1_sensor_noise (10 seeds for C2_comp),
%          illustrating localization error vs steering-jitter trade-off.

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

numSeeds = 10;
seeds = 2000 + (1:numSeeds);

%% ========================================================================
%  STUDY A: Pure Pursuit LookaheadDistance Sweep
%  ========================================================================
lookaheadList = [0.25, 0.35, 0.50, 0.75, 1.00, 1.40];
nLA = numel(lookaheadList);

laCteIdeal = zeros(nLA, 1);
laWtvIdeal = zeros(nLA, 1);
laTimeIdeal = zeros(nLA, 1);

laCteC4Mean = zeros(nLA, 1);
laCteC4Std  = zeros(nLA, 1);
laWtvC4Mean = zeros(nLA, 1);
laWtvC4Std  = zeros(nLA, 1);
laTimeC4Mean = zeros(nLA, 1);

fprintf('=== Study A: Pure Pursuit LookaheadDistance Sweep ===\n');
fprintf('%-7s | %10s | %10s | %15s | %15s | %8s\n', ...
    'd_L[m]', 'S0_CTE[m]', 'S0_wTV', 'S5_C4_CTE[m]', 'S5_C4_wTV', 'S5_Time');
fprintf('%s\n', repmat('-', 1, 78));

for i = 1:nLA
    dL = lookaheadList(i);

    % S0 deterministic
    cfg0 = scenario_presets('S0_ideal');
    cfg0.pp.LookaheadDistance = dL;
    r0 = simulate_closed_loop('C0_ideal', cfg0);
    laCteIdeal(i) = r0.metrics.rmseCrossTrack;
    laWtvIdeal(i) = r0.metrics.wTotalVariation;
    laTimeIdeal(i) = r0.metrics.completionTime;

    % S5 worst case with C4_comp_fuzzy_pidf across seeds
    cteBuf = zeros(numSeeds, 1);
    wtvBuf = zeros(numSeeds, 1);
    tBuf   = zeros(numSeeds, 1);
    for r = 1:numSeeds
        cfg5 = scenario_presets('S5_worst_case');
        cfg5.pp.LookaheadDistance = dL;
        cfg5.seed = seeds(r);
        r5 = simulate_closed_loop('C4_comp_fuzzy_pidf', cfg5);
        cteBuf(r) = r5.metrics.rmseCrossTrack;
        wtvBuf(r) = r5.metrics.wTotalVariation;
        tBuf(r)   = r5.metrics.completionTime;
    end

    laCteC4Mean(i)  = mean(cteBuf);
    laCteC4Std(i)   = std(cteBuf);
    laWtvC4Mean(i)  = mean(wtvBuf);
    laWtvC4Std(i)   = std(wtvBuf);
    laTimeC4Mean(i) = mean(tBuf);

    fprintf('%-7.2f | %10.4f | %10.2f | %6.4f+-%6.4f | %6.2f+-%6.2f | %8.2f\n', ...
        dL, laCteIdeal(i), laWtvIdeal(i), ...
        laCteC4Mean(i), laCteC4Std(i), laWtvC4Mean(i), laWtvC4Std(i), laTimeC4Mean(i));
end

tblLookahead = table(lookaheadList(:), laCteIdeal, laWtvIdeal, laTimeIdeal, ...
    laCteC4Mean, laCteC4Std, laWtvC4Mean, laWtvC4Std, laTimeC4Mean, ...
    'VariableNames', {'Lookahead_m', 'S0_CTE_m', 'S0_wTV', 'S0_Time_s', ...
    'S5_C4_CTE_Mean_m', 'S5_C4_CTE_Std_m', 'S5_C4_wTV_Mean', 'S5_C4_wTV_Std', 'S5_C4_Time_s'});
writetable(tblLookahead, fullfile(resultsDir, 'param_lookahead_table.csv'));

%% ========================================================================
%  STUDY B: Complementary Filter Blend Gain alphaPos Sweep on S1_sensor_noise
%  ========================================================================
alphas = [0.10, 0.25, 0.45, 0.65, 0.85];
nA = numel(alphas);

compPosRmseMean = zeros(nA, 1);
compPosRmseStd  = zeros(nA, 1);
compWtvMean     = zeros(nA, 1);
compWtvStd      = zeros(nA, 1);

fprintf('\n=== Study B: Complementary Filter Gain alphaPos Sweep on S1_sensor_noise ===\n');
fprintf('%-10s | %15s | %15s\n', ...
    'alphaPos', 'Est_RMSE [m]', 'w_TV [rad/s]');
fprintf('%s\n', repmat('-', 1, 46));

for i = 1:nA
    aPos = alphas(i);
    posBuf = zeros(numSeeds, 1);
    wtvBuf = zeros(numSeeds, 1);

    for r = 1:numSeeds
        cfg1 = scenario_presets('S1_sensor_noise');
        cfg1.seed = seeds(r);
        cfg1.compFilter.alphaPos = aPos;

        res = simulate_closed_loop('C2_comp', cfg1);
        posBuf(r) = res.estMetrics.posRmse;
        wtvBuf(r) = res.metrics.wTotalVariation;
    end

    compPosRmseMean(i) = mean(posBuf);
    compPosRmseStd(i)  = std(posBuf);
    compWtvMean(i)     = mean(wtvBuf);
    compWtvStd(i)      = std(wtvBuf);

    fprintf('%-10.2f | %6.4f+-%6.4f | %6.2f+-%6.2f\n', ...
        aPos, compPosRmseMean(i), compPosRmseStd(i), compWtvMean(i), compWtvStd(i));
end

tblCompGain = table(alphas(:), compPosRmseMean, compPosRmseStd, compWtvMean, compWtvStd, ...
    'VariableNames', {'AlphaPos', 'EstRmseMean_m', 'EstRmseStd_m', 'wTV_Mean', 'wTV_Std'});
writetable(tblCompGain, fullfile(resultsDir, 'param_comp_gain_table.csv'));

%% ========================================================================
%  Plot Parameter Sweep Results
%  ========================================================================
figVisible = 'on';
if batchStartupOptionUsed
    figVisible = 'off';
end

fig = figure('Visible', figVisible, 'Position', [90 90 1200 450], 'Color', 'w');

subplot(1, 2, 1);
yyaxis left;
errorbar(lookaheadList, laCteC4Mean, laCteC4Std, '-o', 'LineWidth', 1.6, ...
    'DisplayName', 'S5 C4 CTE RMSE');
hold on; grid on;
plot(lookaheadList, laCteIdeal, '--s', 'LineWidth', 1.3, ...
    'DisplayName', 'S0 Ideal CTE RMSE');
ylabel('Cross-Track RMSE [m]');
yyaxis right;
errorbar(lookaheadList, laWtvC4Mean, laWtvC4Std, '-^', 'LineWidth', 1.5, ...
    'DisplayName', 'S5 C4 \Sigma|\Delta\omega|');
ylabel('Steering Total Variation [rad/s]');
xlabel('Lookahead Distance d_L [m]');
title('Study A: Corner-Cutting vs. Steering Oscillation Trade-Off');
legend('Location', 'north', 'FontSize', 8);

subplot(1, 2, 2);
yyaxis left;
errorbar(alphas, compPosRmseMean, compPosRmseStd, '-o', 'LineWidth', 1.8, ...
    'DisplayName', 'Est Pos RMSE [m]');
hold on; grid on;
ylabel('Localization Position RMSE [m]');
yyaxis right;
errorbar(alphas, compWtvMean, compWtvStd, '-s', 'LineWidth', 1.6, ...
    'DisplayName', 'Steering TV \Sigma|\Delta\omega|');
ylabel('Steering Total Variation [rad/s]');
xlabel('Complementary Filter Position Gain \alpha_{pos}');
title('Study B: Localization Accuracy vs. Landmark-Step Jitter');
legend('Location', 'north', 'FontSize', 8);

exportgraphics(fig, fullfile(resultsDir, 'param_sweeps_analysis.png'), 'Resolution', 200);
save(fullfile(resultsDir, 'param_sweeps_results.mat'), 'tblLookahead', 'tblCompGain');
fprintf('\nSaved parameter sweep outputs to: %s\n', resultsDir);
