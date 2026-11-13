%% Parameter Sensitivity Analysis (Pure Pursuit Lookahead & EKF Vhat/V Ratio)
% Study A: Pure Pursuit LookaheadDistance sweep (0.25 .. 1.40 m) comparing
%          corner-cutting (CTE RMSE) vs steering smoothness (wTotalVariation)
%          on S0_ideal and S5_worst_case (10 seeds for C4_aug_ekf_simc).
% Study B: EKF assumed process noise covariance ratio Vhat / V_true
%          (alpha = 0.10 .. 16.0) on S1_sensor_noise (10 seeds for C2_ekf),
%          directly verifying overconfident vs well-tuned vs pessimistic EKF
%          behavior (Slide Chapter 4_part 2, p.22).

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

    % S5 worst case with C4_aug_ekf_simc across seeds
    cteBuf = zeros(numSeeds, 1);
    wtvBuf = zeros(numSeeds, 1);
    tBuf   = zeros(numSeeds, 1);
    for r = 1:numSeeds
        cfg5 = scenario_presets('S5_worst_case');
        cfg5.pp.LookaheadDistance = dL;
        cfg5.seed = seeds(r);
        r5 = simulate_closed_loop('C4_aug_ekf_simc', cfg5);
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
%  STUDY B: EKF Assumed Odometry Covariance Ratio Vhat / V_true
%  ========================================================================
ratios = [0.10, 0.25, 1.00, 4.00, 16.00];
nR = numel(ratios);

ekfPosRmseMean  = zeros(nR, 1);
ekfPosRmseStd   = zeros(nR, 1);
ekfSig2Mean     = zeros(nR, 1);
ekfNisMean      = zeros(nR, 1);

fprintf('\n=== Study B: EKF Covariance Ratio Vhat / V_true on S1_sensor_noise ===\n');
fprintf('%-10s | %15s | %11s | %10s\n', ...
    'Vhat/V', 'Est_RMSE [m]', 'Within2Sig%', 'Mean_NIS');
fprintf('%s\n', repmat('-', 1, 56));

for i = 1:nR
    alpha = ratios(i);
    posBuf  = zeros(numSeeds, 1);
    sig2Buf = zeros(numSeeds, 1);
    nisBuf  = zeros(numSeeds, 1);

    for r = 1:numSeeds
        cfg1 = scenario_presets('S1_sensor_noise');
        cfg1.seed = seeds(r);
        Vtrue = diag([cfg1.scenario.sigmaOdoD^2, cfg1.scenario.sigmaOdoTheta^2]);
        cfg1.ekf.Vhat = alpha * Vtrue;
        cfg1.ekf.useGating = false; % observe raw consistency & NIS

        res = simulate_closed_loop('C2_ekf', cfg1);
        posBuf(r)  = res.estMetrics.posRmse;
        sig2Buf(r) = mean(res.estMetrics.within2SigmaPos(1:2)) * 100.0;
        nisBuf(r)  = res.estMetrics.meanNis;
    end

    ekfPosRmseMean(i) = mean(posBuf);
    ekfPosRmseStd(i)  = std(posBuf);
    ekfSig2Mean(i)    = mean(sig2Buf);
    ekfNisMean(i)     = mean(nisBuf);

    fprintf('%-10.2f | %6.4f+-%6.4f | %10.1f%% | %10.3f\n', ...
        alpha, ekfPosRmseMean(i), ekfPosRmseStd(i), ekfSig2Mean(i), ekfNisMean(i));
end

tblEkfRatio = table(ratios(:), ekfPosRmseMean, ekfPosRmseStd, ekfSig2Mean, ekfNisMean, ...
    'VariableNames', {'Vhat_Over_Vtrue', 'EstRmseMean_m', 'EstRmseStd_m', ...
    'Within2SigmaPct', 'MeanNIS'});
writetable(tblEkfRatio, fullfile(resultsDir, 'param_ekf_ratio_table.csv'));

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
semilogx(ratios, ekfSig2Mean, '-o', 'LineWidth', 1.8, 'DisplayName', 'Within \pm2\sigma [%]');
hold on; grid on;
yline(95.0, 'b--', '95% Target', 'HandleVisibility', 'off');
ylabel('Samples Within \pm2\sigma [%]');
ylim([60 102]);
yyaxis right;
errorbar(ratios, ekfPosRmseMean, ekfPosRmseStd, '-s', 'LineWidth', 1.6, ...
    'DisplayName', 'Est Pos RMSE [m]');
set(gca, 'XScale', 'log');
ylabel('Localization Position RMSE [m]');
xlabel('Covariances Ratio \alpha = V_{hat} / V_{true} (log scale)');
title('Study B: Overconfident (\alpha<1) vs Pessimistic (\alpha>1) EKF');
legend('Location', 'southeast', 'FontSize', 8);

exportgraphics(fig, fullfile(resultsDir, 'param_sweeps_analysis.png'), 'Resolution', 200);
save(fullfile(resultsDir, 'param_sweeps_results.mat'), 'tblLookahead', 'tblEkfRatio');
fprintf('\nSaved parameter sweep outputs to: %s\n', resultsDir);
