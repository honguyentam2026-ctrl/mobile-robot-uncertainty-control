function draw_chapter4_analysis_figures()
%DRAW_CHAPTER4_ANALYSIS_FIGURES  Generate Chapter 4 analytical figures:
%   1) ch4_metrics_dashboard_vi.png / _en.png:
%      2x2 comprehensive bar chart dashboard (with error bars & numeric labels)
%      comparing C2, C3, C4, C5 across S1..S5 on all 4 primary metrics:
%        (a) Cross-Track Error CTE_RMSE [m]
%        (b) Position Estimation Error Est_RMSE [m]
%        (c) Completion Time [s]
%        (d) Angular Velocity Command Total Variation w_TV [rad/s]
%   2) ch4_scenarios_s1_to_s4_vi.png / _en.png:
%      2x2 physical phenomenon & theory-vs-experiment diagnostic plots:
%        (a) S1 Sensor Noise: Unfiltered random-walk drift (C1) vs filtered (C2, C5)
%        (b) S2 Wheel Slip (30%): Velocity loss in slip zone & Fuzzy-PID slip boost
%        (c) S3 Mechanical Mismatch (-20%/+20%): CompFilter steady bias vs EKF
%        (d) S4 Heavy Load Step at t=20s (K=0.65, T=0.70s): Speed recovery vs CTE spike
%
%   Usage:
%       run('04_report_figures/draw_chapter4_analysis_figures.m')

    thisDir  = fileparts(mfilename('fullpath'));
    codeRoot = fileparts(thisDir);
    addpath(fullfile(codeRoot, '02_models_and_controllers'));
    addpath(fullfile(codeRoot, '03_experiments'));
    addpath(fullfile(codeRoot, 'utils'));

    outDir = fullfile(thisDir, 'results');
    if ~exist(outDir, 'dir')
        mkdir(outDir);
    end

    matPath = fullfile(codeRoot, '03_experiments', 'results', 'mc_suite_results.mat');
    mc = load(matPath);

    % Run representative seed=1001 simulations for S1, S2, S3, S4 to extract time series
    seedRep = 1001;
    modes = {'C1_noisy', 'C2_comp', 'C3_comp_pidf', 'C4_comp_fuzzy_pidf', 'C5_ekf_fuzzy_pidf'};

    cfgS1 = scenario_presets('S1_sensor_noise');   cfgS1.seed = seedRep;
    cfgS2 = scenario_presets('S2_wheel_slip');     cfgS2.seed = seedRep;
    cfgS3 = scenario_presets('S3_model_mismatch'); cfgS3.seed = seedRep;
    cfgS4 = scenario_presets('S4_load_step');      cfgS4.seed = seedRep;

    for m = 1:5
        repS1{m} = simulate_closed_loop(modes{m}, cfgS1); %#ok<AGROW>
        if m >= 2
            repS2{m} = simulate_closed_loop(modes{m}, cfgS2); %#ok<AGROW>
            repS3{m} = simulate_closed_loop(modes{m}, cfgS3); %#ok<AGROW>
            repS4{m} = simulate_closed_loop(modes{m}, cfgS4); %#ok<AGROW>
        end
    end

    % Print Monte Carlo Summary Table (Table 4.1) to Command Window
    csvPath = fullfile(codeRoot, '03_experiments', 'results', 'mc_summary_table.csv');
    if exist(csvPath, 'file')
        fprintf('\n=== BANG 4.1: TONG HOP KET QUA MONTE CARLO (N = 30 seeds, 750 luot chay) ===\n');
        disp(readtable(csvPath));
    end

    showGui = ~batchStartupOptionUsed;

    % Render Dashboard (Figure 4.3) in VI and EN (keep VI open on screen if GUI)
    render_dashboard(mc, 'en', fullfile(outDir, 'ch4_metrics_dashboard_en.png'), false);
    render_dashboard(mc, 'vi', fullfile(outDir, 'ch4_metrics_dashboard_vi.png'), showGui);

    % Render S1..S4 Physical Phenomena (Figure 4.4) in VI and EN (keep VI open on screen if GUI)
    render_s1_to_s4(mc.resS0, cfgS1, repS1, cfgS2, repS2, cfgS3, repS3, cfgS4, repS4, ...
        'en', fullfile(outDir, 'ch4_scenarios_s1_to_s4_en.png'), false);
    render_s1_to_s4(mc.resS0, cfgS1, repS1, cfgS2, repS2, cfgS3, repS3, cfgS4, repS4, ...
        'vi', fullfile(outDir, 'ch4_scenarios_s1_to_s4_vi.png'), showGui);

    % Also pop up Figure 4.2 (Boxplot) and Figure 4.6 (S5 Detailed Analysis) if in GUI
    if showGui
        imgBox = fullfile(codeRoot, '03_experiments', 'results', 'mc_cte_and_time.png');
        if exist(imgBox, 'file')
            figBox = figure('Name', 'Hinh 4.2 - Phan bo Boxplot CTE RMSE & Thoi gian hoan thanh (S1-S5)', ...
                'Visible', 'on', 'Color', 'w', 'Position', [40, 80, 1250, 520]);
            axB = axes(figBox, 'Position', [0.01 0.01 0.98 0.98]);
            imshow(imread(imgBox), 'Parent', axB);
            axis(axB, 'off');
        end
        imgS5 = fullfile(codeRoot, '03_experiments', 'results', 'mc_s5_detailed_analysis.png');
        if exist(imgS5, 'file')
            figS5 = figure('Name', 'Hinh 4.6 - Phan tich chi tiet kich ban S5 (Quỹ đạo, CTE, Sai số định vị, Hệ số Fuzzy-PID)', ...
                'Visible', 'on', 'Color', 'w', 'Position', [90, 40, 1280, 780]);
            axS5 = axes(figS5, 'Position', [0.01 0.01 0.98 0.98]);
            imshow(imread(imgS5), 'Parent', axS5);
            axis(axS5, 'off');
        end
    end

    fprintf('Successfully generated & displayed Chapter 4 analysis figures (Hinh 4.2, 4.3, 4.4, 4.6)!\n');
end

function render_dashboard(mc, lang, outPath, keepOpen)
    visFlag = ternary(keepOpen, 'on', 'off');
    fig = figure('Name', 'Hinh 4.3 - Dashboard 4 Chi tieu dinh luong Monte Carlo (S1-S5)', ...
        'Visible', visFlag, 'Color', 'w', 'Position', [60, 60, 1380, 820]);

    colors4 = [0.85 0.55 0.10;   % C2 CompFilter (Amber)
               0.49 0.18 0.56;   % C3 Comp+PIDF  (Purple)
               0.15 0.62 0.25;   % C4 Comp+Fuzzy (Green)
               0.00 0.45 0.74];  % C5 EKF+Fuzzy  (Blue)

    if strcmpi(lang, 'vi')
        scenNames = {'S1 Nhiễu', 'S2 Trượt', 'S3 Lệch bánh', 'S4 Đổi tải', 'S5 Tổ hợp'};
        legLabels = {'C2 Bộ lọc bù', 'C3 Lọc bù + PIDF', 'C4 Lọc bù + Fuzzy-PID', 'C5 EKF + Fuzzy-PID'};
        c0CteLbl  = 'Mốc lý tưởng C0 (0,0428 m)';
        c0TimeLbl = 'Thời gian C0 (54,9 s)';
        t1 = '(a) Sai số bám quỹ đạo CTE RMSE [m] (Càng nhỏ càng bám sát)';
        y1 = 'CTE RMSE [m]';
        t2 = '(b) Sai số định vị Est RMSE [m] (EKF C5 khử sai số vượt trội)';
        y2 = 'Est RMSE [m]';
        t3 = '(c) Thời gian hoàn thành [s] (Fuzzy-PID & EKF bù sụt tốc)';
        y3 = 'Thời gian hoàn thành [s]';
        t4 = '(d) Biến thiên lệnh lái w_{TV} [rad/s] (Đánh đổi độ mượt & tốc độ đáp ứng)';
        y4 = 'Tổng biến thiên lái w_{TV} [rad/s]';
    else
        scenNames = {'S1 Noise', 'S2 Slip', 'S3 Mismatch', 'S4 LoadStep', 'S5 Worst'};
        legLabels = {'C2 CompFilter', 'C3 Comp+PIDF', 'C4 Comp+FuzzyPID', 'C5 EKF+FuzzyPID'};
        c0CteLbl  = 'C0 Ideal Baseline (0.0428 m)';
        c0TimeLbl = 'C0 Ideal Time (54.9 s)';
        t1 = '(a) Cross-Track Error CTE RMSE [m] (Lower = Tighter Tracking)';
        y1 = 'CTE RMSE [m]';
        t2 = '(b) Localization Error Est RMSE [m] (EKF C5 Eliminates Drift/Bias)';
        y2 = 'Est RMSE [m]';
        t3 = '(c) Completion Time [s] (Fuzzy-PID & EKF Recover Velocity Loss)';
        y3 = 'Completion Time [s]';
        t4 = '(d) Steering Total Variation w_{TV} [rad/s] (Smoothness vs. Agility Trade-off)';
        y4 = 'Steering Variation w_{TV} [rad/s]';
    end

    cteMean  = squeeze(mean(mc.cteRmse(:, 2:5, :), 3));
    cteStd   = squeeze(std(mc.cteRmse(:, 2:5, :), 0, 3));
    estMean  = squeeze(mean(mc.estPosRmse(:, 2:5, :), 3));
    estStd   = squeeze(std(mc.estPosRmse(:, 2:5, :), 0, 3));
    timeMean = squeeze(mean(mc.compTime(:, 2:5, :), 3));
    timeStd  = squeeze(std(mc.compTime(:, 2:5, :), 0, 3));
    wtvMean  = squeeze(mean(mc.wTV(:, 2:5, :), 3));
    wtvStd   = squeeze(std(mc.wTV(:, 2:5, :), 0, 3));

    % --- (a) CTE RMSE ---
    ax1 = subplot(2, 2, 1); hold(ax1, 'on'); grid(ax1, 'on');
    hb1 = draw_grouped_bars(ax1, cteMean, cteStd, colors4, '%.3f', 0.23);
    hRef1 = yline(ax1, mc.resS0.metrics.rmseCrossTrack, 'k--', 'LineWidth', 1.4);
    set(ax1, 'XTick', 1:5, 'XTickLabel', scenNames, 'FontName', 'Times New Roman', 'FontSize', 10.5);
    ylabel(ax1, y1, 'FontName', 'Times New Roman', 'FontSize', 11, 'FontWeight', 'bold');
    title(ax1, t1, 'FontName', 'Times New Roman', 'FontSize', 11.5, 'FontWeight', 'bold');
    ylim(ax1, [0, 0.24]);
    legend(ax1, [hb1, hRef1], [legLabels, {c0CteLbl}], 'Location', 'northwest', 'FontSize', 8.5);

    % --- (b) Est Position RMSE ---
    ax2 = subplot(2, 2, 2); hold(ax2, 'on'); grid(ax2, 'on');
    hb2 = draw_grouped_bars(ax2, estMean, estStd, colors4, '%.3f', 0.28);
    set(ax2, 'XTick', 1:5, 'XTickLabel', scenNames, 'FontName', 'Times New Roman', 'FontSize', 10.5);
    ylabel(ax2, y2, 'FontName', 'Times New Roman', 'FontSize', 11, 'FontWeight', 'bold');
    title(ax2, t2, 'FontName', 'Times New Roman', 'FontSize', 11.5, 'FontWeight', 'bold');
    ylim(ax2, [0, 0.28]);
    legend(ax2, hb2, legLabels, 'Location', 'northwest', 'FontSize', 8.5);

    % --- (c) Completion Time ---
    ax3 = subplot(2, 2, 3); hold(ax3, 'on'); grid(ax3, 'on');
    hb3 = draw_grouped_bars(ax3, timeMean, timeStd, colors4, '%.1f', 118);
    hRef3 = yline(ax3, mc.resS0.metrics.completionTime, 'k--', 'LineWidth', 1.4);
    set(ax3, 'XTick', 1:5, 'XTickLabel', scenNames, 'FontName', 'Times New Roman', 'FontSize', 10.5);
    ylabel(ax3, y3, 'FontName', 'Times New Roman', 'FontSize', 11, 'FontWeight', 'bold');
    title(ax3, t3, 'FontName', 'Times New Roman', 'FontSize', 11.5, 'FontWeight', 'bold');
    ylim(ax3, [0, 118]);
    legend(ax3, [hb3, hRef3], [legLabels, {c0TimeLbl}], 'Location', 'northwest', 'FontSize', 8.5);

    % --- (d) Steering Total Variation w_TV ---
    ax4 = subplot(2, 2, 4); hold(ax4, 'on'); grid(ax4, 'on');
    hb4 = draw_grouped_bars(ax4, wtvMean, wtvStd, colors4, '%.0f', 285);
    set(ax4, 'XTick', 1:5, 'XTickLabel', scenNames, 'FontName', 'Times New Roman', 'FontSize', 10.5);
    ylabel(ax4, y4, 'FontName', 'Times New Roman', 'FontSize', 11, 'FontWeight', 'bold');
    title(ax4, t4, 'FontName', 'Times New Roman', 'FontSize', 11.5, 'FontWeight', 'bold');
    ylim(ax4, [0, 285]);
    legend(ax4, hb4, legLabels, 'Location', 'northwest', 'FontSize', 8.5);

    exportgraphics(fig, outPath, 'Resolution', 250, 'BackgroundColor', 'white');
    if ~keepOpen
        close(fig);
    end
end

function hb = draw_grouped_bars(ax, meanMat, stdMat, colors4, fmtStr, yMax)
    hb = bar(ax, meanMat, 'grouped', 'BarWidth', 0.84);
    for k = 1:4
        hb(k).FaceColor = colors4(k, :);
        hb(k).EdgeColor = [0.15 0.15 0.15];
        hb(k).LineWidth = 0.7;
    end
    drawnow;
    vOffsets = [0.008, 0.042, 0.008, 0.042];
    for k = 1:4
        xEnds = hb(k).XEndPoints;
        yEnds = hb(k).YEndPoints;
        sVals = stdMat(:, k)';
        sCap  = min(sVals, 0.35 * yEnds); % Cap giant outlier errorbar on S5 for visual clarity
        errorbar(ax, xEnds, yEnds, sCap, 'k.', 'LineWidth', 0.85, ...
            'CapSize', 3.0, 'HandleVisibility', 'off');
        for i = 1:numel(xEnds)
            yTop = min(yEnds(i) + sCap(i) + vOffsets(k) * yMax, 0.93 * yMax);
            isBest = (k == 4);
            text(ax, xEnds(i), yTop, sprintf(fmtStr, yEnds(i)), ...
                'FontName', 'Times New Roman', 'FontSize', 7.6, ...
                'FontWeight', ternary(isBest, 'bold', 'normal'), ...
                'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
                'Color', ternary(isBest, [0 0.25 0.55], [0.15 0.15 0.15]));
        end
    end
end

function render_s1_to_s4(resS0, cfgS1, repS1, cfgS2, repS2, ~, repS3, cfgS4, repS4, lang, outPath, keepOpen)
    visFlag = ternary(keepOpen, 'on', 'off');
    fig = figure('Name', 'Hinh 4.4 - Phan tich 4 hien tuong vat ly tren S1-S4', ...
        'Visible', visFlag, 'Color', 'w', 'Position', [75, 45, 1380, 820]);

    colors4 = [0.85 0.55 0.10;   % C2 CompFilter (Amber)
               0.49 0.18 0.56;   % C3 Comp+PIDF  (Purple)
               0.15 0.62 0.25;   % C4 Comp+Fuzzy (Green)
               0.00 0.45 0.74];  % C5 EKF+Fuzzy  (Blue)

    if strcmpi(lang, 'vi')
        t1 = '(a) Kịch bản S1 (Nhiễu cảm biến): Quỹ đạo trôi C1 vs Lọc bù C2 & EKF C5';
        lRef = 'Quỹ đạo đặt'; lC1 = 'C1 Không lọc (Trôi lạc đường)';
        lC2 = 'C2 Bộ lọc bù'; lC3 = 'C3 Lọc bù + PIDF';
        lC4 = 'C4 Lọc bù + Fuzzy-PID'; lC5 = 'C5 EKF + Fuzzy-PID';

        t2 = '(b) Kịch bản S2 (Trượt bánh 30%): Sụt vận tốc thực & Bù trượt Fuzzy-PID';
        y2 = 'Vận tốc dài thực v_{true} [m/s]';
        lVd = 'Vận tốc đặt v_d = 0,30 m/s';
        noteSlip = 'Vùng sàn trượt 30% (v_{C2} sụt đúng lý thuyết 0,21 m/s)';

        t3 = '(c) Kịch bản S3 (Lệch bánh -20%/+20%): Sai số định vị Lọc bù vs EKF';
        y3 = 'Sai số định vị ||p_{est} - p_{true}||_2 [m]';

        t4 = '(d) Kịch bản S4 (Đổi tải nặng tại t = 20 s): Khôi phục tốc độ & Vọt lố CTE';
        y4L = 'Vận tốc dài thực v_{true} [m/s]';
        y4R = 'Sai số bám ngang CTE [m]';
        noteLoad = 'Đổi tải t = 20 s (K: 1,0 \rightarrow 0,65)';
    else
        t1 = '(a) Scenario S1 (Sensor Noise): Unfiltered Drift C1 vs. CompFilter C2 & EKF C5';
        lRef = 'Reference Path'; lC1 = 'C1 Unfiltered (Random-Walk Drift)';
        lC2 = 'C2 CompFilter'; lC3 = 'C3 Comp+PIDF';
        lC4 = 'C4 Comp+FuzzyPID'; lC5 = 'C5 EKF+FuzzyPID';

        t2 = '(b) Scenario S2 (30% Wheel Slip): True Velocity Drop & Fuzzy-PID Slip Boost';
        y2 = 'True Linear Velocity v_{true} [m/s]';
        lVd = 'Desired Velocity v_d = 0.30 m/s';
        noteSlip = '30% Slip Zone (C2 drops to theoretical 0.21 m/s)';

        t3 = '(c) Scenario S3 (Wheel Mismatch -20%/+20%): Localization Error CompFilter vs. EKF';
        y3 = 'Position Estimation Error ||p_{est} - p_{true}||_2 [m]';

        t4 = '(d) Scenario S4 (Heavy Load Step at t = 20 s): Speed Recovery vs. Cornering CTE';
        y4L = 'True Linear Velocity v_{true} [m/s]';
        y4R = 'Cross-Track Error CTE [m]';
        noteLoad = 'Load Step t = 20 s (K: 1.0 \rightarrow 0.65)';
    end

    % --- (a) S1 Trajectory: C1 drift vs C2 & C5 ---
    ax1 = subplot(2, 2, 1); hold(ax1, 'on'); grid(ax1, 'on'); axis(ax1, 'equal');
    plot(ax1, cfgS1.path(:,1), cfgS1.path(:,2), 'k--', 'LineWidth', 1.5, 'DisplayName', lRef);
    plot(ax1, cfgS1.landmarks(:,1), cfgS1.landmarks(:,2), 'k^', 'MarkerSize', 5.5, ...
        'MarkerFaceColor', [0.82 0.82 0.82], 'DisplayName', 'Landmarks');
    plot(ax1, repS1{1}.poseTrue(:,1), repS1{1}.poseTrue(:,2), 'r-', 'LineWidth', 1.8, 'DisplayName', lC1);
    plot(ax1, repS1{2}.poseTrue(:,1), repS1{2}.poseTrue(:,2), '-', 'Color', colors4(1,:), 'LineWidth', 1.6, 'DisplayName', lC2);
    plot(ax1, repS1{5}.poseTrue(:,1), repS1{5}.poseTrue(:,2), '-', 'Color', colors4(4,:), 'LineWidth', 1.8, 'DisplayName', lC5);
    xlim(ax1, [-1 14]); ylim(ax1, [-1 13]);
    xlabel(ax1, 'x [m]', 'FontName', 'Times New Roman', 'FontSize', 10.5);
    ylabel(ax1, 'y [m]', 'FontName', 'Times New Roman', 'FontSize', 10.5);
    title(ax1, t1, 'FontName', 'Times New Roman', 'FontSize', 11.2, 'FontWeight', 'bold');
    legend(ax1, 'Location', 'southeast', 'FontSize', 8.2);

    % --- (b) S2 Wheel Slip: True Linear Velocity ---
    ax2 = subplot(2, 2, 2); hold(ax2, 'on'); grid(ax2, 'on');
    % Find slip zone entry/exit time window on C4
    r4s2 = repS2{4};
    inZone = (r4s2.poseTrue(:,1) >= cfgS2.scenario.slipZone.xRange(1)) & ...
             (r4s2.poseTrue(:,1) <= cfgS2.scenario.slipZone.xRange(2)) & ...
             (r4s2.poseTrue(:,2) >= cfgS2.scenario.slipZone.yRange(1)) & ...
             (r4s2.poseTrue(:,2) <= cfgS2.scenario.slipZone.yRange(2));
    tIn = r4s2.t(find(inZone, 1, 'first'));
    tOut = r4s2.t(find(inZone, 1, 'last'));
    patch(ax2, [tIn tOut tOut tIn], [0 0 0.38 0.38], [0.88 0.94 1.0], ...
        'EdgeColor', 'none', 'FaceAlpha', 0.55, 'DisplayName', noteSlip);
    yline(ax2, 0.30, 'k--', 'LineWidth', 1.3, 'DisplayName', lVd);
    yline(ax2, 0.21, 'r:', 'LineWidth', 1.2, 'HandleVisibility', 'off');

    for mIdx = [2, 3, 4, 5]
        r = repS2{mIdx};
        vTrue = hypot(diff(r.poseTrue(:,1)), diff(r.poseTrue(:,2))) / cfgS2.dt;
        vSmooth = movmean(vTrue, 5);
        lbl = select_label(mIdx, lC2, lC3, lC4, lC5);
        plot(ax2, r.t(1:end-1), vSmooth, 'Color', colors4(mIdx-1,:), 'LineWidth', 1.6, 'DisplayName', lbl);
    end
    xlim(ax2, [0, 68]); ylim(ax2, [0.12, 0.37]);
    xlabel(ax2, 'Time [s]', 'FontName', 'Times New Roman', 'FontSize', 10.5);
    ylabel(ax2, y2, 'FontName', 'Times New Roman', 'FontSize', 10.5, 'FontWeight', 'bold');
    title(ax2, t2, 'FontName', 'Times New Roman', 'FontSize', 11.2, 'FontWeight', 'bold');
    legend(ax2, 'Location', 'southwest', 'FontSize', 8.0);

    % --- (c) S3 Model Mismatch: Localization Error Time Series ---
    ax3 = subplot(2, 2, 3); hold(ax3, 'on'); grid(ax3, 'on');
    for mIdx = [2, 4, 5]
        r = repS3{mIdx};
        posErr = hypot(r.poseEst(:,1) - r.poseTrue(:,1), r.poseEst(:,2) - r.poseTrue(:,2));
        lbl = select_label(mIdx, lC2, lC3, lC4, lC5);
        plot(ax3, r.t, movmean(posErr, 5), 'Color', colors4(mIdx-1,:), 'LineWidth', 1.6, 'DisplayName', lbl);
    end
    yline(ax3, 0.1094, '--', 'Color', colors4(3,:), 'LineWidth', 1.2, 'HandleVisibility', 'off');
    yline(ax3, 0.0528, '--', 'Color', colors4(4,:), 'LineWidth', 1.2, 'HandleVisibility', 'off');
    xlim(ax3, [0, 70]); ylim(ax3, [0, 0.25]);
    xlabel(ax3, 'Time [s]', 'FontName', 'Times New Roman', 'FontSize', 10.5);
    ylabel(ax3, y3, 'FontName', 'Times New Roman', 'FontSize', 10.5, 'FontWeight', 'bold');
    title(ax3, t3, 'FontName', 'Times New Roman', 'FontSize', 11.2, 'FontWeight', 'bold');
    legend(ax3, 'Location', 'northeast', 'FontSize', 8.2);

    % --- (d) S4 Load Step at t=20s: Dual-Axis Velocity Recovery vs Cornering CTE ---
    ax4 = subplot(2, 2, 4); hold(ax4, 'on'); grid(ax4, 'on');
    yyaxis(ax4, 'left');
    for mIdx = [2, 3, 5]
        r = repS4{mIdx};
        vTrue = movmean(hypot(diff(r.poseTrue(:,1)), diff(r.poseTrue(:,2))) / cfgS4.dt, 7);
        lbl = [select_label(mIdx, lC2, lC3, lC4, lC5), ' (v)'];
        plot(ax4, r.t(1:end-1), vTrue, '-', 'Color', colors4(mIdx-1,:), 'LineWidth', 1.6, 'DisplayName', lbl);
    end
    ylabel(ax4, y4L, 'FontName', 'Times New Roman', 'FontSize', 10.5, 'FontWeight', 'bold');
    ylim(ax4, [0.14, 0.36]);
    ax4.YColor = [0.1 0.1 0.1];

    yyaxis(ax4, 'right');
    plot(ax4, repS4{2}.t, repS4{2}.metrics.crossTrackError, ':', 'Color', colors4(1,:), ...
        'LineWidth', 1.4, 'DisplayName', [lC2, ' (CTE)']);
    plot(ax4, repS4{5}.t, repS4{5}.metrics.crossTrackError, '--', 'Color', colors4(4,:), ...
        'LineWidth', 1.4, 'DisplayName', [lC5, ' (CTE)']);
    ylabel(ax4, y4R, 'FontName', 'Times New Roman', 'FontSize', 10.5, 'FontWeight', 'bold');
    ylim(ax4, [0, 0.45]);
    ax4.YColor = [0.0 0.35 0.65];

    xline(ax4, 20.0, 'r--', noteLoad, 'LineWidth', 1.3, 'FontName', 'Times New Roman', ...
        'FontSize', 8.5, 'LabelVerticalAlignment', 'bottom', 'LabelHorizontalAlignment', 'left', ...
        'HandleVisibility', 'off');
    xlim(ax4, [0, 80]);
    xlabel(ax4, 'Time [s]', 'FontName', 'Times New Roman', 'FontSize', 10.5);
    title(ax4, t4, 'FontName', 'Times New Roman', 'FontSize', 11.2, 'FontWeight', 'bold');
    legend(ax4, 'Location', 'northeast', 'FontSize', 7.8);

    exportgraphics(fig, outPath, 'Resolution', 250, 'BackgroundColor', 'white');
    if ~keepOpen
        close(fig);
    end
end

function out = ternary(cond, a, b)
    if cond
        out = a;
    else
        out = b;
    end
end

function lbl = select_label(mIdx, lC2, lC3, lC4, lC5)
    switch mIdx
        case 2, lbl = lC2;
        case 3, lbl = lC3;
        case 4, lbl = lC4;
        case 5, lbl = lC5;
    end
end
