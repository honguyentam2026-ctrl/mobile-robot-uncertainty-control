function draw_figure_2_5_ekf()
%DRAW_FIGURE_2_5_EKF  Draw Figure 2.5 (2D Extended Kalman Filter Block Diagram)
%   Generates Black-and-White (Monochrome) versions in Vietnamese and English
%   with large, crisp typography at 300 DPI:
%       - hinh_2_5_ekf_bw_vi.png  (Black & White - Vietnamese)
%       - hinh_2_5_ekf_bw_en.png  (Black & White - English)
%
%   Usage:
%       run('04_report_figures/draw_figure_2_5_ekf.m')

    thisDir = fileparts(mfilename('fullpath'));
    outDir  = fullfile(thisDir, 'results');
    if ~exist(outDir, 'dir')
        mkdir(outDir);
    end

    outBwVi = fullfile(outDir, 'hinh_2_5_ekf_bw_vi.png');
    outBwEn = fullfile(outDir, 'hinh_2_5_ekf_bw_en.png');

    render_ekf_bw('vi', outBwVi);
    render_ekf_bw('en', outBwEn);

    fprintf('Saved EKF B&W VI : %s\n', outBwVi);
    fprintf('Saved EKF B&W EN : %s\n', outBwEn);
end

function L = get_labels(lang)
    if strcmpi(lang, 'vi')
        L.tOdoTitle  = 'Odometry (10 Hz)';
        L.tPredTitle = '1. Dự đoán EKF (10 Hz)';
        L.tLmTitle   = 'Landmark (2 Hz)';
        L.tMeasTitle = '2. Mô hình đo & Cổng Mahalanobis';
        L.tUpdTitle  = '3. Độ lợi Kalman & Cập nhật Joseph';
        L.tOutTitle  = 'Đầu ra EKF';
        L.tSlipLabel = 'Độ trượt dọc:';
        L.tDelay     = 'Trễ 1 bước';
    else
        L.tOdoTitle  = 'Odometry (10 Hz)';
        L.tPredTitle = '1. EKF Prediction (10 Hz)';
        L.tLmTitle   = 'Landmark (2 Hz)';
        L.tMeasTitle = '2. Measurement Model & Gate';
        L.tUpdTitle  = '3. Kalman Gain & Joseph Update';
        L.tOutTitle  = 'EKF Output';
        L.tSlipLabel = 'Slip ratio:';
        L.tDelay     = 'Unit Delay';
    end
end

function render_ekf_bw(lang, outPath)
    % Compact canvas (1120 x 540 px) with large 14-16 pt fonts matching Figure 2.4
    fig = figure('Visible', 'off', 'Color', 'w', ...
        'Units', 'pixels', 'Position', [80, 80, 1120, 540]);

    ax = axes('Parent', fig, 'Position', [0.005, 0.005, 0.99, 0.99]);
    hold(ax, 'on');
    axis(ax, 'equal');
    axis(ax, 'off');
    xlim(ax, [0.05, 14.95]);
    ylim(ax, [0.05, 6.15]);

    L = get_labels(lang);
    cBlack = [0 0 0];
    cWhite = [1 1 1];

    % ---------------------------------------------------------------------
    % TOP BRANCH: Odometry (10 Hz) -> 1. EKF Prediction (10 Hz)
    % ---------------------------------------------------------------------
    draw_round_box(ax, 0.15, 4.00, 2.35, 1.45, cWhite, cBlack, 2.0);
    text(ax, 1.325, 5.00, L.tOdoTitle, 'FontName', 'Times New Roman', ...
        'FontSize', 14.5, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', cBlack);
    text(ax, 1.325, 4.38, '$(\delta_d,\;\delta_\theta)$', 'Interpreter', 'latex', ...
        'FontSize', 16.5, 'HorizontalAlignment', 'center', 'Color', cBlack);

    draw_tech_arrow(ax, 2.50, 4.92, 3.15, 4.92, cBlack, 2.0);

    draw_round_box(ax, 3.15, 3.78, 4.15, 1.90, cWhite, cBlack, 2.0);
    text(ax, 5.225, 5.34, L.tPredTitle, 'FontName', 'Times New Roman', ...
        'FontSize', 14.5, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', cBlack);
    text(ax, 5.225, 4.74, '$\hat{\mathbf{x}}_k^- = f(\hat{\mathbf{x}}_{k-1},\;\delta_d,\;\delta_\theta)$', ...
        'Interpreter', 'latex', 'FontSize', 15.0, 'HorizontalAlignment', 'center', 'Color', cBlack);
    text(ax, 5.225, 4.16, '$\mathbf{P}_k^- = \mathbf{F}_x \mathbf{P}_{k-1}\mathbf{F}_x^T + \mathbf{F}_v \hat{\mathbf{V}}\mathbf{F}_v^T$', ...
        'Interpreter', 'latex', 'FontSize', 14.2, 'HorizontalAlignment', 'center', 'Color', cBlack);

    % ---------------------------------------------------------------------
    % BOTTOM BRANCH: Landmark (2 Hz) -> 2. Measurement Model & Gate
    % ---------------------------------------------------------------------
    draw_round_box(ax, 0.15, 1.72, 2.35, 1.45, cWhite, cBlack, 2.0);
    text(ax, 1.325, 2.72, L.tLmTitle, 'FontName', 'Times New Roman', ...
        'FontSize', 14.5, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', cBlack);
    text(ax, 1.325, 2.08, '$\mathbf{z}_i = [r_i,\;\beta_i]^T$', 'Interpreter', 'latex', ...
        'FontSize', 15.5, 'HorizontalAlignment', 'center', 'Color', cBlack);

    draw_tech_arrow(ax, 2.50, 2.44, 3.15, 2.44, cBlack, 2.0);

    draw_round_box(ax, 3.15, 1.48, 4.15, 1.92, cWhite, cBlack, 2.0);
    text(ax, 5.225, 3.06, L.tMeasTitle, 'FontName', 'Times New Roman', ...
        'FontSize', 14.2, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', cBlack);
    text(ax, 5.225, 2.48, '$\nu_i = \mathbf{z}_i - h_i(\hat{\mathbf{x}}_k^-),\;\; \mathbf{H}_i = \partial h_i / \partial \mathbf{x}$', ...
        'Interpreter', 'latex', 'FontSize', 14.2, 'HorizontalAlignment', 'center', 'Color', cBlack);
    text(ax, 5.225, 1.86, '$d_{\mathrm{NIS},i} = \nu_i^T \mathbf{S}_i^{-1}\nu_i \le \gamma_{\chi^2}$', ...
        'Interpreter', 'latex', 'FontSize', 14.5, 'HorizontalAlignment', 'center', 'Color', cBlack);

    % ---------------------------------------------------------------------
    % BLOCK 3: Kalman Gain & Joseph Update
    % ---------------------------------------------------------------------
    % Orthogonal routing from Block 1 and Block 2 into Block 3
    plot(ax, [7.30, 7.62], [4.73, 4.73], '-', 'Color', cBlack, 'LineWidth', 2.0);
    plot(ax, [7.62, 7.62], [4.73, 4.15], '-', 'Color', cBlack, 'LineWidth', 2.0);
    draw_tech_arrow(ax, 7.62, 4.15, 7.95, 4.15, cBlack, 2.0);

    plot(ax, [7.30, 7.62], [2.44, 2.44], '-', 'Color', cBlack, 'LineWidth', 2.0);
    plot(ax, [7.62, 7.62], [2.44, 3.02], '-', 'Color', cBlack, 'LineWidth', 2.0);
    draw_tech_arrow(ax, 7.62, 3.02, 7.95, 3.02, cBlack, 2.0);

    draw_round_box(ax, 7.95, 2.25, 4.55, 2.65, cWhite, cBlack, 2.0);
    text(ax, 10.225, 4.56, L.tUpdTitle, 'FontName', 'Times New Roman', ...
        'FontSize', 14.0, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', cBlack);
    text(ax, 10.225, 4.00, '$\mathbf{K}_i = \mathbf{P}\mathbf{H}_i^T(\mathbf{H}_i\mathbf{P}\mathbf{H}_i^T + \hat{\mathbf{W}})^{-1}$', ...
        'Interpreter', 'latex', 'FontSize', 14.2, 'HorizontalAlignment', 'center', 'Color', cBlack);
    text(ax, 10.225, 3.40, '$\hat{\mathbf{x}}_k \leftarrow \hat{\mathbf{x}}_k^- + \mathbf{K}_i\nu_i$', ...
        'Interpreter', 'latex', 'FontSize', 14.8, 'HorizontalAlignment', 'center', 'Color', cBlack);
    text(ax, 10.225, 2.74, '$\mathbf{P}_k \leftarrow (\mathbf{I}-\mathbf{K}_i\mathbf{H}_i)\mathbf{P}(\mathbf{I}-\mathbf{K}_i\mathbf{H}_i)^T + \mathbf{K}_i\hat{\mathbf{W}}\mathbf{K}_i^T$', ...
        'Interpreter', 'latex', 'FontSize', 12.6, 'HorizontalAlignment', 'center', 'Color', cBlack);

    % ---------------------------------------------------------------------
    % OUTPUT BOX: EKF Output
    % ---------------------------------------------------------------------
    draw_tech_arrow(ax, 12.50, 3.58, 12.95, 3.58, cBlack, 2.0);

    draw_round_box(ax, 12.95, 2.58, 1.90, 2.00, cWhite, cBlack, 2.2);
    text(ax, 13.90, 4.24, L.tOutTitle, 'FontName', 'Times New Roman', ...
        'FontSize', 14.5, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', cBlack);
    text(ax, 13.90, 3.68, '$\hat{\mathbf{x}}_k,\;\mathbf{P}_k$', ...
        'Interpreter', 'latex', 'FontSize', 15.5, 'HorizontalAlignment', 'center', 'Color', cBlack);
    text(ax, 13.90, 3.20, L.tSlipLabel, 'FontName', 'Times New Roman', ...
        'FontSize', 13.0, 'FontAngle', 'italic', 'HorizontalAlignment', 'center', 'Color', cBlack);
    text(ax, 13.90, 2.84, '$\mathrm{slip}_{\mathrm{est}}(k)$', ...
        'Interpreter', 'latex', 'FontSize', 14.8, 'HorizontalAlignment', 'center', 'Color', cBlack);

    % ---------------------------------------------------------------------
    % FEEDBACK LOOP (z^-1) FROM OUTPUT TO PREDICTION (10 Hz)
    % ---------------------------------------------------------------------
    fbX = 13.90;
    fbY = 0.56;
    upX = 2.76;
    inY = 4.35;

    plot(ax, [fbX, fbX], [2.58, fbY], '-', 'Color', cBlack, 'LineWidth', 1.8);
    plot(ax, [fbX, 7.80], [fbY, fbY], '-', 'Color', cBlack, 'LineWidth', 1.8);
    draw_tech_arrow(ax, 8.20, fbY, 7.80, fbY, cBlack, 1.8);

    draw_round_box(ax, 6.00, 0.14, 1.80, 0.84, cWhite, cBlack, 1.8);
    text(ax, 6.90, 0.72, L.tDelay, 'FontName', 'Times New Roman', ...
        'FontSize', 13.2, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', cBlack);
    text(ax, 6.90, 0.36, '$z^{-1}$', 'Interpreter', 'latex', ...
        'FontSize', 15.5, 'HorizontalAlignment', 'center', 'Color', cBlack);

    plot(ax, [6.00, upX], [fbY, fbY], '-', 'Color', cBlack, 'LineWidth', 1.8);
    plot(ax, [upX, upX], [fbY, inY], '-', 'Color', cBlack, 'LineWidth', 1.8);
    draw_tech_arrow(ax, upX, inY, 3.15, inY, cBlack, 1.8);

    text(ax, 3.15, 0.86, '$\hat{\mathbf{x}}_{k-1},\;\mathbf{P}_{k-1}$', ...
        'Interpreter', 'latex', 'FontSize', 15.0, 'HorizontalAlignment', 'left', 'Color', cBlack);

    exportgraphics(fig, outPath, 'Resolution', 300, 'BackgroundColor', 'white');
    close(fig);
end

function draw_round_box(ax, x, y, w, h, faceCol, edgeCol, lw)
    rectangle(ax, 'Position', [x, y, w, h], 'Curvature', [0.10, 0.15], ...
        'FaceColor', faceCol, 'EdgeColor', edgeCol, 'LineWidth', lw);
end

function draw_tech_arrow(ax, x1, y1, x2, y2, col, lw)
    dx = x2 - x1;
    dy = y2 - y1;
    L  = hypot(dx, dy);
    if L < 1e-6
        return;
    end
    ux = dx / L;
    uy = dy / L;
    px = -uy;
    py =  ux;

    hw = 0.125;
    hl = 0.23;

    plot(ax, [x1, x2 - hl*ux], [y1, y2 - hl*uy], '-', 'Color', col, 'LineWidth', lw);
    patch(ax, [x2, x2 - hl*ux + hw*px, x2 - hl*ux - hw*px], ...
              [y2, y2 - hl*uy + hw*py, y2 - hl*uy - hw*py], ...
              col, 'EdgeColor', col, 'LineWidth', 0.8);
end
