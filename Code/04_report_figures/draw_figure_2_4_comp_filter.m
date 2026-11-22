function draw_figure_2_4_comp_filter()
%DRAW_FIGURE_2_4_COMP_FILTER  Draw Figure 2.4 (2D Complementary Filter Block Diagram)
%   Generates Black-and-White (Monochrome) versions in Vietnamese and English
%   with large, crisp typography at 300 DPI:
%       - hinh_2_4_comp_bw_vi.png  (Black & White - Vietnamese)
%       - hinh_2_4_comp_bw_en.png  (Black & White - English)
%
%   Usage:
%       run('04_report_figures/draw_figure_2_4_comp_filter.m')

    thisDir = fileparts(mfilename('fullpath'));
    outDir  = fullfile(thisDir, 'results');
    if ~exist(outDir, 'dir')
        mkdir(outDir);
    end

    outBwVi = fullfile(outDir, 'hinh_2_4_comp_bw_vi.png');
    outBwEn = fullfile(outDir, 'hinh_2_4_comp_bw_en.png');

    render_bw_diagram('vi', outBwVi);
    render_bw_diagram('en', outBwEn);

    fprintf('Saved B&W VI : %s\n', outBwVi);
    fprintf('Saved B&W EN : %s\n', outBwEn);
end

function L = get_labels(lang)
    if strcmpi(lang, 'vi')
        L.tOdoTitle  = 'Odometry (10 Hz)';
        L.tPredTitle = '1. Dự đoán động học (2.5)';
        L.tW1Title   = 'Trọng số';
        L.tHpNote    = {'Ổn định ngắn hạn', '(thông cao — High-pass)'};

        L.tLmTitle   = 'Landmark (2 Hz)';
        L.tMeasTitle = '2. Tái tạo vị trí & Lọc ngoại lai';
        L.tGateEq    = '$\|\mathbf{p}_{\mathrm{meas}} - \hat{\mathbf{p}}_k^-\|_2 \le 1{,}50\;\mathrm{m}$';
        L.tW2Title   = 'Trọng số';
        L.tLpNote    = {'Chuẩn tham chiếu dài hạn', '(thông thấp — Low-pass)'};

        L.tOutTitle  = '3. Tư thế ước lượng (2.6)';
        L.tSlipLabel = 'Trích xuất độ trượt:';
        L.tDelay     = 'Trễ 1 bước';
    else
        L.tOdoTitle  = 'Odometry (10 Hz)';
        L.tPredTitle = '1. Kinematic Prediction (2.5)';
        L.tW1Title   = 'Weight';
        L.tHpNote    = {'Short-term smooth', '(High-pass branch)'};

        L.tLmTitle   = 'Landmark (2 Hz)';
        L.tMeasTitle = '2. Pose Reconstruction & Gate';
        L.tGateEq    = '$\|\mathbf{p}_{\mathrm{meas}} - \hat{\mathbf{p}}_k^-\|_2 \le 1.50\;\mathrm{m}$';
        L.tW2Title   = 'Weight';
        L.tLpNote    = {'Long-term reference', '(Low-pass branch)'};

        L.tOutTitle  = '3. Fused Pose Estimate (2.6)';
        L.tSlipLabel = 'Slip ratio output:';
        L.tDelay     = 'Unit Delay';
    end
end

function render_bw_diagram(lang, outPath)
    % Compact canvas (1120 x 530 px) with large 14-16.5 pt fonts so text is
    % bold, large, and effortless to read when embedded in a Word page.
    fig = figure('Visible', 'off', 'Color', 'w', ...
        'Units', 'pixels', 'Position', [80, 80, 1120, 530]);

    ax = axes('Parent', fig, 'Position', [0.005, 0.005, 0.99, 0.99]);
    hold(ax, 'on');
    axis(ax, 'equal');
    axis(ax, 'off');
    xlim(ax, [0.05, 14.55]);
    ylim(ax, [0.05, 6.30]);

    L = get_labels(lang);
    cBlack = [0 0 0];
    cWhite = [1 1 1];

    % ---------------------------------------------------------------------
    % TOP BRANCH: Odometry (10 Hz) -> 1. Prediction (2.5) -> Weight
    % ---------------------------------------------------------------------
    draw_round_box(ax, 0.15, 4.05, 2.35, 1.45, cWhite, cBlack, 2.0);
    text(ax, 1.325, 5.05, L.tOdoTitle, 'FontName', 'Times New Roman', ...
        'FontSize', 14.5, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', cBlack);
    text(ax, 1.325, 4.42, '$(\delta_d,\;\delta_\theta)$', 'Interpreter', 'latex', ...
        'FontSize', 16.5, 'HorizontalAlignment', 'center', 'Color', cBlack);

    draw_tech_arrow(ax, 2.50, 4.95, 3.20, 4.95, cBlack, 2.0);

    draw_round_box(ax, 3.20, 3.82, 4.45, 1.82, cWhite, cBlack, 2.0);
    text(ax, 5.425, 5.32, L.tPredTitle, 'FontName', 'Times New Roman', ...
        'FontSize', 14.5, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', cBlack);
    text(ax, 5.425, 4.74, '$\hat{x}_k^- = \hat{x}_{k-1} + \delta_d\cos\hat{\theta}_{k-1}$', ...
        'Interpreter', 'latex', 'FontSize', 15.0, 'HorizontalAlignment', 'center', 'Color', cBlack);
    text(ax, 5.425, 4.18, '$\hat{y}_k^- = \hat{y}_{k-1} + \delta_d\sin\hat{\theta}_{k-1},\;\;\hat{\theta}_k^- = \hat{\theta}_{k-1} + \delta_\theta$', ...
        'Interpreter', 'latex', 'FontSize', 14.0, 'HorizontalAlignment', 'center', 'Color', cBlack);

    draw_tech_arrow(ax, 7.65, 4.73, 8.20, 4.73, cBlack, 2.0);

    draw_round_box(ax, 8.20, 4.02, 2.00, 1.42, cWhite, cBlack, 2.0);
    text(ax, 9.20, 5.04, L.tW1Title, 'FontName', 'Times New Roman', ...
        'FontSize', 14.5, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', cBlack);
    text(ax, 9.20, 4.40, '$(1 - \alpha_{\mathrm{pos}},\; 1 - \alpha_\theta)$', ...
        'Interpreter', 'latex', 'FontSize', 14.2, 'HorizontalAlignment', 'center', 'Color', cBlack);

    text(ax, 9.20, 5.86, L.tHpNote, 'FontName', 'Times New Roman', ...
        'FontSize', 13.0, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', cBlack);

    % ---------------------------------------------------------------------
    % BOTTOM BRANCH: Landmark (2 Hz) -> 2. Measurement & Gate -> Weight
    % ---------------------------------------------------------------------
    draw_round_box(ax, 0.15, 1.72, 2.35, 1.45, cWhite, cBlack, 2.0);
    text(ax, 1.325, 2.72, L.tLmTitle, 'FontName', 'Times New Roman', ...
        'FontSize', 14.5, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', cBlack);
    text(ax, 1.325, 2.08, '$(r_i,\;\beta_i),\;\; M \ge 2$', 'Interpreter', 'latex', ...
        'FontSize', 15.5, 'HorizontalAlignment', 'center', 'Color', cBlack);

    draw_tech_arrow(ax, 2.50, 2.44, 3.20, 2.44, cBlack, 2.0);

    draw_round_box(ax, 3.20, 1.50, 4.45, 1.88, cWhite, cBlack, 2.0);
    text(ax, 5.425, 3.06, L.tMeasTitle, 'FontName', 'Times New Roman', ...
        'FontSize', 14.5, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', cBlack);
    text(ax, 5.425, 2.48, '$\mathbf{q}_{\mathrm{meas}} = [x_{\mathrm{meas}},\; y_{\mathrm{meas}},\; \theta_{\mathrm{meas}}]^T$', ...
        'Interpreter', 'latex', 'FontSize', 14.2, 'HorizontalAlignment', 'center', 'Color', cBlack);
    text(ax, 5.425, 1.88, L.tGateEq, 'Interpreter', 'latex', ...
        'FontSize', 14.2, 'HorizontalAlignment', 'center', 'Color', cBlack);

    draw_tech_arrow(ax, 7.65, 2.44, 8.20, 2.44, cBlack, 2.0);

    draw_round_box(ax, 8.20, 1.73, 2.00, 1.42, cWhite, cBlack, 2.0);
    text(ax, 9.20, 2.75, L.tW2Title, 'FontName', 'Times New Roman', ...
        'FontSize', 14.5, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', cBlack);
    text(ax, 9.20, 2.12, '$(\alpha_{\mathrm{pos}},\; \alpha_\theta)$', ...
        'Interpreter', 'latex', 'FontSize', 15.5, 'HorizontalAlignment', 'center', 'Color', cBlack);

    text(ax, 9.20, 1.32, L.tLpNote, 'FontName', 'Times New Roman', ...
        'FontSize', 13.0, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', cBlack);

    % ---------------------------------------------------------------------
    % SUMMING JUNCTION (+) AND 3. OUTPUT BOX (2.6)
    % ---------------------------------------------------------------------
    sx = 10.75;
    sy = 3.58;
    sr = 0.36;
    th = linspace(0, 2*pi, 100);
    patch(ax, sx + sr*cos(th), sy + sr*sin(th), 'w', 'EdgeColor', 'k', 'LineWidth', 2.0);
    text(ax, sx, sy, '$+$', 'Interpreter', 'latex', 'FontSize', 20, ...
        'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');

    plot(ax, [10.20, sx], [4.73, 4.73], '-', 'Color', cBlack, 'LineWidth', 2.0);
    draw_tech_arrow(ax, sx, 4.73, sx, sy + sr, cBlack, 2.0);

    plot(ax, [10.20, sx], [2.44, 2.44], '-', 'Color', cBlack, 'LineWidth', 2.0);
    draw_tech_arrow(ax, sx, 2.44, sx, sy - sr, cBlack, 2.0);

    draw_tech_arrow(ax, sx + sr, sy, 11.45, sy, cBlack, 2.0);

    draw_round_box(ax, 11.45, 2.58, 3.00, 2.00, cWhite, cBlack, 2.2);
    text(ax, 12.95, 4.24, L.tOutTitle, 'FontName', 'Times New Roman', ...
        'FontSize', 13.5, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', cBlack);
    text(ax, 12.95, 3.70, '$\hat{\mathbf{q}}_k = [\hat{x}_k,\; \hat{y}_k,\; \hat{\theta}_k]^T$', ...
        'Interpreter', 'latex', 'FontSize', 15.0, 'HorizontalAlignment', 'center', 'Color', cBlack);
    text(ax, 12.95, 3.22, L.tSlipLabel, 'FontName', 'Times New Roman', ...
        'FontSize', 13.0, 'FontAngle', 'italic', 'HorizontalAlignment', 'center', 'Color', cBlack);
    text(ax, 12.95, 2.84, '$\mathrm{slip}_{\mathrm{est}}(k) \in [0,\;0.50]$', ...
        'Interpreter', 'latex', 'FontSize', 14.0, 'HorizontalAlignment', 'center', 'Color', cBlack);

    % ---------------------------------------------------------------------
    % FEEDBACK LOOP (z^-1) FROM OUTPUT TO PREDICTION (2.5)
    % ---------------------------------------------------------------------
    fbX = 12.95;
    fbY = 0.56;
    upX = 2.78;
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
    draw_tech_arrow(ax, upX, inY, 3.20, inY, cBlack, 1.8);

    text(ax, 3.15, 0.86, '$\hat{\mathbf{q}}_{k-1} = [\hat{x}_{k-1},\; \hat{y}_{k-1},\; \hat{\theta}_{k-1}]^T$', ...
        'Interpreter', 'latex', 'FontSize', 14.2, 'HorizontalAlignment', 'left', 'Color', cBlack);

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
