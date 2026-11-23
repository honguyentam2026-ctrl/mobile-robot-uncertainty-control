# Chủ đề 4: Điều khiển robot di động trong môi trường có nhiễu và bất định

Mã nguồn MATLAB mô phỏng điều khiển bám quỹ đạo cho robot vi sai (differential drive) trong môi trường có nhiễu cảm biến, trượt bánh, sai lệch thông số hình học và đổi tải giữa hành trình, sử dụng **Bộ lọc bù (Complementary Filter)** hoặc **Bộ lọc Kalman Mở rộng (EKF)** kết hợp **PID có lọc (PIDF)** và **Fuzzy-PID**.

## 1. Môi trường & Thư viện phụ thuộc
- **MATLAB:** `R2024b` (`24.2.0.2712019`)
- **Toolboxes chính thức sử dụng:**
  - `Robotics System Toolbox` (`24.2`) — `differentialDriveKinematics`
  - `Navigation Toolbox` (`24.2`) — `controllerPurePursuit`
  - `Control System Toolbox` (`24.2`) — nền tảng vòng điều khiển tốc độ PID có lọc (`PIDF`)
  - `Fuzzy Logic Toolbox` (`24.2`) — `sugfis`, `evalfis` cho bộ lập lịch `Fuzzy-PID`
  - `Statistics and Machine Learning Toolbox` (`24.2`) — phân tích thống kê, boxplot

## 2. Cấu trúc thư mục
```text
Code/
├── 01_baseline_mathworks/         # [Tham khảo gốc] Tái lập baseline Pure Pursuit của MathWorks (S0 lý tưởng)
│   ├── run_baseline_pure_pursuit.m
│   └── results/
├── 02_models_and_controllers/     # [Tự thiết kế] Plant bất định, cảm biến, Lọc bù, EKF, PIDF, Fuzzy-PID
│   ├── default_sim_config.m
│   ├── robot_plant_step.m
│   ├── sensor_odometry_landmark.m
│   ├── landmark_measurement_model.m
│   ├── complementary_pose_filter.m
│   ├── ekf_standard.m
│   ├── ekf_joseph_update.m
│   ├── pid_speed_filtered.m
│   ├── fuzzy_pid_scheduler.m
│   ├── simulate_closed_loop.m
│   ├── run_smoke_compare_c0_to_c4.m
│   └── results/
├── 03_experiments/                # [Tự thiết kế] Bộ kịch bản S0–S5, Monte Carlo (30 seeds), khảo sát tham số
│   ├── scenario_presets.m
│   ├── run_monte_carlo_suite.m
│   ├── run_parameter_sweeps.m
│   └── results/
├── 04_report_figures/             # [Tự thiết kế] Vẽ sơ đồ khối (Hình 2.4, 2.5) & đồ thị phân tích Chương 4
│   ├── draw_figure_2_4_comp_filter.m
│   ├── draw_figure_2_5_ekf.m
│   ├── draw_chapter4_analysis_figures.m
│   └── results/
├── utils/
│   ├── wrap_to_pi.m
│   └── compute_path_metrics.m
├── README.md
└── REFERENCES.md
```

## 3. 6 cấu hình điều khiển đối chứng (`C0`–`C5`)
1. **`C0_ideal`**: `controllerPurePursuit` gốc MathWorks dùng vị trí lý tưởng (mốc chuẩn trên).
2. **`C1_noisy`**: `controllerPurePursuit` dùng odometry thô không lọc, vòng trong hở (mốc dưới).
3. **`C2_comp`**: **Lọc bù (Complementary Filter)** khử nhiễu vị trí + `controllerPurePursuit`, vòng trong hở.
4. **`C3_comp_pidf`**: **Lọc bù** + `controllerPurePursuit` + **PID có lọc ở khâu D (`PIDF`)** với hệ số cố định.
5. **`C4_comp_fuzzy_pidf`**: **Lọc bù** + `controllerPurePursuit` + **Fuzzy-PID** tự chỉnh $(K_p, K_i, K_d)$ và bù trượt/tầm nhìn theo mức độ sai số & trượt.
6. **`C5_ekf_fuzzy_pidf`**: **Bộ lọc Kalman Mở rộng (EKF cập nhật tuần tự dạng Joseph + cổng $\chi^2$)** + `controllerPurePursuit` + **Fuzzy-PID** tự chỉnh $(K_p, K_i, K_d)$ và bù trượt/tầm nhìn.

## 4. Hướng dẫn chạy toàn bộ thí nghiệm
Mở MATLAB tại thư mục `Code/` và chạy lần lượt:

1. **Bước 1 — Baseline MathWorks (S0 lý tưởng):**
   ```matlab
   run('01_baseline_mathworks/run_baseline_pure_pursuit.m')
   ```
2. **Bước 2 — Kiểm chứng so sánh 6 cấu hình `C0`–`C5` trên 1 seed tổ hợp bất định:**
   ```matlab
   run('02_models_and_controllers/run_smoke_compare_c0_to_c4.m')
   ```
3. **Bước 3a — Đánh giá Monte Carlo (`N = 30` seeds, kịch bản `S1`–`S5`, cấu hình `C1`–`C5` = 750 lượt chạy):**
   ```matlab
   run('03_experiments/run_monte_carlo_suite.m')
   ```
   Xuất bảng `mc_summary_table.csv`, dữ liệu `.mat` và các đồ thị `mc_cte_and_time.png`, `mc_s5_detailed_analysis.png` vào `03_experiments/results/`.
4. **Bước 3b — Khảo sát độ nhạy tham số (`LookaheadDistance` & hệ số lọc bù $\alpha_{\text{pos}}$):**
   ```matlab
   run('03_experiments/run_parameter_sweeps.m')
   ```
   Xuất `param_lookahead_table.csv`, `param_comp_gain_table.csv` và `param_sweeps_analysis.png`.
5. **Bước 4 — Vẽ lại bộ sơ đồ khối & đồ thị phân tích dùng trong báo cáo (`Hình 2.4, 2.5, 4.3, 4.4`):**
   ```matlab
   run('04_report_figures/draw_figure_2_4_comp_filter.m')
   run('04_report_figures/draw_figure_2_5_ekf.m')
   run('04_report_figures/draw_chapter4_analysis_figures.m')
   ```
   Xuất các ảnh `hinh_2_4_*.png`, `hinh_2_5_*.png`, `ch4_metrics_dashboard_*.png` và `ch4_scenarios_s1_to_s4_*.png` (bản Tiếng Việt & Tiếng Anh) vào `04_report_figures/results/` *(cần chạy Bước 3a trước để có `mc_summary_table.csv`)*.
