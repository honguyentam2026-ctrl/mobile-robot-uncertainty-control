# Chủ đề 4: Điều khiển robot di động trong môi trường có nhiễu và bất định

Mã nguồn MATLAB mô phỏng điều khiển bám quỹ đạo cho robot vi sai (differential drive) trong môi trường có nhiễu cảm biến, trượt bánh, sai lệch thông số hình học và đổi tải giữa hành trình.

## 1. Môi trường & Thư viện phụ thuộc
- **MATLAB:** `R2024b` (`24.2.0.2712019`)
- **Toolboxes chính thức sử dụng:**
  - `Robotics System Toolbox` (`24.2`) — `differentialDriveKinematics`
  - `Navigation Toolbox` (`24.2`) — `controllerPurePursuit`
  - `Control System Toolbox` (`24.2`) — nền tảng điều khiển vòng tốc độ PI (S-IMC)
  - `Statistics and Machine Learning Toolbox` (`24.2`) — phân tích thống kê, boxplot, ngưỡng $\chi^2$ (`chi2inv`)

## 2. Cấu trúc thư mục
```text
Code/
├── 01_baseline_mathworks/         # [Tham khảo gốc] Tái lập baseline Pure Pursuit của MathWorks (S0 lý tưởng)
├── 02_models_and_controllers/     # [Tự thiết kế] Plant bất định, cảm biến, EKF 3D, Augmented EKF 5D, PI (S-IMC)
│   ├── default_sim_config.m
│   ├── robot_plant_step.m
│   ├── sensor_odometry_landmark.m
│   ├── landmark_measurement_model.m
│   ├── pi_speed_simc.m
│   ├── ekf_joseph_update.m
│   ├── ekf_standard.m
│   ├── ekf_augmented.m
│   ├── simulate_closed_loop.m
│   └── run_smoke_compare_c0_to_c4.m
├── 03_experiments/                # [Tự thiết kế] Bộ kịch bản S0–S5, Monte Carlo (30 seeds), khảo sát tham số
│   ├── scenario_presets.m
│   ├── run_monte_carlo_suite.m
│   └── run_parameter_sweeps.m
├── utils/
│   ├── wrap_to_pi.m
│   └── compute_path_metrics.m
├── README.md
└── REFERENCES.md
```

## 3. Hướng dẫn chạy toàn bộ thí nghiệm
Mở MATLAB tại thư mục `Code/` và chạy lần lượt:

1. **Bước 1 — Baseline MathWorks (S0 lý tưởng):**
   ```matlab
   run('01_baseline_mathworks/run_baseline_pure_pursuit.m')
   ```
2. **Bước 2 — Kiểm chứng so sánh 5 cấu hình `C0`–`C4` trên 1 seed tổ hợp bất định:**
   ```matlab
   run('02_models_and_controllers/run_smoke_compare_c0_to_c4.m')
   ```
3. **Bước 3a — Đánh giá Monte Carlo (`N = 30` seeds, kịch bản `S0`–`S5`):**
   ```matlab
   run('03_experiments/run_monte_carlo_suite.m')
   ```
   Xuất bảng `mc_summary_table.csv`, dữ liệu `.mat` và các đồ thị boxplot + kiểm tra nhất quán $\pm 2\sigma$ vào `03_experiments/results/`.
4. **Bước 3b — Khảo sát độ nhạy tham số (`LookaheadDistance` & tỷ lệ $\hat{\mathbf{V}}/\mathbf{V}_{\text{true}}$):**
   ```matlab
   run('03_experiments/run_parameter_sweeps.m')
   ```
   Xuất `param_lookahead_table.csv`, `param_ekf_ratio_table.csv` và `param_sweeps_analysis.png`.
