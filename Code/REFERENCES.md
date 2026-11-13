# Tài liệu tham khảo và Phân định Tham khảo vs. Tự thực hiện

## 1. Bảng phân định Tham khảo và Tự thực hiện

| Thành phần | Nguồn / Mức độ tham khảo | Phần tự thực hiện & đóng góp kỹ thuật |
| :--- | :--- | :--- |
| `01_baseline_mathworks/run_baseline_pure_pursuit.m` | Tái lập theo ví dụ chính thức MathWorks *"Path Following for a Differential Drive Robot"* dùng `differentialDriveKinematics` và `controllerPurePursuit` (Robotics System Toolbox / Navigation Toolbox `24.2`, MATLAB `R2024b`). | Cấu trúc hóa vòng lặp lưu log, tách hàm tính chỉ tiêu định lượng cho bài toán S0 làm mốc so sánh. |
| `utils/wrap_to_pi.m`, `utils/compute_path_metrics.m` | Công thức hình học khoảng cách điểm–đoạn thẳng và chuẩn hóa góc chuẩn. | Tự viết toàn bộ hàm tính `crossTrackError`, RMSE, max error, RMS vận tốc và total variation của $\omega$. |
| `02_models_and_controllers/` (`robot_plant_step.m`, `sensor_odometry_landmark.m`, `ekf_standard.m`, `ekf_augmented.m`, `pi_speed_simc.m`, `simulate_closed_loop.m`) | Công thức động học odometry, Jacobian $F_x, F_v, H_x$, Joseph-form và quy tắc chỉnh PI S-IMC theo Slide Chương 3–4 của giảng viên. | Tự xây dựng mô hình robot "thật" có sai số bánh ($\eta_v, \eta_\omega$), vùng sàn trượt $s(x,y)$ và đổi tải FOPDT $(K, T, \tau)$; tự cài đặt EKF 3D chuẩn, **Augmented EKF 5D** ước lượng và bù trực tuyến hệ số bất định, cùng vòng tốc độ PI (S-IMC). |
| `03_experiments/` (`scenario_presets.m`, `run_monte_carlo_suite.m`, `run_parameter_sweeps.m`) | Thiết kế thực nghiệm theo hướng dẫn Rubric & File 4. | Tự viết bộ kịch bản `S0`–`S5`, khung chạy Monte Carlo `N = 30` seeds cố định, kiểm tra nhất quán $\pm 2\sigma$ / `Mean_NIS`, và khảo sát độ nhạy (`LookaheadDistance`, tỷ lệ $\hat{\mathbf{V}}/\mathbf{V}_{\text{true}}$). |

## 2. Danh mục trích dẫn (định dạng IEEE)

1. The MathWorks, Inc., "Path Following for a Differential Drive Robot," *Robotics System Toolbox Documentation*, MATLAB R2024b (v24.2). [Online]. Available: https://www.mathworks.com/help/robotics/ug/path-following-for-a-differential-drive-robot.html (accessed Nov. 11, 2026).
2. The MathWorks, Inc., "`controllerPurePursuit` — Create controller to follow a set of waypoints," *Navigation Toolbox Reference*, MATLAB R2024b (v24.2).
3. The MathWorks, Inc., "`differentialDriveKinematics` — Create differential-drive vehicle model," *Robotics System Toolbox Reference*, MATLAB R2024b (v24.2).
4. J. M. Snider, "Automatic Steering Methods for Autonomous Automobile Path Tracking," Robotics Institute, Carnegie Mellon University, Pittsburgh, PA, USA, Tech. Rep. CMU-RI-TR-09-08, Feb. 2009.
5. S. Skogestad, "Simple analytic rules for model reduction and PID controller tuning," *Journal of Process Control*, vol. 13, no. 4, pp. 291–309, Jun. 2003.
6. Bài giảng học phần *Kỹ thuật Robot* — Chương 3 (Mobile Robots & Control) và Chương 4 (Localization & Mapping), lưu hành nội bộ.

## 3. Ghi nhận sử dụng công cụ AI (theo Mục 7 — File 4)

- Công cụ trợ lý AI được sử dụng để hỗ trợ cấu trúc hóa mã nguồn MATLAB, viết các hàm mô phỏng/thí nghiệm theo đúng công thức trong tài liệu học phần và kiểm thử chạy trên `MATLAB R2024b`.
- Toàn bộ thuật toán, tham số, kết quả mô phỏng (`N = 30` seeds và quét tham số) đã được chạy kiểm chứng trực tiếp trên máy cục bộ và kiểm tra đối chiếu lại với yêu cầu học phần.
