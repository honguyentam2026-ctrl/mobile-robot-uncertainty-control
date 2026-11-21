# Tài liệu tham khảo và Phân định Tham khảo vs. Tự thực hiện

## 1. Bảng phân định Tham khảo và Tự thực hiện

| Thành phần | Nguồn / Mức độ tham khảo | Phần tự thực hiện & đóng góp kỹ thuật |
| :--- | :--- | :--- |
| `01_baseline_mathworks/run_baseline_pure_pursuit.m` | Tái lập theo ví dụ chính thức MathWorks *"Path Following for a Differential Drive Robot"* dùng `differentialDriveKinematics` và `controllerPurePursuit` (Robotics System Toolbox / Navigation Toolbox `24.2`, MATLAB `R2024b`). | Cấu trúc hóa vòng lặp lưu log, tách hàm tính chỉ tiêu định lượng cho bài toán S0 làm mốc so sánh. |
| `utils/wrap_to_pi.m`, `utils/compute_path_metrics.m` | Công thức hình học khoảng cách điểm–đoạn thẳng và chuẩn hóa góc chuẩn. | Tự viết toàn bộ hàm tính `crossTrackError`, RMSE, max error, RMS vận tốc và total variation của $\omega$. |
| `02_models_and_controllers/` (`robot_plant_step.m`, `sensor_odometry_landmark.m`, `landmark_measurement_model.m`, `complementary_pose_filter.m`, `ekf_standard.m`, `ekf_joseph_update.m`, `pid_speed_filtered.m`, `fuzzy_pid_scheduler.m`, `simulate_closed_loop.m`) | Cấu trúc Pure Pursuit & hàm FIS Sugeno (`sugfis`, `evalfis`) từ Fuzzy Logic Toolbox `24.2`; kiến thức nền động học robot vi sai, odometry, EKF định vị landmark và PID công nghiệp theo `[6]–[10]`. | Tự xây dựng mô hình vật lý của robot (plant) có sai số bánh ($\eta_v, \eta_\omega$), vùng sàn trượt $s(x,y)$ và đổi tải FOPDT $(K, T, \tau)$; tự cài đặt **Bộ lọc bù** và **Bộ lọc Kalman Mở rộng (EKF cập nhật tuần tự dạng Joseph + cổng Mahalanobis $\chi^2$)** trộn odometry + landmark kèm ước lượng trượt dọc, **PID có lọc khâu D (`PIDF`)** chống khuếch đại nhiễu, và **bộ lập lịch `Fuzzy-PID`** 9 luật tự chỉnh $(K_p, K_i, K_d)$ + bù trượt. |
| `03_experiments/` (`scenario_presets.m`, `run_monte_carlo_suite.m`, `run_parameter_sweeps.m`) | Thiết kế thực nghiệm theo hướng dẫn Rubric & File 4. | Tự viết bộ kịch bản `S0`–`S5`, khung chạy Monte Carlo `N = 30` seeds cố định cho `C1`–`C5` ($750$ lượt chạy), và khảo sát độ nhạy (`LookaheadDistance`, hệ số lọc bù $\alpha_{\text{pos}}$). |

## 2. Danh mục trích dẫn (định dạng IEEE)

1. The MathWorks, Inc., "Path Following for a Differential Drive Robot," *Robotics System Toolbox Documentation*, MATLAB R2024b (v24.2). [Online]. Available: https://www.mathworks.com/help/robotics/ug/path-following-for-differential-drive-robot.html (accessed Nov. 17, 2026).
2. The MathWorks, Inc., "`controllerPurePursuit` — Create controller to follow a set of waypoints," *Navigation Toolbox Reference*, MATLAB R2024b (v24.2), 2024.
3. The MathWorks, Inc., "Mobile Robot Kinematics Equations" & "`differentialDriveKinematics`," *Robotics System Toolbox Documentation*, MATLAB R2024b (v24.2). [Online]. Available: https://www.mathworks.com/help/robotics/ug/mobile-robot-kinematics-equations.html (accessed Nov. 18, 2026).
4. The MathWorks, Inc., *Fuzzy Logic Toolbox User's Guide* (`sugfis`, `evalfis`), MATLAB R2024b (v24.2), 2024.
5. J. M. Snider, "Automatic Steering Methods for Autonomous Automobile Path Tracking," Robotics Institute, Carnegie Mellon University, Pittsburgh, PA, USA, Tech. Rep. CMU-RI-TR-09-08, Feb. 2009.
6. K. J. Åström and T. Hägglund, *PID Controllers: Theory, Design, and Tuning*, 2nd ed. Research Triangle Park, NC, USA: ISA, 1995.
7. Z.-Y. Zhao, M. Tomizuka, and S. Isaka, "Fuzzy gain scheduling of PID controllers," *IEEE Trans. Syst., Man, Cybern.*, vol. 23, no. 5, pp. 1392–1398, Sep./Oct. 1993.
8. P. Corke, *Robotics, Vision and Control: Fundamental Algorithms in MATLAB*, 3rd ed. Cham, Switzerland: Springer, 2023; cùng bài giảng học phần *Kỹ thuật Robot* (Chương 3 & Chương 4), Khoa Điện tử – Viễn thông, Trường ĐH Bách khoa – ĐH Đà Nẵng.
9. California Institute of Technology (Caltech), "ME/CS 133(a,b): Kinematics and Odometry for a Differential Drive Vehicle." [Online]. Available: http://robotics.caltech.edu/wiki/images/d/d5/DiffDriveOdometry.pdf (accessed Nov. 18, 2026).
10. Massachusetts Institute of Technology (MIT), "6.186: A Primer on Odometry and Motor Control." [Online]. Available: https://web.mit.edu/6.186/2007/tutorials/odomtutorial/odomtutorial.pdf (accessed Nov. 18, 2026).

## 3. Ghi nhận sử dụng công cụ AI (theo Mục 7 — File 4)

- Công cụ trợ lý AI được sử dụng để hỗ trợ cấu trúc hóa mã nguồn MATLAB, thiết kế các hàm mô phỏng/thí nghiệm (Lọc bù, EKF, PIDF, Fuzzy-PID) và kiểm thử chạy trên `MATLAB R2024b`.
- Toàn bộ thuật toán, tham số, kết quả mô phỏng (`N = 30` seeds và quét tham số) đã được chạy kiểm chứng trực tiếp trên máy cục bộ và kiểm tra đối chiếu lại với yêu cầu học phần.
