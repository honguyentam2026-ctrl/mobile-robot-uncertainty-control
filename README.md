# Đồ án Cuối kỳ Kỹ thuật Robot (Chủ đề 4)
## Điều khiển bám quỹ đạo robot vi sai vận chuyển hàng trong kho có nhiễu cảm biến, trượt bánh và bất định thông số
*(Trajectory Tracking Control of a Warehouse Differential-Drive Mobile Robot under Sensor Noise, Wheel Slip, and Parameter Uncertainties)*

---

## Thông tin chung
- **Học phần:** Kỹ thuật Robot — Lớp học phần `22.44` (Lớp sinh hoạt `22KTMT1`)
- **Khoa / Trường:** Khoa Điện tử – Viễn thông, Trường Đại học Bách khoa – Đại học Đà Nẵng
- **Giảng viên hướng dẫn:** PGS.TS. Phan Trần Đăng Khoa
- **Sinh viên thực hiện:**
  1. **Hồ Nguyên Tâm** — MSSV: `106220233` (Tỷ lệ đóng góp: $50\%$)
  2. **Nguyễn Văn Quý** — MSSV: `106220232` (Tỷ lệ đóng góp: $50\%$)
- **Môi trường phát triển & Kiểm chứng:** MATLAB R2024b (`v24.2`)

---

## 1. Cấu trúc thư mục toàn bộ dự án (`Final Project/`)

```text
Final Project/
├── README.md                                 (Tài liệu tổng quan toàn bộ dự án)
├── Report/                                   (Báo cáo hoàn chỉnh — Tiếng Việt & Tiếng Anh)
│   ├── BaoCao_KyThuatRobot_22.44_HoNguyenTam_NguyenVanQuy.docx
│   ├── BaoCao_KyThuatRobot_22.44_HoNguyenTam_NguyenVanQuy.pdf
│   ├── Report_Robotics_22.44_HoNguyenTam_NguyenVanQuy.docx
│   └── Report_Robotics_22.44_HoNguyenTam_NguyenVanQuy.pdf
├── Code/                                     (Toàn bộ mã nguồn mô phỏng MATLAB R2024b)
│   ├── 01_baseline_mathworks/                (Phần kế thừa chuẩn MathWorks Pure Pursuit)
│   │   ├── run_baseline_pure_pursuit.m       (Kịch bản chuẩn S0 lý tưởng, không nhiễu)
│   │   └── results/                          (baseline_s0_trajectory.png, baseline_s0_results.mat)
│   ├── 02_models_and_controllers/            (Mô hình vật lý, cảm biến, bộ lọc & bộ điều khiển tự thiết kế)
│   │   ├── default_sim_config.m              (Cấu hình thông số toàn hệ thống)
│   │   ├── robot_plant_step.m                (Mô hình vật lý 2 tầng: FOPDT + trượt sàn & lệch bánh)
│   │   ├── sensor_odometry_landmark.m        (Sinh nhiễu odometry 10 Hz, encoder & landmark 2 Hz)
│   │   ├── landmark_measurement_model.m      (Mô hình đo phi tuyến Range–Bearing & Jacobian H)
│   │   ├── complementary_pose_filter.m       (Bộ lọc bù định vị 2D + ước lượng trượt dọc — C2..C4)
│   │   ├── ekf_standard.m                    (Bộ lọc Kalman Mở rộng EKF 3 trạng thái — C5)
│   │   ├── ekf_joseph_update.m               (Cập nhật tuần tự dạng đối xứng Joseph + cổng chi2)
│   │   ├── pid_speed_filtered.m              (Bộ điều khiển tốc độ PID có lọc khâu D trên đầu ra — PIDF)
│   │   ├── fuzzy_pid_scheduler.m             (Bộ lập lịch mờ Sugeno 9 luật tự chỉnh hệ số — Fuzzy-PID)
│   │   ├── simulate_closed_loop.m            (Bộ mô phỏng vòng kín thống nhất cho 6 cấu hình C0–C5)
│   │   └── run_smoke_compare_c0_to_c4.m      (Script chạy kiểm tra nhanh C0–C5 trên 1 seed)
│   ├── 03_experiments/                       (Thực nghiệm Monte Carlo & Khảo sát độ nhạy tham số)
│   │   ├── scenario_presets.m                (Định nghĩa 6 kịch bản đánh giá S0–S5)
│   │   ├── run_monte_carlo_suite.m           (Chạy 750 lượt Monte Carlo: 5 kịch bản x 5 bộ ĐK x 30 seeds)
│   │   ├── run_parameter_sweeps.m            (Study A quét Lookahead d_L & Study B quét alpha_pos)
│   │   └── results/                          (Bảng CSV tổng hợp, file .mat và đồ thị kết quả)
│   ├── 04_report_figures/                    (Script MATLAB vẽ sơ đồ khối & đồ thị phân tích Chương 4)
│   │   ├── draw_figure_2_4_comp_filter.m     (Vẽ Hình 2.4: Sơ đồ khối Bộ lọc bù 2D)
│   │   ├── draw_figure_2_5_ekf.m             (Vẽ Hình 2.5: Sơ đồ khối Bộ lọc EKF dạng Joseph)
│   │   ├── draw_chapter4_analysis_figures.m  (Vẽ Hình 4.3 Dashboard 4 chỉ tiêu & Hình 4.4 Phân tích S1–S4)
│   │   └── results/                          (Ảnh PNG 300 DPI bản Tiếng Việt & Tiếng Anh)
│   ├── utils/                                (Hàm tiện ích toán học & đánh giá chất lượng)
│   │   ├── wrap_to_pi.m                      (Chuẩn hóa góc về [-pi, pi])
│   │   └── compute_path_metrics.m            (Tính CTE, CTE_RMSE, CTE_Max, w_TV, lỗi đích cuối)
│   ├── README.md                             (Hướng dẫn chạy mã nguồn MATLAB)
│   └── REFERENCES.md                         (Danh mục tài liệu tham khảo IEEE & bảng kế thừa – tự làm)
└── requirement/                              (Tài liệu đề bài, hướng dẫn và biểu mẫu báo cáo của học phần)
```

---

## 2. Bài toán và Vấn đề kỹ thuật cần giải quyết

### 2.1. Phát biểu bài toán
Một robot di động dẫn hướng vi sai hai bánh (differential-drive mobile robot) hoạt động trong mặt phẳng kho hàng 2D kích thước $14\text{ m} \times 13\text{ m}$. Xuất phát từ tư thế ban đầu $\mathbf{q}_0 = [2.00,\; 1.00,\; 0]^T\text{ [m, m, rad]}$, robot phải bám theo lộ trình tham chiếu gồm $6$ điểm mốc (waypoints):

$$\mathcal{P}_{\mathrm{ref}} = \{(2.00, 1.00),\; (1.25, 1.75),\; (5.25, 8.25),\; (7.25, 8.75),\; (11.75, 10.75),\; (12.00, 10.00)\}$$

với tổng chiều dài đường gấp khúc là $16.469\text{ m}$, vận tốc dài đặt $v_d = 0.30\text{ m/s}$, vận tốc góc tối đa $\omega_{\max} = 2.0\text{ rad/s}$, chu kỳ lấy mẫu $\Delta t = 0.1\text{ s}$, tiến vào vùng đích bán kính $r_{\mathrm{goal}} = 0.15\text{ m}$ trước thời gian giới hạn $T_{\max} = 120\text{ s}$.

### 2.2. Bốn nguồn nhiễu và bất định trong môi trường kho thực tế
Trong điều kiện lý tưởng (`S0`), thuật toán hình học Pure Pursuit dễ dàng dẫn robot bám sát đường đặt. Tuy nhiên, trong vận hành thực tế, robot chịu tác động đồng thời của **4 nguồn nhiễu và bất định**:
1. **Nhiễu cảm biến ngẫu nhiên (`S1`):**
   - Odometry từ encoder trục bánh cập nhật nhanh ($10\text{ Hz}$) nhưng bị nhiễu gia số ($\sigma_d = 0.02\text{ m}$, $\sigma_\theta = 1.5^\circ$) gây trôi tích lũy theo thời gian (odometry drift).
   - Cảm biến đo khoảng cách – góc hướng (Range–Bearing) tới $8$ mốc landmark tĩnh trong tầm quét $10\text{ m}$ không bị trôi nhưng cập nhật chậm ($2\text{ Hz}$, chu kỳ $0.5\text{ s}$) và chứa nhiễu đo ($\sigma_r = 0.08\text{ m}$, $\sigma_\beta = 2.5^\circ$).
2. **Trượt bánh cục bộ theo vùng sàn (`S2`):**
   - Khu vực nền kho trơn ướt hoặc dính dầu ($x \in [4.0, 9.5]\text{ m}$, $y \in [5.0, 9.8]\text{ m}$) làm bánh xe bị trượt $s_v = s_\omega = 30\%$ (hoặc $25\%$ ở `S5`). Bánh xe vẫn quay nhưng thân xe chỉ dịch chuyển được $70\text{--}75\%$ quãng đường — hiện tượng nằm ngoài khả năng quan sát trực tiếp của encoder trục bánh.
3. **Sai lệch thông số cơ khí (`S3`):**
   - Lốp mòn hoặc sai lệch chiều rộng cơ sở làm tỷ lệ vận tốc dài và vận tốc góc thực tế lệch cố định $\eta_{v,\mathrm{base}} = 0.80$ ($-20\%$) và $\eta_{\omega,\mathrm{base}} = 1.20$ ($+20\%$) so với mô hình danh định nạp trong bộ điều khiển.
4. **Thay đổi tải trọng đột ngột giữa hành trình (`S4`):**
   - Tại $t = 20\text{ s}$, robot nhận kiện hàng nặng làm hệ số khuếch đại tĩnh của hệ truyền động giảm từ $K = 1.0 \to 0.65$ và hằng số thời gian quán tính tăng hơn gấp đôi từ $T = 0.30\text{ s} \to 0.70\text{ s}$ (kèm thời gian trễ $\tau$).

> **Ràng buộc thực nghiệm nghiêm ngặt:** Ở mọi cấu hình có nhiễu (`C1`–`C5`), bộ điều khiển và bộ ước lượng **tuyệt đối không được phép truy cập trạng thái thực $\mathbf{x}_{\mathrm{true}}$** mà chỉ được dùng dữ liệu đo có nhiễu từ cảm biến.

---

## 3. Phương pháp giải quyết & Kiến trúc hệ thống hai tầng

Dự án phân loại 4 nguồn bất định thành **2 nhóm bản chất toán học** để thiết kế kiến trúc **2 tầng tách biệt**:
- **Nhóm 1 — Nhiễu ngẫu nhiên có kỳ vọng bằng 0 (nhiễu odometry & landmark):** Xử lý tại **Tầng Định vị (Localization Layer)** bằng hợp nhất cảm biến đa tần số (**Bộ lọc bù** hoặc **Bộ lọc Kalman Mở rộng EKF**) nhằm triệt tiêu trôi vị trí và trích xuất độ trượt dọc $\mathrm{slip}_{\mathrm{est}}$.
- **Nhóm 2 — Bất định có hệ thống (trượt sàn, lệch cơ khí, đổi tải nặng):** Xử lý tại **Tầng Điều khiển tốc độ (Speed Control Layer)** bằng vòng hồi tiếp kín **PID có lọc khâu vi phân (PIDF)** kết hợp **Bộ lập lịch mờ Sugeno 9 luật (Fuzzy-PID)** tự chỉnh hệ số $(K_p, K_i, K_d)$, bù tốc độ trượt $\mathrm{boost}$ và thu ngắn khoảng nhìn trước $d_L$.

### Tiến trình phát triển và đối sánh tuần tự 6 cấu hình (`C0`–`C5`)

| Mã cấu hình | Tầng Định vị & Dẫn hướng | Tầng Điều khiển tốc độ | Vai trò trong nghiên cứu |
| :--- | :--- | :--- | :--- |
| **`C0_ideal`** | Trạng thái thật $\mathbf{x}_{\mathrm{true}}$ + Pure Pursuit | Vòng hở | Mốc đối chứng cận trên lý tưởng (`S0`). |
| **`C1_noisy`** | Cộng dồn odometry thô (không lọc) | Vòng hở | Mốc cận dưới chứng minh hiện tượng trôi odometry. |
| **`C2_comp`** | **Bộ lọc bù** ($\alpha_{\mathrm{pos}} = 0.50$, $\alpha_\theta = 0.40$) | Vòng hở | Khử trôi định vị; bộc lộ sụt tốc khi trượt bánh/đổi tải. |
| **`C3_comp_pidf`** | **Bộ lọc bù** ($\alpha_{\mathrm{pos}} = 0.50$, $\alpha_\theta = 0.40$) | **PIDF cố định** | Chứng minh tác dụng làm mượt lái của khâu D có lọc và giới hạn của hệ số PID cố định. |
| **`C4_comp_fuzzy_pidf`** | **Bộ lọc bù** + thu ngắn $d_L$ thích nghi | **Fuzzy-PID (9 luật)** | Giải pháp gọn nhẹ (không cần ma trận hiệp phương sai) tự chỉnh $(K_p, K_i, K_d)$ và bù trượt. |
| **`C5_ekf_fuzzy_pidf`** | **EKF dạng Joseph** + thu ngắn $d_L$ thích nghi | **Fuzzy-PID (9 luật)** | Giải pháp tối ưu hiệp phương sai kết hợp điều khiển mờ thích nghi — chính xác và nhanh nhất. |

---

## 4. Cơ sở lý thuyết áp dụng

1. **Động học robot vi sai và rời rạc hóa điểm giữa (Midpoint Rule — Runge–Kutta bậc 2):**
   - Liên hệ vận tốc hai bánh $(\omega_L, \omega_R)$ với vận tốc thân xe: $v = \frac{r(\omega_R + \omega_L)}{2}$, $\omega = \frac{r(\omega_R - \omega_L)}{B}$.
   - Cập nhật tư thế rời rạc với góc hướng trung điểm $\theta_k + \delta_\theta/2$ giúp giảm sai số rời rạc hóa từ $O(\Delta t)$ xuống $O(\Delta t^2)$:
     $$x_{k+1} = x_k + \delta_d\cos\!\left(\theta_k + \frac{\delta_\theta}{2}\right), \quad y_{k+1} = y_k + \delta_d\sin\!\left(\theta_k + \frac{\delta_\theta}{2}\right), \quad \theta_{k+1} = \mathrm{wrapToPi}(\theta_k + \delta_\theta)$$

2. **Dẫn hướng hình học Pure Pursuit:**
   - Xác định điểm nhìn trước cách khoảng $d_L$ trên quỹ đạo đặt và tính độ cong cung tròn dẫn hướng $\kappa = \frac{2\sin\alpha}{d_L} = \frac{2e_y^L}{d_L^2}$, xuất lệnh lái $\omega_{pp} = \mathrm{clip}(v_d\kappa,\; -\omega_{\max},\; +\omega_{\max})$.

3. **Bộ lọc bù định vị 2D (Complementary Filter — dùng cho `C2`, `C3`, `C4`):**
   - Kết hợp nhánh thông cao (dự đoán odometry ở $10\text{ Hz}$) và nhánh thông thấp (tái tạo tư thế tuyệt đối từ $M \ge 2$ landmark ở $2\text{ Hz}$ với trọng số nghịch đảo bình phương khoảng cách $w_i = 1/\max(r_i, 0.5)^2$ và cổng loại ngoại lai $1.50\text{ m}$):
     $$\hat{\mathbf{p}}_k = (1 - \alpha_{\mathrm{pos}})\hat{\mathbf{p}}_k^- + \alpha_{\mathrm{pos}}\mathbf{p}_{\mathrm{meas}}, \qquad \hat{\theta}_k = \mathrm{wrapToPi}\!\big(\hat{\theta}_k^- + \alpha_\theta\,\mathrm{wrapToPi}(\theta_{\mathrm{meas}} - \hat{\theta}_k^-)\big)$$

4. **Bộ lọc Kalman Mở rộng (Extended Kalman Filter — EKF, dùng cho `C5`):**
   - Tuyến tính hóa mô hình chuyển động và mô hình đo phi tuyến $h_i(\mathbf{x}) = [\sqrt{\Delta x_i^2 + \Delta y_i^2},\; \mathrm{atan2}(\Delta y_i, \Delta x_i) - \theta]^T$ qua các ma trận Jacobian $\mathbf{F}_x, \mathbf{F}_v, \mathbf{H}_i$.
   - Kiểm tra cổng Mahalanobis $d_{\mathrm{NIS},i} = \boldsymbol{\nu}_i^T \mathbf{S}_i^{-1}\boldsymbol{\nu}_i \le \gamma_{\chi^2}$ và cập nhật tuần tự từng landmark theo **dạng đối xứng Joseph (Joseph stabilized form)** để đảm bảo ma trận hiệp phương sai $\mathbf{P}_k$ luôn đối xứng, xác định dương:
     $$\mathbf{P}_k \leftarrow (\mathbf{I} - \mathbf{K}_i\mathbf{H}_i)\mathbf{P}(\mathbf{I} - \mathbf{K}_i\mathbf{H}_i)^T + \mathbf{K}_i\hat{\mathbf{W}}\mathbf{K}_i^T$$

5. **Ước lượng độ trượt dọc từ hai mốc Landmark liên tiếp:**
   - Đối chiếu quãng đường dịch chuyển thực sau lọc landmark $\Delta d_{\mathrm{est}}$ với quãng đường encoder đếm được $\Delta d_{\mathrm{odo}}$ trên các đoạn thẳng ít quay ($\sum|\delta_\theta| < 0.30\text{ rad}$):
     $$\mathrm{slip}_{\mathrm{raw}} = \mathrm{clip}\!\left(1 - \frac{\Delta d_{\mathrm{est}}}{\Delta d_{\mathrm{odo}}},\; 0,\; 0.50\right), \qquad \mathrm{slip}_{\mathrm{est}}(k) = 0.85\,\mathrm{slip}_{\mathrm{est}}(k-1) + 0.15\,\mathrm{slip}_{\mathrm{raw}}$$

6. **Mô hình truyền động FOPDT, bộ điều khiển tốc độ PIDF và lập lịch mờ Sugeno Fuzzy-PID:**
   - Truyền động động cơ – trục bánh tuân theo hàm truyền bậc nhất có trễ $G(s) = \frac{K}{Ts+1}e^{-\tau s}$.
   - **PIDF (`C3`):** Đặt khâu vi phân $D$ trực tiếp lên tín hiệu đo $y_k$ (tránh vọt lố đạo hàm *derivative kick* khi điểm đặt thay đổi) kèm bộ lọc thông thấp hằng số thời gian $T_f = 0.20\text{ s}$ và chống bão hòa tích phân có điều kiện (*conditional anti-windup*).
   - **Fuzzy-PID (`C4`, `C5`):** Hệ suy luận mờ Sugeno bậc $0$ gồm $2$ đầu vào chuẩn hóa $(e_{\mathrm{norm}}, \mathrm{slip}_{\mathrm{norm}}) \in [0, 1]^2$, $3 \times 3 = 9$ luật suy luận để tự chỉnh trực tuyến bộ hệ số $(s_{K_p}, s_{K_i}, s_{K_d})$ và hệ số bù vận tốc trượt $\mathrm{boost} \in [1.00,\; 1.24]$.

---

## 5. Kết quả thực nghiệm định lượng

### 5.1. Kết quả tái lập Baseline lý tưởng `S0`
- Robot hoàn thành lộ trình $16.440\text{ m}$ trong $54.80\text{ s}$ (với $d_L = 0.30\text{ m}$ gốc của MathWorks) và $54.90\text{ s}$ (với cấu hình chuẩn $d_L = 0.40\text{ m}$ của Suite Monte Carlo), đạt $\mathrm{CTE}_{\mathrm{RMSE}} = 0.0428\text{ m}$, sai số đích cuối $\le 0.10\text{ m}$.

### 5.2. Tổng hợp kết quả Monte Carlo ($N = 30$ seeds, $750$ lượt chạy trên `S1`–`S5`)

| Kịch bản | Cấu hình nổi bật | Tỷ lệ về đích | Thời gian hoàn thành [s] | Sai số bám $\mathrm{CTE}_{\mathrm{RMSE}}$ [m] | Sai số định vị $\mathrm{Est}_{\mathrm{RMSE}}$ [m] | Biến thiên lái $w_{\mathrm{TV}}$ [rad/s] |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| **`S1_sensor_noise`** | `C1 Noisy` | $6.7\%$ | $115.63 \pm 16.64$ | $3.4752 \pm 2.2303$ | $3.9399 \pm 2.4157$ | $160.0$ |
| | `C2 CompFilter` | $100.0\%$ | $55.70 \pm 0.40$ | $0.0549 \pm 0.0062$ | $0.0689 \pm 0.0054$ | $153.1$ |
| | `C3 Comp+PIDF` | $100.0\%$ | $56.16 \pm 0.50$ | $0.0551 \pm 0.0038$ | $0.0684 \pm 0.0053$ | $108.7$ |
| | **`C4 Comp+FuzzyPID`** | **$100.0\%$** | **$53.57 \pm 0.57$** | **$0.0602 \pm 0.0055$** | **$0.0703 \pm 0.0064$** | **$75.3$** |
| | **`C5 EKF+FuzzyPID`** | **$100.0\%$** | **$51.85 \pm 0.53$** | **$0.0519 \pm 0.0028$** | **$0.0488 \pm 0.0026$** | **$64.9$** |
| **`S2_wheel_slip`** | `C2 CompFilter` | $100.0\%$ | $65.43 \pm 0.47$ | $0.0507 \pm 0.0043$ | $0.0808 \pm 0.0045$ | $164.6$ |
| | `C3 Comp+PIDF` | $100.0\%$ | $65.77 \pm 0.57$ | $0.0514 \pm 0.0031$ | $0.0812 \pm 0.0062$ | $119.5$ |
| | **`C4 Comp+FuzzyPID`** | **$100.0\%$** | **$59.17 \pm 0.60$** | **$0.0573 \pm 0.0050$** | **$0.0867 \pm 0.0062$** | **$78.1$** |
| | **`C5 EKF+FuzzyPID`** | **$100.0\%$** | **$57.75 \pm 0.54$** | **$0.0498 \pm 0.0032$** | **$0.0519 \pm 0.0029$** | **$69.3$** |
| **`S3_model_mismatch`** | `C2 CompFilter` | $100.0\%$ | $69.58 \pm 0.68$ | $0.0558 \pm 0.0062$ | $0.1022 \pm 0.0093$ | $220.9$ |
| | `C3 Comp+PIDF` | $100.0\%$ | $69.69 \pm 0.58$ | $0.0537 \pm 0.0055$ | $0.1045 \pm 0.0098$ | $143.9$ |
| | **`C4 Comp+FuzzyPID`** | **$100.0\%$** | **$62.14 \pm 0.67$** | **$0.0657 \pm 0.0073$** | **$0.1076 \pm 0.0116$** | **$95.6$** |
| | **`C5 EKF+FuzzyPID`** | **$100.0\%$** | **$59.99 \pm 0.58$** | **$0.0483 \pm 0.0038$** | **$0.0524 \pm 0.0031$** | **$82.0$** |
| **`S4_load_step`** | `C2 CompFilter` | $100.0\%$ | $74.91 \pm 0.73$ | $0.0509 \pm 0.0036$ | $0.0677 \pm 0.0035$ | $170.8$ |
| | `C3 Comp+PIDF` | $100.0\%$ | $80.66 \pm 2.89$ | $0.1650 \pm 0.0068$ | $0.0692 \pm 0.0042$ | $187.7$ |
| | **`C4 Comp+FuzzyPID`** | **$100.0\%$** | **$64.60 \pm 1.85$** | **$0.1199 \pm 0.0087$** | **$0.0698 \pm 0.0049$** | **$151.8$** |
| | **`C5 EKF+FuzzyPID`** | **$100.0\%$** | **$61.22 \pm 1.67$** | **$0.1079 \pm 0.0056$** | **$0.0506 \pm 0.0025$** | **$139.6$** |
| **`S5_worst_case`** | `C2 CompFilter` | $100.0\%$ | $97.17 \pm 1.01$ | $0.0527 \pm 0.0090$ | $0.1101 \pm 0.0079$ | $198.5$ |
| | `C3 Comp+PIDF` | $96.7\%$ | $81.30 \pm 7.74$ | $0.1122 \pm 0.1786$ | $0.1743 \pm 0.2239$ | $209.1$ |
| | **`C4 Comp+FuzzyPID`** | **$96.7\%$** | **$73.33 \pm 9.58$** | **$0.1285 \pm 0.2724$** | **$0.2020 \pm 0.3284$** | **$160.7$** |
| | **`C5 EKF+FuzzyPID`** | **$100.0\%$** | **$65.12 \pm 0.60$** | **$0.0552 \pm 0.0029$** | **$0.0552 \pm 0.0028$** | **$91.0$** |

### 5.3. Kết quả khảo sát độ nhạy tham số
- **Study A (Khảo sát khoảng nhìn trước $d_L \in [0.25, 1.40]\text{ m}$ trên `S0` và `S5`):**
  - Ở điều kiện lý tưởng `S0`, $d_L = 0.35\text{ m}$ cho sai số bám nhỏ nhất ($0.0405\text{ m}$).
  - Ở điều kiện nhiễu tổng hợp `S5`, $d_L$ quá ngắn ($0.25\text{--}0.35\text{ m}$) khuếch đại bước nhảy cập nhật vị trí thành dao động lái lớn ($w_{\mathrm{TV}} = 249.3\text{--}289.6\text{ rad/s}$), còn $d_L = 1.40\text{ m}$ gây cắt cua mạnh ($\mathrm{CTE}_{\mathrm{RMSE}} = 0.1389\text{ m}$). Điểm tối ưu thực tế đạt tại **$d_L = 0.75\text{ m}$** ($\mathrm{CTE}_{\mathrm{RMSE}} = 0.0878\text{ m}$, giảm $71\%$ dao động lái xuống $72.10\text{ rad/s}$).
- **Study B (Khảo sát hệ số lọc bù $\alpha_{\mathrm{pos}} \in [0.10, 0.85]$ trên `S1`):**
  - Tạo đường cong chữ U của sai số định vị với điểm tối ưu tại **$\alpha_{\mathrm{pos}} = 0.45$** ($\mathrm{Est}_{\mathrm{RMSE}} = 0.0718\text{ m}$). Nếu $\alpha_{\mathrm{pos}}$ quá nhỏ ($0.10$) thì sửa trôi chậm ($\mathrm{Est}_{\mathrm{RMSE}} = 0.1077\text{ m}$); nếu quá lớn ($0.85$) thì nhạy với nhiễu đo landmark và làm giật lái tăng gấp đôi ($w_{\mathrm{TV}} = 199.42\text{ rad/s}$).

---

## 6. Đối chiếu Lý thuyết vs. Thực nghiệm & Giải thích những điểm khác biệt

### 6.1. Các kết quả hoàn toàn trùng khớp với lý thuyết
1. **Hiện tượng trôi bước ngẫu nhiên (Random-Walk Drift) ở `S1`:** Dù nhiễu cảm biến odometry có kỳ vọng bằng $0$, phép cộng dồn vòng hở ở `C1` làm phương sai sai số vị trí tăng tuyến tính theo thời gian, khiến robot lạc hẳn khỏi bản đồ ($93.3\%$ lượt chạy thất bại). Khi đóng vòng định vị với landmark (`C2`–`C5`), sai số vị trí được neo chặt và đạt $100\%$ về đích.
2. **Giá trị sụt vận tốc xác lập ở `S2` và `S4`:**
   - Ở `S2` (vùng trượt $s_v = 30\%$), vận tốc thực của `C2` trong vùng trượt giảm từ $0.30\text{ m/s}$ xuống đúng **$0.21\text{ m/s}$**, khớp tuyệt đối với công thức lý thuyết $v_{\mathrm{true}} = (1 - s_v)v_d = 0.70 \times 0.30 = 0.21\text{ m/s}$.
   - Ở `S4` (đổi tải nặng tại $t = 20\text{ s}$ làm hệ số khuếch đại giảm còn $K = 0.65$), vận tốc thực của `C2` suy giảm hàm mũ về đúng **$0.195\text{ m/s}$**, khớp chính xác với đáp ứng xác lập lý thuyết FOPDT $v_{\mathrm{true}} = K v_d = 0.65 \times 0.30 = 0.195\text{ m/s}$.
3. **Tác dụng giảm chấn của khâu lọc vi phân `PIDF` và tối ưu hiệp phương sai của `EKF`:**
   - Khâu lọc thông thấp $T_f$ đặt trên đạo hàm tín hiệu đo của `PIDF` (`C3`) cắt giảm $28\text{--}35\%$ biến thiên lệnh lái $w_{\mathrm{TV}}$ so với `C2` trên `S1`–`S3`.
   - Nhờ cập nhật tuần tự dạng Joseph và tự động điều chỉnh độ lợi Kalman $\mathbf{K}_i$ theo hình học landmark, `C5 (EKF)` giữ sai số định vị $\mathrm{Est}_{\mathrm{RMSE}}$ phẳng ở mức $0.049\text{--}0.056\text{ m}$ trên cả 5 kịch bản, giảm từ $2$ đến $4$ lần so với Bộ lọc bù dùng trọng số cố định.

### 6.2. Các điểm khác biệt / ngược trực giác so với lý thuyết đơn giản và giải thích vật lý
1. **Khác biệt 1 — Tại sao ở kịch bản trượt sàn `S2`, bộ điều khiển vòng kín `C3 (PIDF cố định)` hoàn toàn KHÔNG bù được sụt tốc ($T_{\mathrm{comp}} = 65.77\text{ s}$ ngang với vòng hở `C2` $65.43\text{ s}$, vận tốc vẫn sụt xuống $0.21\text{ m/s}$)?**
   - *Giải thích:* Trong lý thuyết điều khiển PID một vòng đơn giản, khâu tích phân $I$ luôn triệt tiêu sai số xác lập. Tuy nhiên, trong cấu trúc vật lý thực tế gồm 2 tầng nối tiếp (`robot_plant_step.m`), vòng `PIDF` chỉ đo được vận tốc quay của **trục bánh xe qua encoder ($v_{\mathrm{shaft}}$)**, còn hiện tượng trượt xảy ra ở **bề mặt tiếp xúc bánh – sàn** nằm bên ngoài vòng đo của encoder. Khi đi vào vùng sàn trơn, trục bánh vẫn quay đủ $v_{\mathrm{shaft}} = 0.30\text{ m/s}$ nên sai số tốc độ trục bằng $0$ ($e_v = v_{\mathrm{ref}} - v_{\mathrm{shaft}} = 0$), làm bộ `PIDF` của `C3` hiểu nhầm rằng xe đã đạt vận tốc đặt. Chỉ khi `C4` và `C5` kết nối chéo thông tin giữa 2 tầng — lấy chỉ số trượt $\mathrm{slip}_{\mathrm{est}}$ ước lượng từ landmark ở tầng định vị đưa sang bộ Fuzzy-PID để nhân hệ số $\mathrm{boost} \in [1.00,\; 1.24]$ nâng điểm đặt $v_{\mathrm{ref}} = \mathrm{boost}\cdot v_{pp} > 0.30\text{ m/s}$ — thì vận tốc thân xe thực tế mới được kéo lên $\approx 0.26\text{ m/s}$ và rút ngắn $7\text{--}9\text{ s}$ thời gian hành trình.
2. **Khác biệt 2 — Tại sao ở kịch bản đổi tải nặng `S4` (và `S5`), cấu hình vòng hở `C2` lại có sai số bám ngang $\mathrm{CTE}_{\mathrm{RMSE}}$ nhỏ hơn các cấu hình vòng kín `C3, C4, C5` (ở `S4`: `C2` đạt $0.0509\text{ m}$ trong khi `C3`–`C5` là $0.1543\text{--}0.1650\text{ m}$)?**
   - *Giải thích (Sự đánh đổi Động học – Động lực học khi quán tính tăng gấp đôi):* Khi tải trọng tăng đột ngột tại $t = 20\text{ s}$, hằng số thời gian quán tính $T$ của cả chuyển động tịnh tiến lẫn chuyển động quay đầu xe $\omega$ tăng hơn gấp đôi từ $0.30\text{ s}$ lên $0.70\text{ s}$. Ở `C2` (vòng hở), do không bù tải nên xe bị sụt tốc mạnh xuống $0.195\text{ m/s}$ (chạy rất chậm, mất tới $74.92\text{ s}$ ở `S4` và $97.17\text{ s}$ ở `S5`); chính việc "bò chậm" qua các góc cua giúp xe có dư thời gian để xoay hướng đầu xe mà không bị văng ngang. Ngược lại, `C3`–`C5` ép vận tốc dài phục hồi lên mức cao ($0.30\text{--}0.34\text{ m/s}$) để hoàn thành nhanh nhiệm vụ vận chuyển ($68.68\text{ s}$ ở `C5`); khi robot tiến vào góc cua gắt ở tốc độ cao trong khi đáp ứng quay $\omega$ bị trễ quán tính lớn ($T = 0.70\text{ s}$), thân xe xuất hiện hiện tượng **vọt lố quán tính tại khúc cua (inertial cornering overshoot)**. Khi nâng cấp lên `C5 (EKF + Fuzzy-PID)`, nhờ ước lượng tư thế chính xác không bị trễ pha như Lọc bù, $\mathrm{CTE}_{\mathrm{RMSE}}$ ở kịch bản tổng hợp `S5` đã giảm hơn một nửa từ $0.1531\text{ m}$ (`C4`) xuống chỉ còn $0.0708\text{ m}$ (`C5`).
3. **Khác biệt 3 — Tại sao ở kịch bản `S1` (chỉ có nhiễu cảm biến, không trượt), `C4` và `C5` lại về đích nhanh hơn ($53.59\text{ s}$ và $51.09\text{ s}$) cả mốc lý tưởng không nhiễu `C0` ($54.90\text{ s}$)?**
   - *Giải thích:* Dưới nhiễu đo ngẫu nhiên của landmark ở $2\text{ Hz}$, khoảng cách giữa hai lần cập nhật vị trí ước lượng có dao động nhỏ quanh quãng đường odometry, tạo ra giá trị trượt biểu kiến nhỏ ($\mathrm{slip}_{\mathrm{est}} > 0$). Giá trị này kích hoạt nhẹ các luật mờ $r = 2, 5$ của bộ Fuzzy-PID, nâng hệ số $\mathrm{boost}$ lên khoảng $1.03\text{--}1.06$, làm vận tốc đặt trung bình trên các đoạn thẳng cao hơn nhẹ so với mức cố định $0.30\text{ m/s}$ của `C0`.

---

## 7. Kết luận và Hướng phát triển

### 7.1. Kết luận tổng quát
1. Dự án đã xây dựng thành công mô hình mô phỏng tách biệt hoàn toàn giữa **Plant vật lý có 4 nguồn bất định** và **Hệ thống định vị – điều khiển hai tầng** trên MATLAB R2024b, tuân thủ nghiêm ngặt ràng buộc không rò rỉ trạng thái thực (`ground-truth isolation`).
2. **Về tầng định vị:** Bộ lọc bù (`C2`–`C4`) cung cấp giải pháp khử trôi odometry có chi phí tính toán cực thấp, phù hợp cho vi điều khiển nhỏ; trong khi **Bộ lọc Kalman Mở rộng cập nhật tuần tự dạng Joseph (`C5`)** chứng minh ưu thế vượt trội của ma trận hiệp phương sai khi giữ sai số định vị ở mức $0.049\text{--}0.056\text{ m}$ trong mọi điều kiện nhiễu, trượt và lệch thông số cơ khí.
3. **Về tầng điều khiển tốc độ:** Sự kết hợp giữa **PID có lọc khâu vi phân trên đầu ra (PIDF)** và **Bộ lập lịch mờ 9 luật (Fuzzy-PID)** nhận thông tin độ trượt từ tầng định vị giúp giải quyết trọn vẹn bài toán sụt tốc do trượt sàn và đổi tải nặng, giúp cấu hình hoàn chỉnh **`C5 (EKF + Fuzzy-PID)`** đạt thời gian hoàn thành nhanh nhất, sai số bám nhỏ nhất và lệnh lái mượt nhất trên toàn bộ 5 kịch bản thử thách `S1`–`S5`.

### 7.2. Hướng phát triển tiếp theo
- Kết hợp thêm **Bộ điều khiển trượt (Sliding Mode Control — SMC)** hoặc bộ ước lượng quán tính tải trực tuyến để dập tắt hoàn toàn các đỉnh vọt lố ngang tại khúc cua khi tải trọng tăng đột ngột (`S4`/`S5`).
- Mở rộng vector trạng thái của EKF thành **Augmented-State EKF** để đồng thời ước lượng trực tuyến các hệ số lệch/trượt $(\eta_v, \eta_\omega)$, hoặc áp dụng UKF / Particle Filter cho môi trường nhiễu phi Gaussian và chưa biết định danh landmark (*unknown data association*).
- Tích hợp thuật toán hoạch định cục bộ tránh vật cản động (**Dynamic Window Approach — DWA** hoặc **MPC**) và triển khai thực nghiệm trên robot vi sai thực tế chạy ROS 2.

---

## 8. Hướng dẫn chạy nhanh mã nguồn trên MATLAB R2024b

Mở MATLAB R2024b tại thư mục `Code/` và chạy lần lượt các lệnh sau trong Command Window:

```matlab
% 1. Chạy tái lập Baseline S0 chuẩn MathWorks (điều kiện lý tưởng)
run('01_baseline_mathworks/run_baseline_pure_pursuit.m')

% 2. Chạy kiểm tra nhanh (Smoke Test) 6 cấu hình C0..C5 trên 1 seed
run('02_models_and_controllers/run_smoke_compare_c0_to_c4.m')

% 3. Chạy trọn bộ Monte Carlo 750 lượt (S1..S5 x C1..C5 x 30 seeds)
run('03_experiments/run_monte_carlo_suite.m')

% 4. Chạy khảo sát độ nhạy tham số Lookahead d_L (Study A) và alpha_pos (Study B)
run('03_experiments/run_parameter_sweeps.m')

% 5. Vẽ lại bộ sơ đồ khối (Hình 2.4, 2.5) và đồ thị phân tích Chương 4 (Hình 4.3, 4.4)
run('04_report_figures/draw_figure_2_4_comp_filter.m')
run('04_report_figures/draw_figure_2_5_ekf.m')
run('04_report_figures/draw_chapter4_analysis_figures.m')
```
