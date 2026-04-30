%% Allan Variance 분석 — ISM330DHCX (6축 전부)
clear; close all; clc;

%% 1. 데이터 로드
fprintf('Loading CSV...\n');
data = readtable('imu_1h.csv');
N = height(data);
fprintf('  Loaded %d samples\n', N);

%% 2. 실제 fs 자동 계산
last_t_us = data.t_us(end);
duration_sec = double(last_t_us) / 1e6;
fs = N / duration_sec;
fprintf('  Duration: %.2f sec\n', duration_sec);
fprintf('  Actual fs: %.3f Hz\n\n', fs);

%% 3. 단위 변환 (6축)
ax = data.ax_g * 9.80665;        % g → m/s²
ay = data.ay_g * 9.80665;
az = data.az_g * 9.80665;
gx = data.gx_dps * pi/180;       % deg/s → rad/s
gy = data.gy_dps * pi/180;
gz = data.gz_dps * pi/180;

%% 4. Sanity check
fprintf('========== Sanity Check ==========\n');
fprintf('Accel (m/s²):\n');
fprintf('  ax: mean=%+.4f std=%.5f\n', mean(ax), std(ax));
fprintf('  ay: mean=%+.4f std=%.5f\n', mean(ay), std(ay));
fprintf('  az: mean=%+.4f std=%.5f (1g≈9.81)\n', mean(az), std(az));
fprintf('Gyro (rad/s):\n');
fprintf('  gx: mean=%+.5f std=%.5f\n', mean(gx), std(gx));
fprintf('  gy: mean=%+.5f std=%.5f\n', mean(gy), std(gy));
fprintf('  gz: mean=%+.5f std=%.5f\n', mean(gz), std(gz));
fprintf('====================================\n\n');

%% 5. Allan Variance 계산 (6축 모두)
fprintf('Computing Allan variance for 6 axes (3-5 min)...\n');

[avar, tau] = allanvar(ax, 'octave', fs); adev_ax = sqrt(avar); tau_ax = tau;
fprintf('  ax done\n');
[avar, tau] = allanvar(ay, 'octave', fs); adev_ay = sqrt(avar); tau_ay = tau;
fprintf('  ay done\n');
[avar, tau] = allanvar(az, 'octave', fs); adev_az = sqrt(avar); tau_az = tau;
fprintf('  az done\n');
[avar, tau] = allanvar(gx, 'octave', fs); adev_gx = sqrt(avar); tau_gx = tau;
fprintf('  gx done\n');
[avar, tau] = allanvar(gy, 'octave', fs); adev_gy = sqrt(avar); tau_gy = tau;
fprintf('  gy done\n');
[avar, tau] = allanvar(gz, 'octave', fs); adev_gz = sqrt(avar); tau_gz = tau;
fprintf('  gz done\n\n');

%% 6. 그래프 (2x3 subplot)
figure('Position', [50 50 1500 800], 'Name', 'Allan Deviation - 6 axes');

% 가속도 3축
subplot(2, 3, 1);
loglog(tau_ax, adev_ax, 'b-', 'LineWidth', 1.5); hold on; grid on;
loglog(tau_ax, adev_ax(1)./sqrt(tau_ax/tau_ax(1)), 'k--', 'LineWidth', 0.7);
loglog(tau_ax, adev_ax(end).*sqrt(tau_ax/tau_ax(end)), 'k:', 'LineWidth', 0.7);
xlabel('\tau (s)'); ylabel('\sigma_a (m/s²)');
title('Accel X'); legend('실측','-1/2 (white)','+1/2 (rw)','Location','best');

subplot(2, 3, 2);
loglog(tau_ay, adev_ay, 'b-', 'LineWidth', 1.5); hold on; grid on;
loglog(tau_ay, adev_ay(1)./sqrt(tau_ay/tau_ay(1)), 'k--', 'LineWidth', 0.7);
loglog(tau_ay, adev_ay(end).*sqrt(tau_ay/tau_ay(end)), 'k:', 'LineWidth', 0.7);
xlabel('\tau (s)'); ylabel('\sigma_a (m/s²)');
title('Accel Y');

subplot(2, 3, 3);
loglog(tau_az, adev_az, 'b-', 'LineWidth', 1.5); hold on; grid on;
loglog(tau_az, adev_az(1)./sqrt(tau_az/tau_az(1)), 'k--', 'LineWidth', 0.7);
loglog(tau_az, adev_az(end).*sqrt(tau_az/tau_az(end)), 'k:', 'LineWidth', 0.7);
xlabel('\tau (s)'); ylabel('\sigma_a (m/s²)');
title('Accel Z');

% 자이로 3축
subplot(2, 3, 4);
loglog(tau_gx, adev_gx, 'r-', 'LineWidth', 1.5); hold on; grid on;
loglog(tau_gx, adev_gx(1)./sqrt(tau_gx/tau_gx(1)), 'k--', 'LineWidth', 0.7);
loglog(tau_gx, adev_gx(end).*sqrt(tau_gx/tau_gx(end)), 'k:', 'LineWidth', 0.7);
xlabel('\tau (s)'); ylabel('\sigma_\omega (rad/s)');
title('Gyro X');

subplot(2, 3, 5);
loglog(tau_gy, adev_gy, 'r-', 'LineWidth', 1.5); hold on; grid on;
loglog(tau_gy, adev_gy(1)./sqrt(tau_gy/tau_gy(1)), 'k--', 'LineWidth', 0.7);
loglog(tau_gy, adev_gy(end).*sqrt(tau_gy/tau_gy(end)), 'k:', 'LineWidth', 0.7);
xlabel('\tau (s)'); ylabel('\sigma_\omega (rad/s)');
title('Gyro Y');

subplot(2, 3, 6);
loglog(tau_gz, adev_gz, 'r-', 'LineWidth', 1.5); hold on; grid on;
loglog(tau_gz, adev_gz(1)./sqrt(tau_gz/tau_gz(1)), 'k--', 'LineWidth', 0.7);
loglog(tau_gz, adev_gz(end).*sqrt(tau_gz/tau_gz(end)), 'k:', 'LineWidth', 0.7);
xlabel('\tau (s)'); ylabel('\sigma_\omega (rad/s)');
title('Gyro Z');

sgtitle(sprintf('ISM330DHCX Allan Deviation (6-axis, fs=%.1fHz, T=%.0fs)', fs, duration_sec));
saveas(gcf, 'allan_variance_6axis.png');
fprintf('Graph saved: allan_variance_6axis.png\n\n');

%% 7. 핵심 수치 — 6축 모두
[~, idx_a] = min(abs(tau_ax - 1));
[~, idx_g] = min(abs(tau_gz - 1));

VRW_x = adev_ax(idx_a);  VRW_y = adev_ay(idx_a);  VRW_z = adev_az(idx_a);
ARW_x = adev_gx(idx_g) * 180/pi * 60;
ARW_y = adev_gy(idx_g) * 180/pi * 60;
ARW_z = adev_gz(idx_g) * 180/pi * 60;

[bi_ax, ix_ax] = min(adev_ax);  [bi_ay, ix_ay] = min(adev_ay);  [bi_az, ix_az] = min(adev_az);
[bi_gx, ix_gx] = min(adev_gx);  [bi_gy, ix_gy] = min(adev_gy);  [bi_gz, ix_gz] = min(adev_gz);

fprintf('========== 핵심 수치 (6축) ==========\n');
fprintf('가속도 (VRW @ τ=1s, BI = min):\n');
fprintf('  ax: VRW=%.4e m/s²/√Hz, BI=%.4e m/s² @ τ=%.1fs\n', VRW_x, bi_ax, tau_ax(ix_ax));
fprintf('  ay: VRW=%.4e m/s²/√Hz, BI=%.4e m/s² @ τ=%.1fs\n', VRW_y, bi_ay, tau_ay(ix_ay));
fprintf('  az: VRW=%.4e m/s²/√Hz, BI=%.4e m/s² @ τ=%.1fs\n', VRW_z, bi_az, tau_az(ix_az));
fprintf('자이로 (ARW @ τ=1s, BI = min):\n');
fprintf('  gx: ARW=%.4f deg/√h, BI=%.4e rad/s @ τ=%.1fs\n', ARW_x, bi_gx, tau_gx(ix_gx));
fprintf('  gy: ARW=%.4f deg/√h, BI=%.4e rad/s @ τ=%.1fs\n', ARW_y, bi_gy, tau_gy(ix_gy));
fprintf('  gz: ARW=%.4f deg/√h, BI=%.4e rad/s @ τ=%.1fs\n', ARW_z, bi_gz, tau_gz(ix_gz));
fprintf('=======================================\n\n');

%% 8. 데이터시트 비교
fprintf('========== 데이터시트 비교 ==========\n');
fprintf('Accel VRW 데이터시트: 5.89e-04 m/s²/√Hz (60 μg/√Hz)\n');
fprintf('  실측 ax: %.4e (%.2fx)\n', VRW_x, VRW_x/5.89e-4);
fprintf('  실측 ay: %.4e (%.2fx)\n', VRW_y, VRW_y/5.89e-4);
fprintf('  실측 az: %.4e (%.2fx)\n', VRW_z, VRW_z/5.89e-4);
fprintf('\n');
fprintf('Gyro ARW 데이터시트: 0.300 deg/√h (5 mdps/√Hz)\n');
fprintf('  실측 gx: %.4f deg/√h (%.2fx)\n', ARW_x, ARW_x/0.3);
fprintf('  실측 gy: %.4f deg/√h (%.2fx)\n', ARW_y, ARW_y/0.3);
fprintf('  실측 gz: %.4f deg/√h (%.2fx)\n', ARW_z, ARW_z/0.3);
fprintf('======================================\n\n');

fprintf('==> DynamicBias 윈도우 권장:\n');
fprintf('    가속도 ax 기준 τ ≈ %.0f초\n', tau_ax(ix_ax));
fprintf('    가속도 ay 기준 τ ≈ %.0f초\n', tau_ay(ix_ay));
fprintf('    가속도 az 기준 τ ≈ %.0f초\n', tau_az(ix_az));
fprintf('    자이로 gz 기준 τ ≈ %.0f초\n', tau_gz(ix_gz));