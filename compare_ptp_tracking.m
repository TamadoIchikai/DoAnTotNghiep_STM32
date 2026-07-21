% compare_ptp_tracking.m
clear; close all; clc;

% ==============================
% CONFIGURATION
% ==============================
disp('--- PTP Tracking Performance Comparison ---');
disp('You can find the exported CSVs inside STM32/Out/IK/');
file_open = input('Enter Open-Loop PTP CSV file path (e.g., STM32/Out/IK/PTP_OUT_1.csv): ', 's');
file_closed = input('Enter Closed-Loop PTP CSV file path (e.g., STM32/Out/IK/PTP_OUT_2.csv): ', 's');

if isempty(file_open) || isempty(file_closed)
    disp('File paths cannot be empty. Exiting.');
    return;
end

if ~exist(file_open, 'file')
    disp(['File not found: ', file_open]);
    return;
end
if ~exist(file_closed, 'file')
    disp(['File not found: ', file_closed]);
    return;
end

% ==============================
% DATA LOADING
% ==============================
opts = detectImportOptions(file_open);
data_open = readtable(file_open, opts);

opts = detectImportOptions(file_closed);
data_closed = readtable(file_closed, opts);

% Open Loop Data
t_open = data_open.Time_ms / 1000.0;
x_D_open = data_open.x_D;
y_D_open = data_open.y_D;
z_D_open = data_open.z_D;
angle_D_open = data_open.angle_D;

x_out_open = data_open.x_Out;
y_out_open = data_open.y_Out;
z_out_open = data_open.z_Out;
angle_out_open = data_open.angle_Out;

err_x_open = x_D_open - x_out_open;
err_y_open = y_D_open - y_out_open;
err_z_open = z_D_open - z_out_open;
err_angle_open = angle_D_open - angle_out_open;

% Closed Loop Data
t_closed = data_closed.Time_ms / 1000.0;
x_D_closed = data_closed.x_D;
y_D_closed = data_closed.y_D;
z_D_closed = data_closed.z_D;
angle_D_closed = data_closed.angle_D;

x_out_closed = data_closed.x_Out;
y_out_closed = data_closed.y_Out;
z_out_closed = data_closed.z_Out;
angle_out_closed = data_closed.angle_Out;

err_x_closed = x_D_closed - x_out_closed;
err_y_closed = y_D_closed - y_out_closed;
err_z_closed = z_D_closed - z_out_closed;
err_angle_closed = angle_D_closed - angle_out_closed;

% ==============================
% PLOTTING COMPARISON
% ==============================
figure('Name', 'PTP Tracking Comparison: Open vs Closed Loop', 'Color', 'w', 'Position', [100, 100, 1200, 800]);

% 1. X Error Comparison
subplot(4, 2, 1);
plot(t_open, err_x_open, 'r-', 'LineWidth', 1.5); hold on;
plot(t_closed, err_x_closed, 'b-', 'LineWidth', 1.5);
title('X Error Comparison'); xlabel('Time (s)'); ylabel('Error X (m)');
legend('Open-Loop', 'Closed-Loop (PD)', 'Location', 'best'); grid on;

% 2. X Tracking Comparison
subplot(4, 2, 2);
plot(t_open, x_D_open, 'k--', 'LineWidth', 1.5); hold on;
plot(t_open, x_out_open, 'r-', 'LineWidth', 1.5);
plot(t_closed, x_out_closed, 'b-', 'LineWidth', 1.5);
title('X Tracking Comparison'); xlabel('Time (s)'); ylabel('X (m)');
legend('Target', 'Open-Loop', 'Closed-Loop (PD)', 'Location', 'best'); grid on;

% 3. Y Error Comparison
subplot(4, 2, 3);
plot(t_open, err_y_open, 'r-', 'LineWidth', 1.5); hold on;
plot(t_closed, err_y_closed, 'b-', 'LineWidth', 1.5);
title('Y Error Comparison'); xlabel('Time (s)'); ylabel('Error Y (m)');
legend('Open-Loop', 'Closed-Loop (PD)', 'Location', 'best'); grid on;

% 4. Y Tracking Comparison
subplot(4, 2, 4);
plot(t_open, y_D_open, 'k--', 'LineWidth', 1.5); hold on;
plot(t_open, y_out_open, 'r-', 'LineWidth', 1.5);
plot(t_closed, y_out_closed, 'b-', 'LineWidth', 1.5);
title('Y Tracking Comparison'); xlabel('Time (s)'); ylabel('Y (m)');
legend('Target', 'Open-Loop', 'Closed-Loop (PD)', 'Location', 'best'); grid on;

% 5. Z Error Comparison
subplot(4, 2, 5);
plot(t_open, err_z_open, 'r-', 'LineWidth', 1.5); hold on;
plot(t_closed, err_z_closed, 'b-', 'LineWidth', 1.5);
title('Z Error Comparison'); xlabel('Time (s)'); ylabel('Error Z (m)');
legend('Open-Loop', 'Closed-Loop (PD)', 'Location', 'best'); grid on;

% 6. Z Tracking Comparison
subplot(4, 2, 6);
plot(t_open, z_D_open, 'k--', 'LineWidth', 1.5); hold on;
plot(t_open, z_out_open, 'r-', 'LineWidth', 1.5);
plot(t_closed, z_out_closed, 'b-', 'LineWidth', 1.5);
title('Z Tracking Comparison'); xlabel('Time (s)'); ylabel('Z (m)');
legend('Target', 'Open-Loop', 'Closed-Loop (PD)', 'Location', 'best'); grid on;

% 7. Angle Error Comparison
subplot(4, 2, 7);
plot(t_open, err_angle_open, 'r-', 'LineWidth', 1.5); hold on;
plot(t_closed, err_angle_closed, 'b-', 'LineWidth', 1.5);
title('Angle Error Comparison'); xlabel('Time (s)'); ylabel('Error Angle (deg)');
legend('Open-Loop', 'Closed-Loop (PD)', 'Location', 'best'); grid on;

% 8. Angle Tracking Comparison
subplot(4, 2, 8);
plot(t_open, angle_D_open, 'k--', 'LineWidth', 1.5); hold on;
plot(t_open, angle_out_open, 'r-', 'LineWidth', 1.5);
plot(t_closed, angle_out_closed, 'b-', 'LineWidth', 1.5);
title('Angle Tracking Comparison'); xlabel('Time (s)'); ylabel('Angle (deg)');
legend('Target', 'Open-Loop', 'Closed-Loop (PD)', 'Location', 'best'); grid on;
