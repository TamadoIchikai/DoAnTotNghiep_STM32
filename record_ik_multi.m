% record_ik_multi.m
clear; close all; clc;

% ==============================
% CONFIGURATION
% ==============================
com_port = 'COM5';
baud_rate = 115200;
out_dir = fullfile('STM32', 'Out', 'IK');

if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

cmd = input('Enter IK PD test command (e.g., ik_mulrun_con_1 or ptp_run_con_1_90_0.2_0_0.05): ', 's');
if isempty(cmd)
    disp('No command entered. Exiting.');
    return;
end

is_ptp_cmd = startsWith(cmd, 'ptp_run');

% ==============================
% SERIAL CONNECTION
% ==============================
try
    s = serialport(com_port, baud_rate);
    configureTerminator(s, "CR/LF");
    
    try
        s.DataTerminalReady = "off";
        s.RequestToSend = "off";
    catch
    end
    
    disp('Waiting 2 seconds for STM32 to boot/settle...');
    pause(2); 
    
    flush(s);
    disp(['Connected to ', com_port]);
catch ME
    disp(['Failed to connect to ', com_port]);
    disp(ME.message);
    return;
end

% Send characters with delay to prevent UART overrun
disp(['Sending command: ', cmd]);
full_cmd = [cmd, 13, 10];
for i = 1:length(full_cmd)
    write(s, uint8(full_cmd(i)), "uint8");
    pause(0.005); 
end

% ==============================
% DATA COLLECTION
% ==============================
disp('Waiting for [MPD_START]...');
recording = false;

% Preallocate arrays
time_ms = [];
X = []; Y = []; Z = [];
ref1 = []; enc1 = []; vel1 = []; u1 = [];
ref2 = []; enc2 = []; vel2 = []; u2 = [];
ref3 = []; enc3 = []; vel3 = []; u3 = [];
ref4 = []; enc4 = []; vel4 = []; u4 = [];

% PTP Arrays
x_D = []; y_D = []; z_D = []; angle_D = [];
x_Out = []; y_Out = []; z_Out = []; angle_Out = [];

% Keep track of where each point's trajectory begins
segment_starts = []; 

tic;
max_wait_time = 20; 

while toc < max_wait_time
    if s.NumBytesAvailable > 0
        line_str = strtrim(readline(s)); 
        
        % Clean up any preceding prompt characters (e.g. "> ")
        line_str = regexprep(line_str, '^>\s*', '');
        
        if ~startsWith(line_str, '[MPD_DATA]') && ~startsWith(line_str, '[PTP_DATA]')
            disp(['[STM32] ', char(line_str)]);
        end
        
        if startsWith(line_str, '[MPD_START]')
            recording = true;
            if is_ptp_cmd
                disp('Recording PTP Telemetry...');
            else
                disp('Recording Multi-PD Telemetry...');
            end
            tic; % reset timeout once data starts
            
            % Save the index of the start of this segment
            segment_starts(end+1) = length(time_ms) + 1;
            
        elseif startsWith(line_str, '[MPD_DATA]') && recording && ~is_ptp_cmd
            parts = split(strrep(line_str, '[MPD_DATA] ', ''), ',');
            if length(parts) == 20
                time_ms(end+1) = str2double(parts{1});
                X(end+1) = str2double(parts{2});
                Y(end+1) = str2double(parts{3});
                Z(end+1) = str2double(parts{4});
                
                ref1(end+1) = str2double(parts{5}); enc1(end+1) = str2double(parts{6}); vel1(end+1) = str2double(parts{7}); u1(end+1) = str2double(parts{8});
                ref2(end+1) = str2double(parts{9}); enc2(end+1) = str2double(parts{10}); vel2(end+1) = str2double(parts{11}); u2(end+1) = str2double(parts{12});
                ref3(end+1) = str2double(parts{13}); enc3(end+1) = str2double(parts{14}); vel3(end+1) = str2double(parts{15}); u3(end+1) = str2double(parts{16});
                ref4(end+1) = str2double(parts{17}); enc4(end+1) = str2double(parts{18}); vel4(end+1) = str2double(parts{19}); u4(end+1) = str2double(parts{20});
            end
            
        elseif startsWith(line_str, '[PTP_DATA]') && recording && is_ptp_cmd
            parts = split(strrep(line_str, '[PTP_DATA] ', ''), ',');
            if length(parts) == 9
                time_ms(end+1) = str2double(parts{1});
                x_D(end+1) = str2double(parts{2});
                y_D(end+1) = str2double(parts{3});
                z_D(end+1) = str2double(parts{4});
                angle_D(end+1) = str2double(parts{5});
                x_Out(end+1) = str2double(parts{6});
                y_Out(end+1) = str2double(parts{7});
                z_Out(end+1) = str2double(parts{8});
                angle_Out(end+1) = str2double(parts{9});
            end
            
        elseif startsWith(line_str, '[MPD_END]')
            if ~startsWith(cmd, 'ik_mulrun')
                break;
            else
                recording = false; 
            end
            
        elseif contains(line_str, 'Sequence complete!')
            break;
        end
    end
end

clear s; 
disp('Serial port closed.');

% ==============================
% SAVING & PLOTTING
% ==============================
if isempty(time_ms)
    disp('No valid data was captured. Check the [STM32] output above!');
    return;
end

n = 1;
while true
    if is_ptp_cmd
        filename = sprintf('PTP_OUT_%d.csv', n);
    else
        filename = sprintf('IK_OUT_%d.csv', n);
    end
    filepath = fullfile(out_dir, filename);
    if ~exist(filepath, 'file')
        break;
    end
    n = n + 1;
end

if is_ptp_cmd
    T = table(time_ms', x_D', y_D', z_D', angle_D', x_Out', y_Out', z_Out', angle_Out', ...
        'VariableNames', {'Time_ms', 'x_D', 'y_D', 'z_D', 'angle_D', 'x_Out', 'y_Out', 'z_Out', 'angle_Out'});
    writetable(T, filepath);
    disp(['PTP Data saved successfully to: ', filepath]);

    % Plotting PTP Performance
    err_x = x_D - x_Out;
    err_y = y_D - y_Out;
    err_z = z_D - z_Out;
    err_angle = angle_D - angle_Out;

    figure('Name', 'PTP Tracking Performance', 'Color', 'w', 'Position', [100, 100, 1200, 800]);
    time_s = time_ms / 1000.0;

    % 1. X Tracking
    subplot(4, 2, 1);
    plot(time_s, x_D, 'r--', 'LineWidth', 1.5); hold on;
    plot(time_s, x_Out, 'b-', 'LineWidth', 1.5);
    title('X Tracking'); xlabel('Time (s)'); ylabel('X (m)');
    legend('X_{Target}', 'X_{Out}', 'Location', 'best'); grid on;

    % 2. X Error
    subplot(4, 2, 2);
    plot(time_s, err_x, 'k-', 'LineWidth', 1.5);
    title('X Error'); xlabel('Time (s)'); ylabel('Error X (m)'); grid on;

    % 3. Y Tracking
    subplot(4, 2, 3);
    plot(time_s, y_D, 'r--', 'LineWidth', 1.5); hold on;
    plot(time_s, y_Out, 'b-', 'LineWidth', 1.5);
    title('Y Tracking'); xlabel('Time (s)'); ylabel('Y (m)');
    legend('Y_{Target}', 'Y_{Out}', 'Location', 'best'); grid on;

    % 4. Y Error
    subplot(4, 2, 4);
    plot(time_s, err_y, 'k-', 'LineWidth', 1.5);
    title('Y Error'); xlabel('Time (s)'); ylabel('Error Y (m)'); grid on;

    % 5. Z Tracking
    subplot(4, 2, 5);
    plot(time_s, z_D, 'r--', 'LineWidth', 1.5); hold on;
    plot(time_s, z_Out, 'b-', 'LineWidth', 1.5);
    title('Z Tracking'); xlabel('Time (s)'); ylabel('Z (m)');
    legend('Z_{Target}', 'Z_{Out}', 'Location', 'best'); grid on;

    % 6. Z Error
    subplot(4, 2, 6);
    plot(time_s, err_z, 'k-', 'LineWidth', 1.5);
    title('Z Error'); xlabel('Time (s)'); ylabel('Error Z (m)'); grid on;

    % 7. Angle Tracking
    subplot(4, 2, 7);
    plot(time_s, angle_D, 'r--', 'LineWidth', 1.5); hold on;
    plot(time_s, angle_Out, 'b-', 'LineWidth', 1.5);
    title('Angle Tracking'); xlabel('Time (s)'); ylabel('Angle (deg)');
    legend('Angle_{Target}', 'Angle_{Out}', 'Location', 'best'); grid on;

    % 8. Angle Error
    subplot(4, 2, 8);
    plot(time_s, err_angle, 'k-', 'LineWidth', 1.5);
    title('Angle Error'); xlabel('Time (s)'); ylabel('Error Angle (deg)'); grid on;

else
    T = table(time_ms', X', Y', Z', ...
        ref1', enc1', vel1', u1', ref2', enc2', vel2', u2', ...
        ref3', enc3', vel3', u3', ref4', enc4', vel4', u4', ...
        'VariableNames', {'Time_ms', 'X', 'Y', 'Z', ...
        'Ref1', 'Enc1', 'Vel1', 'U1', 'Ref2', 'Enc2', 'Vel2', 'U2', ...
        'Ref3', 'Enc3', 'Vel3', 'U3', 'Ref4', 'Enc4', 'Vel4', 'U4'});
    writetable(T, filepath);
    disp(['Data saved successfully to: ', filepath]);

    % Generate distinct colors for each trajectory segment
    num_segments = length(segment_starts);
    colors = lines(num_segments);

    % --- UN-RESET THE TIME VECTOR ---
    continuous_time = zeros(size(time_ms));
    cumulative_time = 0;
    for i = 1:num_segments
        idx_start = segment_starts(i);
        if i < num_segments
            idx_end = segment_starts(i+1) - 1;
        else
            idx_end = length(time_ms);
        end
        
        if idx_end >= idx_start
            seg_time = time_ms(idx_start:idx_end);
            continuous_time(idx_start:idx_end) = seg_time + cumulative_time;
            cumulative_time = continuous_time(idx_end); % Next segment starts where this one left off
        end
    end

    % Plotting Motors
    figure('Name', 'Multi-Motor IK PD Trajectory', 'Color', 'w', 'Position', [100, 100, 1200, 800]);

    % Pass continuous_time instead of time_ms
    plot_motor(1, continuous_time, ref1, enc1, vel1, u1, 'Motor 1 (Base)', 'Interval (\mu s)', segment_starts, colors);
    plot_motor(4, continuous_time, ref2, enc2, vel2, u2, 'Motor 2 (Elbow)', 'Interval (\mu s)', segment_starts, colors);
    plot_motor(7, continuous_time, ref3, enc3, vel3, u3, 'Motor 3 (Z-Axis)', 'PWM (0-999)', segment_starts, colors);
    plot_motor(10, continuous_time, ref4, enc4, vel4, u4, 'Motor 4 (End Effector)', 'Interval (\mu s)', segment_starts, colors);

    % 3D Trajectory Plot
    figure('Name', 'End-Effector Trajectory', 'Color', 'w');
    hold on; grid on; view(3);
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('3D Physical Trajectory');

    leg_entries = {};
    for i = 1:num_segments
        idx_start = segment_starts(i);
        if i < num_segments
            idx_end = segment_starts(i+1) - 1;
        else
            idx_end = length(X);
        end
        
        if idx_end >= idx_start
            plot3(X(idx_start:idx_end), Y(idx_start:idx_end), Z(idx_start:idx_end), '-', 'LineWidth', 2, 'Color', colors(i,:));
            leg_entries{end+1} = sprintf('Trajectory %d', i);
        end
    end

    plot3(X(1), Y(1), Z(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); 
    leg_entries{end+1} = 'Start';
    plot3(X(end), Y(end), Z(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); 
    leg_entries{end+1} = 'Final Target';

    legend(leg_entries, 'Location', 'best');
end

% Helper function for segmented motor subplot
function plot_motor(subplot_idx, time, ref, enc, vel, u, title_str, u_label, segment_starts, colors)
    num_segments = length(segment_starts);
    leg_entries = {};
    
    % --- Position Plot ---
    subplot(4, 3, subplot_idx); hold on; grid on;
    title([title_str ' - Angle']); ylabel('Angle/Pos'); xlabel('Time (ms)');
    
    for i = 1:num_segments
        idx_start = segment_starts(i);
        idx_end = (i < num_segments) * (segment_starts(min(i+1, num_segments)) - 1) + (i == num_segments) * length(time);
        
        if idx_end >= idx_start
            % Plot current angle
            plot(time(idx_start:idx_end), enc(idx_start:idx_end), '-', 'LineWidth', 1.5, 'Color', colors(i,:));
            leg_entries{end+1} = sprintf('Traj %d', i);
            
            % Plot target angle
            plot(time(idx_start:idx_end), ref(idx_start:idx_end), '--', 'LineWidth', 1.5, 'Color', colors(i,:));
            leg_entries{end+1} = sprintf('Target %d', i);
        end
    end
    legend(leg_entries, 'Location', 'best');
    
    % --- Velocity Plot ---
    subplot(4, 3, subplot_idx + 1); hold on; grid on;
    title('Velocity'); ylabel('deg/s'); xlabel('Time (ms)');
    for i = 1:num_segments
        idx_start = segment_starts(i);
        idx_end = (i < num_segments) * (segment_starts(min(i+1, num_segments)) - 1) + (i == num_segments) * length(time);
        if idx_end >= idx_start
            plot(time(idx_start:idx_end), vel(idx_start:idx_end), '-', 'LineWidth', 1.5, 'Color', colors(i,:));
        end
    end
    
    % --- Control Signal Plot ---
    subplot(4, 3, subplot_idx + 2); hold on; grid on;
    title('Control Signal'); ylabel(u_label); xlabel('Time (ms)');
    for i = 1:num_segments
        idx_start = segment_starts(i);
        idx_end = (i < num_segments) * (segment_starts(min(i+1, num_segments)) - 1) + (i == num_segments) * length(time);
        if idx_end >= idx_start
            plot(time(idx_start:idx_end), u(idx_start:idx_end), '-', 'LineWidth', 1, 'Color', colors(i,:));
        end
    end
end
