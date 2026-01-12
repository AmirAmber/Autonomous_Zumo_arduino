% Clear workspace and close previous figures
clear; clc; close all;

% =========================================================================
% 1. CONFIGURATION
% =========================================================================
filename = 'zumo_log.txt';   % Name of your log file
dt = 0.01;                   % Time step (10ms)

% Define distinct colors for up to 5 stages
% Format: [Red, Green, Blue]
colors = [
    0 0.4470 0.7410;      % Stage 1: Blue
    0.8500 0.3250 0.0980; % Stage 2: Orange
    0.9290 0.6940 0.1250; % Stage 3: Yellow
    0.4940 0.1840 0.5560; % Stage 4: Purple
    0.4660 0.6740 0.1880; % Stage 5: Green
];

% =========================================================================
% 2. DATA PARSING
% =========================================================================
fid = fopen(filename, 'r');
if fid == -1, error('File not found. Check filename.'); end

t_vec = []; 
x_vec = []; y_vec = []; 
vl_vec = []; vr_vec = []; 
err_vec = []; stage_vec = [];

currentStage = 1; 
currentTime = 0;

while ~feof(fid)
    tline = fgetl(fid);
    tline = strtrim(tline);
    
    if isempty(tline), continue; end
    
    % Check for STAGE Marker
    if startsWith(tline, 'STAGE', 'IgnoreCase', true)
        nums = regexp(tline, '\d+', 'match');
        if ~isempty(nums)
            currentStage = str2double(nums{1});
        end
        continue; 
    end
    
    % Parse Data: "posx, posy, batt, v_left, v_right, error"
    parts = str2double(strsplit(tline, ','));
    
    if length(parts) == 6 && all(~isnan(parts))
        currentTime = currentTime + dt;
        
        t_vec = [t_vec; currentTime];
        x_vec = [x_vec; parts(1)];
        y_vec = [y_vec; parts(2)];
        % parts(3) is battery
        vl_vec = [vl_vec; parts(4)];
        vr_vec = [vr_vec; parts(5)];
        err_vec = [err_vec; parts(6)];
        stage_vec = [stage_vec; currentStage];
    end
end
fclose(fid);

if isempty(t_vec), error('No valid data parsed.'); end

% =========================================================================
% 3. PLOTTING
% =========================================================================

% --- FIGURE 1: Track (X vs Y) ---
figure(1);
hold on; grid on; axis equal;
title('Robot Path (X vs Y)');
xlabel('Pos X (m)'); ylabel('Pos Y (m)');

plot_colored_line(x_vec, y_vec, stage_vec, colors); 
plot(x_vec(1), y_vec(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(x_vec(end), y_vec(end), 'rs', 'MarkerSize', 10, 'LineWidth', 2);
legend({'Path', 'Start', 'End'}, 'Location', 'best');


% --- FIGURE 2: Error over Time ---
figure(2);
hold on; grid on;
title('Line Error (PID Input)');
xlabel('Time (s)'); ylabel('Error Value');
yline(0, '--k'); 

plot_colored_line(t_vec, err_vec, stage_vec, colors);


% --- FIGURE 3: Velocities (Subplots) ---
figure(3);

% Subplot 1: Left Motor
subplot(2,1,1);
hold on; grid on;
title('Left Motor Velocity');
ylabel('Velocity (m/s)');
plot_colored_line(t_vec, vl_vec, stage_vec, colors);

% Subplot 2: Right Motor
subplot(2,1,2);
hold on; grid on;
title('Right Motor Velocity');
xlabel('Time (s)'); ylabel('Velocity (m/s)');
plot_colored_line(t_vec, vr_vec, stage_vec, colors);


% =========================================================================
% HELPER FUNCTION
% =========================================================================
function plot_colored_line(x, y, s, cmap)
    % Plots line segments with specific color
    for i = 1:length(x)-1
        idx = s(i);
        if idx > size(cmap,1), idx = size(cmap,1); end
        col = cmap(idx, :); 
        
        plot([x(i) x(i+1)], [y(i) y(i+1)], ...
             'Color', col, 'LineWidth', 1.5);
    end
end
