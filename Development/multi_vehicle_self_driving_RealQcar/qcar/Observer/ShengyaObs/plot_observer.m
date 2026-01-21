% filepath: j:\QCar2_Cran\Development\multi_vehicle_self_driving_RealQcar\qcar\Observer\ShengyaObs\plot_distributed_luenberger.m
% Distributed Luenberger Estimator Data Plotter for MATLAB
%
% Usage:
%   plot_distributed_luenberger()              % Interactive mode
%   plot_distributed_luenberger('fake')        % Latest fake vehicle recording
%   plot_distributed_luenberger('real')        % Latest real vehicle recording
%   plot_distributed_luenberger('filepath')    % Specific file

function plot_observer(varargin)
    % Parse input arguments
    if nargin == 0
        mode = 'interactive';
        filepath = '';
    elseif strcmpi(varargin{1}, 'fake')
        mode = 'fake';
        filepath = '';
    elseif strcmpi(varargin{1}, 'real')
        mode = 'real';
        filepath = '';
    else
        mode = 'file';
        filepath = varargin{1};
    end
    
    % Recording directory paths
    recording_dirs = struct();
    recording_dirs.fake = {
        fullfile('..', '..', 'GUI', 'observer_recordings')
        fullfile('GUI', 'observer_recordings')
        'observer_recordings'
    };
    recording_dirs.real = {
        fullfile('..', '..', 'real_recordings')
        'real_recordings'
        fullfile('..', 'observer_recordings')
    };
    
    % Find file to plot
    if strcmp(mode, 'file')
        if ~exist(filepath, 'file')
            error('File not found: %s', filepath);
        end
    elseif strcmp(mode, 'interactive')
        search_dirs = [recording_dirs.fake; recording_dirs.real];
        filepath = interactive_select(search_dirs, 'auto');
        if isempty(filepath)
            fprintf('Cancelled.\n');
            return;
        end
    else
        if strcmp(mode, 'fake')
            search_dirs = recording_dirs.fake;
        else
            search_dirs = recording_dirs.real;
        end
        filepath = find_latest_recording(search_dirs);
        if isempty(filepath)
            error('No recordings found for %s', mode);
        end
        fprintf('Using latest: %s\n', filepath);
    end
    
    % Load and plot data
    try
        data = readtable(filepath);
        fprintf('Loaded %d samples\n', height(data));
    catch ME
        error('Error loading file: %s', ME.message);
    end
    
    [~, basename, ~] = fileparts(filepath);
    plot_title = sprintf('Distributed Luenberger Observer - %s', basename);
    
    create_full_plot(data, plot_title);
end

function filepath = find_latest_recording(directories)
    % Find the most recent recording file
    filepath = '';
    latest_time = 0;
    
    for i = 1:length(directories)
        dir_path = directories{i};
        if exist(dir_path, 'dir')
            files = dir(fullfile(dir_path, 'dist_luenberger_*.csv'));
            for j = 1:length(files)
                file_path = fullfile(files(j).folder, files(j).name);
                file_info = dir(file_path);
                if file_info.datenum > latest_time
                    latest_time = file_info.datenum;
                    filepath = file_path;
                end
            end
        end
    end
end

function recordings = list_recordings(directories)
    % List all recordings with metadata
    recordings = {};
    
    for i = 1:length(directories)
        dir_path = directories{i};
        if exist(dir_path, 'dir')
            files = dir(fullfile(dir_path, 'dist_luenberger_*.csv'));
            for j = 1:length(files)
                file_path = fullfile(files(j).folder, files(j).name);
                file_info = dir(file_path);
                recordings{end+1} = struct(...
                    'path', file_path, ...
                    'mtime', file_info.datenum, ...
                    'size', file_info.bytes);
            end
        end
    end
    
    % Sort by modification time (descending)
    if ~isempty(recordings)
        [~, idx] = sort(cellfun(@(x) x.mtime, recordings), 'descend');
        recordings = recordings(idx);
    end
end

function filepath = interactive_select(directories, source_name)
    % Interactive console menu to select a recording
    recordings = list_recordings(directories);
    
    if isempty(recordings)
        fprintf('\n  No %s recordings found.\n', source_name);
        filepath = '';
        return;
    end
    
    fprintf('\n%s\n', repmat('=', 1, 60));
    fprintf('  📊 %s RECORDINGS\n', upper(source_name));
    fprintf('%s\n\n', repmat('=', 1, 60));
    
    % Display recordings
    for i = 1:min(length(recordings), 20)
        rec = recordings{i};
        mtime_str = datestr(rec.mtime, 'yyyy-mm-dd HH:MM');
        size_str = format_file_size(rec.size);
        
        % Try to get duration
        try
            data = readtable(rec.path);
            duration = max(data.time) - min(data.time);
            samples = height(data);
            dur_str = format_duration(duration);
        catch
            dur_str = '?';
            samples = 0;
        end
        
        fprintf('  [%2d] %s | %8s | %5d samples | %8s\n', ...
            i, mtime_str, dur_str, samples, size_str);
    end
    
    fprintf('\n  [0] Cancel\n');
    fprintf('%s\n', repmat('=', 1, 60));
    
    % Get user input
    while true
        choice = input('\n  Enter number to plot (or 0 to cancel): ', 's');
        if strcmp(choice, '0') || strcmpi(choice, 'q')
            filepath = '';
            return;
        end
        
        choice_num = str2double(choice);
        if ~isnan(choice_num) && choice_num >= 1 && choice_num <= length(recordings)
            filepath = recordings{choice_num}.path;
            return;
        end
        
        fprintf('  Invalid choice. Enter 1-%d\n', length(recordings));
    end
end

function str = format_duration(seconds)
    % Format duration in human-readable format
    if seconds < 60
        str = sprintf('%.1fs', seconds);
    elseif seconds < 3600
        mins = floor(seconds / 60);
        secs = mod(seconds, 60);
        str = sprintf('%dm %.0fs', mins, secs);
    else
        hours = floor(seconds / 3600);
        mins = floor(mod(seconds, 3600) / 60);
        str = sprintf('%dh %dm', hours, mins);
    end
end

function str = format_file_size(bytes)
    % Format file size in human-readable format
    if bytes < 1024
        str = sprintf('%d B', bytes);
    elseif bytes < 1024 * 1024
        str = sprintf('%.1f KB', bytes / 1024);
    else
        str = sprintf('%.2f MB', bytes / (1024*1024));
    end
end

function observer_size = detect_observer_size(data)
    % Detect observer_size from column names
    col_names = data.Properties.VariableNames;
    observer_size = sum(contains(col_names, 'x_vec_before_p'));
end

function fleet_size = detect_fleet_size(data)
    % Detect fleet_size from column names
    col_names = data.Properties.VariableNames;
    fleet_size = sum(startsWith(col_names, 'fleet_x_'));
end

function create_full_plot(data, plot_title)
    % Create comprehensive multi-panel plot
    observer_size = detect_observer_size(data);
    fleet_size = detect_fleet_size(data);
    
    % Calculate stats
    duration = max(data.time) - min(data.time);
    samples = height(data);
    
    % Create figure
    fig = figure('Position', [100, 100, 1400, 1000]);
    
    % Title with stats
    stats_str = sprintf('Duration: %s | Samples: %d | Fleet: %d vehicles', ...
        format_duration(duration), samples, fleet_size);
    sgtitle({plot_title, stats_str}, 'FontSize', 10, 'FontWeight', 'bold');
    
    % Create subplots (4x2 grid)
    % Row 1: Observer states - Position and Velocity
    ax1 = subplot(4, 2, 1);
    ax2 = subplot(4, 2, 2);
    
    % Row 2: Observer states - Acceleration and Observer terms
    ax3 = subplot(4, 2, 3);
    ax4 = subplot(4, 2, 4);
    
    % Row 3: Measurement error and Consensus info
    ax5 = subplot(4, 2, 5);
    ax6 = subplot(4, 2, 6);
    
    % Row 4: Fleet states
    ax7 = subplot(4, 2, 7);
    ax8 = subplot(4, 2, 8);
    
    % Plot observer states
    plot_observer_states(data, observer_size, ax1, ax2, ax3);
    
    % Plot observer terms
    plot_observer_terms(data, observer_size, ax4, 1);
    
    % Plot measurement error and consensus
    plot_measurement_error(data, ax5);
    plot_consensus_info(data, ax6);
    
    % Plot fleet states
    plot_fleet_states(data, fleet_size, ax7, ax8);
    
    xlabel(ax7, 'Time [s]');
    xlabel(ax8, 'Time [s]');
end

function plot_observer_states(data, observer_size, ax_pos, ax_vel, ax_acc)
    % Plot relative position, velocity, and acceleration estimates
    time = data.time;
    colors = lines(observer_size);
    
    % Position
    axes(ax_pos);
    hold on;
    for i = 1:observer_size
        vid = i;
        col_name = sprintf('x_vec_after_p%d', vid);
        if ismember(col_name, data.Properties.VariableNames)
            plot(time, data.(col_name), 'DisplayName', sprintf('V%d', vid), ...
                'Color', colors(i,:), 'LineWidth', 1.2);
        end
    end
    ylabel('Pos [m]');
    title('Relative Position (pi - p0 + di0)');
    legend('Location', 'northeast', 'NumColumns', 2);
    grid on;
    
    % Velocity
    axes(ax_vel);
    hold on;
    for i = 1:observer_size
        vid = i;
        col_name = sprintf('x_vec_after_v%d', vid);
        if ismember(col_name, data.Properties.VariableNames)
            plot(time, data.(col_name), 'DisplayName', sprintf('V%d', vid), ...
                'Color', colors(i,:), 'LineWidth', 1.2);
        end
    end
    ylabel('Vel [m/s]');
    title('Relative Velocity (vi - v0)');
    legend('Location', 'northeast', 'NumColumns', 2);
    grid on;
    
    % Acceleration
    axes(ax_acc);
    hold on;
    for i = 1:observer_size
        vid = i;
        col_name = sprintf('x_vec_after_a%d', vid);
        if ismember(col_name, data.Properties.VariableNames)
            plot(time, data.(col_name), 'DisplayName', sprintf('V%d', vid), ...
                'Color', colors(i,:), 'LineWidth', 1.2);
        end
    end
    ylabel('Acc [m/s²]');
    title('Relative Acceleration (ai - a0)');
    legend('Location', 'northeast', 'NumColumns', 2);
    grid on;
end

function plot_observer_terms(data, observer_size, ax, vehicle_idx)
    % Plot dynamics, measurement, and consensus terms
    axes(ax);
    hold on;
    
    time = data.time;
    col_dyn = sprintf('dynamics_p%d', vehicle_idx);
    col_meas = sprintf('measurement_p%d', vehicle_idx);
    col_cons = sprintf('consensus_p%d', vehicle_idx);
    
    if ismember(col_dyn, data.Properties.VariableNames)
        plot(time, data.(col_dyn), 'DisplayName', 'Dynamics', 'LineWidth', 1.2);
    end
    if ismember(col_meas, data.Properties.VariableNames)
        plot(time, data.(col_meas), 'DisplayName', 'Measurement', 'LineWidth', 1.2);
    end
    if ismember(col_cons, data.Properties.VariableNames)
        plot(time, data.(col_cons), 'DisplayName', 'Consensus', 'LineWidth', 1.2);
    end
    
    ylabel('Value');
    title(sprintf('Observer Terms (Vehicle %d)', vehicle_idx));
    legend('Location', 'northeast', 'NumColumns', 3);
    grid on;
    yline(0, 'k--', 'LineWidth', 0.5, 'Alpha', 0.5);
end

function plot_measurement_error(data, ax)
    % Plot measurement error over time
    axes(ax);
    hold on;
    
    time = data.time;
    
    if ismember('meas_err_rel_pos', data.Properties.VariableNames)
        plot(time, data.meas_err_rel_pos, 'DisplayName', 'Pos Error', 'LineWidth', 1.2);
    end
    if ismember('meas_err_vel', data.Properties.VariableNames)
        plot(time, data.meas_err_vel, 'DisplayName', 'Vel Error', 'LineWidth', 1.2);
    end
    
    ylabel('Error');
    title('Measurement Error');
    legend('Location', 'northeast', 'NumColumns', 2);
    grid on;
    yline(0, 'k--', 'LineWidth', 0.5, 'Alpha', 0.5);
end

function plot_fleet_states(data, fleet_size, ax_pos, ax_vel)
    % Plot absolute fleet positions and velocities
    time = data.time;
    colors = lines(fleet_size);
    
    % Positions
    axes(ax_pos);
    hold on;
    for vid = 0:fleet_size-1
        col_name = sprintf('fleet_x_%d', vid);
        if ismember(col_name, data.Properties.VariableNames)
            if vid == 0
                label = 'Leader';
                ls = '--';
                lw = 1.5;
            else
                label = sprintf('F%d', vid);
                ls = '-';
                lw = 1.2;
            end
            plot(time, data.(col_name), 'DisplayName', label, ...
                'Color', colors(vid+1,:), 'LineWidth', lw, 'LineStyle', ls);
        end
    end
    ylabel('X [m]');
    title('Fleet Positions');
    legend('Location', 'northeast', 'NumColumns', 2);
    grid on;
    
    % Velocities
    axes(ax_vel);
    hold on;
    for vid = 0:fleet_size-1
        col_name = sprintf('fleet_v_%d', vid);
        if ismember(col_name, data.Properties.VariableNames)
            if vid == 0
                label = 'Leader';
                ls = '--';
                lw = 1.5;
            else
                label = sprintf('F%d', vid);
                ls = '-';
                lw = 1.2;
            end
            plot(time, data.(col_name), 'DisplayName', label, ...
                'Color', colors(vid+1,:), 'LineWidth', lw, 'LineStyle', ls);
        end
    end
    ylabel('V [m/s]');
    title('Fleet Velocities');
    legend('Location', 'northeast', 'NumColumns', 2);
    grid on;
end

function plot_consensus_info(data, ax)
    % Plot consensus information
    axes(ax);
    
    time = data.time;
    yyaxis left;
    if ismember('neighbor_count', data.Properties.VariableNames)
        plot(time, data.neighbor_count, 'DisplayName', 'Neighbors', 'LineWidth', 1.2);
    end
    ylabel('Count');
    
    yyaxis right;
    if ismember('consensus_norm', data.Properties.VariableNames)
        plot(time, data.consensus_norm, 'DisplayName', 'Cons. Norm', 'LineWidth', 1.2);
    end
    ylabel('Norm');
    
    title('Consensus Info');
    grid on;
    legend('Location', 'northeast', 'NumColumns', 2);
end