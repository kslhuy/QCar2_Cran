%% MATLAB Plotting Script for Fleet Observer Data
% This script loads observer data from .mat files and creates comprehensive plots
% for analyzing vehicle state estimation and fleet coordination performance.

function plot_observer_data(data_file, options)
    % Plot observer data from MATLAB .mat file
    %
    % Inputs:
    %   data_file: Path to the .mat file containing observer data
    %   options: (optional) Structure with plotting options
    %     .save_figures: Whether to save figures (default: true)
    %     .show_gps: Whether to show GPS measurements (default: true)
    %     .fleet_comparison: Whether to plot fleet comparison (default: true)
    %     .figure_format: Format for saved figures ('png', 'pdf', 'eps', default: 'png')
    
    if nargin < 2
        options = struct();
    end
    
    % Default options
    if ~isfield(options, 'save_figures'), options.save_figures = true; end
    if ~isfield(options, 'show_gps'), options.show_gps = true; end
    if ~isfield(options, 'fleet_comparison'), options.fleet_comparison = true; end
    if ~isfield(options, 'figure_format'), options.figure_format = 'png'; end
    
    % Load data
    fprintf('Loading observer data from: %s\n', data_file);
    data = load(data_file);
    
    % Extract metadata
    vehicle_id = data.metadata.vehicle_id;
    fleet_size = data.metadata.fleet_size;
    total_samples = data.metadata.total_samples;
    
    fprintf('Vehicle ID: %d, Fleet Size: %d, Samples: %d\n', ...
            vehicle_id, fleet_size, total_samples);
    
    % Create output directory for figures
    [filepath, name, ~] = fileparts(data_file);
    fig_dir = fullfile(filepath, sprintf('figures_vehicle_%d', vehicle_id));
    if options.save_figures && ~exist(fig_dir, 'dir')
        mkdir(fig_dir);
    end
    
    %% Plot 1: Vehicle Trajectory
    plot_trajectory(data, vehicle_id, options, fig_dir);
    
    %% Plot 2: State Components vs Time
    plot_state_components(data, vehicle_id, options, fig_dir);
    
    %% Plot 3: Control Inputs
    plot_control_inputs(data, vehicle_id, options, fig_dir);
    
    %% Plot 4: GPS vs Estimation Comparison
    if options.show_gps && isfield(data, 'gps_measurements')
        plot_gps_comparison(data, vehicle_id, options, fig_dir);
    end
    
    %% Plot 5: Fleet Trajectories
    if options.fleet_comparison && isfield(data, 'fleet_estimates')
        plot_fleet_trajectories(data, vehicle_id, fleet_size, options, fig_dir);
    end
    
    %% Plot 6: Fleet State Evolution
    if isfield(data, 'fleet_estimates')
        plot_fleet_evolution(data, vehicle_id, fleet_size, options, fig_dir);
    end
    
    %% Plot 7: Estimation Error Analysis
    if options.show_gps && isfield(data, 'gps_measurements')
        plot_estimation_errors(data, vehicle_id, options, fig_dir);
    end
    
    fprintf('Plotting completed. Figures saved in: %s\n', fig_dir);
end

function plot_trajectory(data, vehicle_id, options, fig_dir)
    % Plot vehicle trajectory
    
    figure('Position', [100, 100, 1000, 800]);
    
    if isfield(data, 'local_states')
        % Plot estimated trajectory
        plot(data.local_states.x, data.local_states.y, 'b-', 'LineWidth', 2, ...
             'DisplayName', 'Estimated Trajectory');
        hold on;
        
        % Mark start and end points
        plot(data.local_states.x(1), data.local_states.y(1), 'go', ...
             'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
        plot(data.local_states.x(end), data.local_states.y(end), 'ro', ...
             'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'End');
    end
    
    % Plot GPS measurements if available
    if options.show_gps && isfield(data, 'gps_measurements')
        gps_valid = ~isnan(data.gps_measurements.x);
        if any(gps_valid)
            scatter(data.gps_measurements.x(gps_valid), ...
                   data.gps_measurements.y(gps_valid), ...
                   20, 'r', 'filled', 'Alpha', 0.6, ...
                   'DisplayName', 'GPS Measurements');
        end
    end
    
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title(sprintf('Vehicle %d Trajectory', vehicle_id));
    legend('Location', 'best');
    grid on;
    axis equal;
    
    if options.save_figures
        filename = sprintf('vehicle_%d_trajectory.%s', vehicle_id, options.figure_format);
        saveas(gcf, fullfile(fig_dir, filename));
    end
end

function plot_state_components(data, vehicle_id, options, fig_dir)
    % Plot state components over time
    
    if ~isfield(data, 'local_states')
        return;
    end
    
    figure('Position', [150, 150, 1200, 900]);
    
    % Position X
    subplot(2, 2, 1);
    plot(data.local_states.timestamps, data.local_states.x, 'b-', 'LineWidth', 2);
    if options.show_gps && isfield(data, 'gps_measurements')
        hold on;
        gps_valid = ~isnan(data.gps_measurements.x);
        scatter(data.local_states.timestamps(gps_valid), ...
               data.gps_measurements.x(gps_valid), 20, 'r', 'filled', 'Alpha', 0.7);
        legend('Estimated', 'GPS', 'Location', 'best');
    end
    xlabel('Time (s)');
    ylabel('X Position (m)');
    title('X Position vs Time');
    grid on;
    
    % Position Y
    subplot(2, 2, 2);
    plot(data.local_states.timestamps, data.local_states.y, 'b-', 'LineWidth', 2);
    if options.show_gps && isfield(data, 'gps_measurements')
        hold on;
        gps_valid = ~isnan(data.gps_measurements.y);
        scatter(data.local_states.timestamps(gps_valid), ...
               data.gps_measurements.y(gps_valid), 20, 'r', 'filled', 'Alpha', 0.7);
        legend('Estimated', 'GPS', 'Location', 'best');
    end
    xlabel('Time (s)');
    ylabel('Y Position (m)');
    title('Y Position vs Time');
    grid on;
    
    % Heading
    subplot(2, 2, 3);
    plot(data.local_states.timestamps, rad2deg(data.local_states.theta), 'b-', 'LineWidth', 2);
    if options.show_gps && isfield(data, 'gps_measurements')
        hold on;
        gps_valid = ~isnan(data.gps_measurements.theta);
        scatter(data.local_states.timestamps(gps_valid), ...
               rad2deg(data.gps_measurements.theta(gps_valid)), 20, 'r', 'filled', 'Alpha', 0.7);
        legend('Estimated', 'GPS', 'Location', 'best');
    end
    xlabel('Time (s)');
    ylabel('Heading (degrees)');
    title('Heading vs Time');
    grid on;
    
    % Velocity
    subplot(2, 2, 4);
    plot(data.local_states.timestamps, data.local_states.velocity, 'b-', 'LineWidth', 2);
    if options.show_gps && isfield(data, 'gps_measurements')
        hold on;
        gps_valid = ~isnan(data.gps_measurements.velocity);
        scatter(data.local_states.timestamps(gps_valid), ...
               data.gps_measurements.velocity(gps_valid), 20, 'r', 'filled', 'Alpha', 0.7);
        legend('Estimated', 'GPS', 'Location', 'best');
    end
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Velocity vs Time');
    grid on;
    
    sgtitle(sprintf('Vehicle %d State Components', vehicle_id));
    
    if options.save_figures
        filename = sprintf('vehicle_%d_state_components.%s', vehicle_id, options.figure_format);
        saveas(gcf, fullfile(fig_dir, filename));
    end
end

function plot_control_inputs(data, vehicle_id, options, fig_dir)
    % Plot control inputs over time
    
    if ~isfield(data, 'control_inputs')
        return;
    end
    
    figure('Position', [200, 200, 1200, 600]);
    
    % Steering input
    subplot(2, 1, 1);
    plot(data.local_states.timestamps, data.control_inputs.steering, 'g-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Steering Command');
    title('Steering Input vs Time');
    grid on;
    
    % Acceleration input
    subplot(2, 1, 2);
    plot(data.local_states.timestamps, data.control_inputs.acceleration, 'm-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Acceleration Command');
    title('Acceleration Input vs Time');
    grid on;
    
    sgtitle(sprintf('Vehicle %d Control Inputs', vehicle_id));
    
    if options.save_figures
        filename = sprintf('vehicle_%d_control_inputs.%s', vehicle_id, options.figure_format);
        saveas(gcf, fullfile(fig_dir, filename));
    end
end

function plot_gps_comparison(data, vehicle_id, options, fig_dir)
    % Plot detailed GPS vs estimation comparison
    
    figure('Position', [250, 250, 1400, 1000]);
    
    % Position error
    subplot(3, 2, 1);
    gps_valid = ~isnan(data.gps_measurements.x) & ~isnan(data.gps_measurements.y);
    pos_error = sqrt((data.local_states.x(gps_valid) - data.gps_measurements.x(gps_valid)).^2 + ...
                     (data.local_states.y(gps_valid) - data.gps_measurements.y(gps_valid)).^2);
    plot(data.local_states.timestamps(gps_valid), pos_error, 'r-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Position Error (m)');
    title('Position Estimation Error');
    grid on;
    
    % X position comparison
    subplot(3, 2, 2);
    plot(data.local_states.timestamps, data.local_states.x, 'b-', 'LineWidth', 2);
    hold on;
    gps_valid = ~isnan(data.gps_measurements.x);
    scatter(data.local_states.timestamps(gps_valid), ...
           data.gps_measurements.x(gps_valid), 20, 'r', 'filled', 'Alpha', 0.7);
    xlabel('Time (s)');
    ylabel('X Position (m)');
    title('X Position: Estimated vs GPS');
    legend('Estimated', 'GPS', 'Location', 'best');
    grid on;
    
    % Y position comparison
    subplot(3, 2, 3);
    plot(data.local_states.timestamps, data.local_states.y, 'b-', 'LineWidth', 2);
    hold on;
    gps_valid = ~isnan(data.gps_measurements.y);
    scatter(data.local_states.timestamps(gps_valid), ...
           data.gps_measurements.y(gps_valid), 20, 'r', 'filled', 'Alpha', 0.7);
    xlabel('Time (s)');
    ylabel('Y Position (m)');
    title('Y Position: Estimated vs GPS');
    legend('Estimated', 'GPS', 'Location', 'best');
    grid on;
    
    % Heading comparison
    subplot(3, 2, 4);
    plot(data.local_states.timestamps, rad2deg(data.local_states.theta), 'b-', 'LineWidth', 2);
    hold on;
    gps_valid = ~isnan(data.gps_measurements.theta);
    scatter(data.local_states.timestamps(gps_valid), ...
           rad2deg(data.gps_measurements.theta(gps_valid)), 20, 'r', 'filled', 'Alpha', 0.7);
    xlabel('Time (s)');
    ylabel('Heading (degrees)');
    title('Heading: Estimated vs GPS');
    legend('Estimated', 'GPS', 'Location', 'best');
    grid on;
    
    % Velocity comparison
    subplot(3, 2, 5);
    plot(data.local_states.timestamps, data.local_states.velocity, 'b-', 'LineWidth', 2);
    hold on;
    gps_valid = ~isnan(data.gps_measurements.velocity);
    scatter(data.local_states.timestamps(gps_valid), ...
           data.gps_measurements.velocity(gps_valid), 20, 'r', 'filled', 'Alpha', 0.7);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Velocity: Estimated vs GPS');
    legend('Estimated', 'GPS', 'Location', 'best');
    grid on;
    
    % Error statistics
    subplot(3, 2, 6);
    error_stats = [];
    labels = {};
    
    if any(gps_valid)
        x_error = data.local_states.x(gps_valid) - data.gps_measurements.x(gps_valid);
        y_error = data.local_states.y(gps_valid) - data.gps_measurements.y(gps_valid);
        theta_error = data.local_states.theta(gps_valid) - data.gps_measurements.theta(gps_valid);
        v_error = data.local_states.velocity(gps_valid) - data.gps_measurements.velocity(gps_valid);
        
        error_stats = [std(x_error), std(y_error), std(rad2deg(theta_error)), std(v_error)];
        labels = {'X (m)', 'Y (m)', 'Theta (deg)', 'Velocity (m/s)'};
        
        bar(error_stats);
        set(gca, 'XTickLabel', labels);
        ylabel('Standard Deviation');
        title('Estimation Error Statistics');
        grid on;
    end
    
    sgtitle(sprintf('Vehicle %d GPS vs Estimation Comparison', vehicle_id));
    
    if options.save_figures
        filename = sprintf('vehicle_%d_gps_comparison.%s', vehicle_id, options.figure_format);
        saveas(gcf, fullfile(fig_dir, filename));
    end
end

function plot_fleet_trajectories(data, vehicle_id, fleet_size, options, fig_dir)
    % Plot trajectories for all vehicles in the fleet
    
    figure('Position', [300, 300, 1200, 900]);
    
    colors = lines(fleet_size);
    
    for i = 1:fleet_size
        vehicle_idx = i - 1; % Convert to 0-based indexing
        field_name = sprintf('vehicle_%d_estimates', vehicle_idx);
        
        if isfield(data, field_name)
            vehicle_data = data.(field_name);
            plot(vehicle_data.x, vehicle_data.y, 'Color', colors(i,:), ...
                 'LineWidth', 2, 'DisplayName', sprintf('Vehicle %d', vehicle_idx));
            hold on;
            
            % Mark start and end points
            plot(vehicle_data.x(1), vehicle_data.y(1), 'o', 'Color', colors(i,:), ...
                 'MarkerSize', 8, 'MarkerFaceColor', colors(i,:));
            plot(vehicle_data.x(end), vehicle_data.y(end), 's', 'Color', colors(i,:), ...
                 'MarkerSize', 8, 'MarkerFaceColor', colors(i,:));
        end
    end
    
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title(sprintf('Fleet Trajectories (Observer: Vehicle %d)', vehicle_id));
    legend('Location', 'best');
    grid on;
    axis equal;
    
    if options.save_figures
        filename = sprintf('fleet_trajectories_observer_%d.%s', vehicle_id, options.figure_format);
        saveas(gcf, fullfile(fig_dir, filename));
    end
end

function plot_fleet_evolution(data, vehicle_id, fleet_size, options, fig_dir)
    % Plot fleet state evolution over time
    
    figure('Position', [350, 350, 1400, 1000]);
    
    colors = lines(fleet_size);
    
    % X positions
    subplot(2, 2, 1);
    for i = 1:fleet_size
        vehicle_idx = i - 1;
        field_name = sprintf('vehicle_%d_estimates', vehicle_idx);
        if isfield(data, field_name)
            vehicle_data = data.(field_name);
            plot(vehicle_data.timestamps, vehicle_data.x, 'Color', colors(i,:), ...
                 'LineWidth', 2, 'DisplayName', sprintf('Vehicle %d', vehicle_idx));
            hold on;
        end
    end
    xlabel('Time (s)');
    ylabel('X Position (m)');
    title('Fleet X Positions');
    legend('Location', 'best');
    grid on;
    
    % Y positions
    subplot(2, 2, 2);
    for i = 1:fleet_size
        vehicle_idx = i - 1;
        field_name = sprintf('vehicle_%d_estimates', vehicle_idx);
        if isfield(data, field_name)
            vehicle_data = data.(field_name);
            plot(vehicle_data.timestamps, vehicle_data.y, 'Color', colors(i,:), ...
                 'LineWidth', 2, 'DisplayName', sprintf('Vehicle %d', vehicle_idx));
            hold on;
        end
    end
    xlabel('Time (s)');
    ylabel('Y Position (m)');
    title('Fleet Y Positions');
    legend('Location', 'best');
    grid on;
    
    % Headings
    subplot(2, 2, 3);
    for i = 1:fleet_size
        vehicle_idx = i - 1;
        field_name = sprintf('vehicle_%d_estimates', vehicle_idx);
        if isfield(data, field_name)
            vehicle_data = data.(field_name);
            plot(vehicle_data.timestamps, rad2deg(vehicle_data.theta), 'Color', colors(i,:), ...
                 'LineWidth', 2, 'DisplayName', sprintf('Vehicle %d', vehicle_idx));
            hold on;
        end
    end
    xlabel('Time (s)');
    ylabel('Heading (degrees)');
    title('Fleet Headings');
    legend('Location', 'best');
    grid on;
    
    % Velocities
    subplot(2, 2, 4);
    for i = 1:fleet_size
        vehicle_idx = i - 1;
        field_name = sprintf('vehicle_%d_estimates', vehicle_idx);
        if isfield(data, field_name)
            vehicle_data = data.(field_name);
            plot(vehicle_data.timestamps, vehicle_data.velocity, 'Color', colors(i,:), ...
                 'LineWidth', 2, 'DisplayName', sprintf('Vehicle %d', vehicle_idx));
            hold on;
        end
    end
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Fleet Velocities');
    legend('Location', 'best');
    grid on;
    
    sgtitle(sprintf('Fleet State Evolution (Observer: Vehicle %d)', vehicle_id));
    
    if options.save_figures
        filename = sprintf('fleet_evolution_observer_%d.%s', vehicle_id, options.figure_format);
        saveas(gcf, fullfile(fig_dir, filename));
    end
end

function plot_estimation_errors(data, vehicle_id, options, fig_dir)
    % Plot estimation error analysis
    
    if ~isfield(data, 'gps_measurements')
        return;
    end
    
    figure('Position', [400, 400, 1400, 800]);
    
    % Calculate errors for valid GPS measurements
    gps_valid = ~isnan(data.gps_measurements.x) & ~isnan(data.gps_measurements.y);
    
    if sum(gps_valid) < 2
        return;
    end
    
    timestamps = data.local_states.timestamps(gps_valid);
    x_error = data.local_states.x(gps_valid) - data.gps_measurements.x(gps_valid);
    y_error = data.local_states.y(gps_valid) - data.gps_measurements.y(gps_valid);
    pos_error = sqrt(x_error.^2 + y_error.^2);
    
    theta_valid = gps_valid & ~isnan(data.gps_measurements.theta);
    theta_error = [];
    if sum(theta_valid) > 0
        theta_error = data.local_states.theta(theta_valid) - data.gps_measurements.theta(theta_valid);
        % Wrap angle errors to [-pi, pi]
        theta_error = mod(theta_error + pi, 2*pi) - pi;
    end
    
    v_valid = gps_valid & ~isnan(data.gps_measurements.velocity);
    v_error = [];
    if sum(v_valid) > 0
        v_error = data.local_states.velocity(v_valid) - data.gps_measurements.velocity(v_valid);
    end
    
    % Position errors
    subplot(2, 3, 1);
    plot(timestamps, x_error, 'r-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('X Error (m)');
    title('X Position Error');
    grid on;
    
    subplot(2, 3, 2);
    plot(timestamps, y_error, 'g-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Y Error (m)');
    title('Y Position Error');
    grid on;
    
    subplot(2, 3, 3);
    plot(timestamps, pos_error, 'b-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Position Error (m)');
    title('Total Position Error');
    grid on;
    
    % Heading error
    subplot(2, 3, 4);
    if ~isempty(theta_error)
        plot(data.local_states.timestamps(theta_valid), rad2deg(theta_error), 'm-', 'LineWidth', 2);
    end
    xlabel('Time (s)');
    ylabel('Heading Error (degrees)');
    title('Heading Error');
    grid on;
    
    % Velocity error
    subplot(2, 3, 5);
    if ~isempty(v_error)
        plot(data.local_states.timestamps(v_valid), v_error, 'c-', 'LineWidth', 2);
    end
    xlabel('Time (s)');
    ylabel('Velocity Error (m/s)');
    title('Velocity Error');
    grid on;
    
    % Error statistics
    subplot(2, 3, 6);
    error_means = [mean(abs(x_error)), mean(abs(y_error)), mean(pos_error)];
    error_stds = [std(x_error), std(y_error), std(pos_error)];
    
    if ~isempty(theta_error)
        error_means(4) = mean(abs(theta_error));
        error_stds(4) = std(theta_error);
    end
    
    if ~isempty(v_error)
        error_means(5) = mean(abs(v_error));
        error_stds(5) = std(v_error);
    end
    
    bar_labels = {'|X|', '|Y|', 'Pos', 'Theta', 'Vel'};
    bar_labels = bar_labels(1:length(error_means));
    
    bar(error_means);
    set(gca, 'XTickLabel', bar_labels);
    ylabel('Mean Absolute Error');
    title('Error Statistics');
    grid on;
    
    sgtitle(sprintf('Vehicle %d Estimation Error Analysis', vehicle_id));
    
    if options.save_figures
        filename = sprintf('vehicle_%d_error_analysis.%s', vehicle_id, options.figure_format);
        saveas(gcf, fullfile(fig_dir, filename));
    end
end

%% Example usage
% plot_observer_data('data_logs/run_20231201_143052/observer_data_vehicle_0_20231201_143052.mat');
%
% With custom options:
% options.save_figures = true;
% options.show_gps = true;
% options.fleet_comparison = true;
% options.figure_format = 'pdf';
% plot_observer_data('your_data_file.mat', options);
