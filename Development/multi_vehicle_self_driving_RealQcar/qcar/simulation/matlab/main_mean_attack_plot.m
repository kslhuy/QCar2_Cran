%%%%%%%%%
%%%%%%%%%
%%%%%%%%%
%%%%%%%%%
%% Here plot mean for all scenarios with the cyberattack ( Bogus type ).

Config

% Log and Debug related
IsShowAnimation = false;
debug_mode = false;
if (debug_mode )
    dbstop if error;
    % dbstop in Observer at 75 if instant_index>=1000;
end


%% Attack and communication modules are reset inside each attack case.



t_star = 10;
t_end = 15;
attacker_vehicle_id = 1;
victim_id = -1;
data_type_attack = "global"; % "local" , "global",
attack_type = "Mix_test"; % "DoS" , "faulty" , "scaling" , "Collusion" ,"Bogus" , "POS" , "VEL" , "ACC" , "Mix_test"


% ──────────────────────────────────────────────────────────────────────────────
% 3) Excel logging setup
excel_filename = sprintf('Results_%s_Attacker_V%d.xlsx', attack_type, attacker_vehicle_id);
sheet_name     = 'Summary';
headers = {'AttackType','AttackerVehicle','Case','Vehicle', ...
    'Mean_Distance','Mean_Orientation','Mean_Velocity','Mean_Acceleration', ...
    'Mean_Trust_Score','Trust_Degradation','Attack_Detection_Time'};

% Write headers if file does not exist
if ~isfile(excel_filename)
    writecell(headers, excel_filename, 'Sheet', sheet_name, 'Range', 'A1');
end

% Read existing rows to find next empty row
raw = readcell(excel_filename, 'Sheet', sheet_name);
row_index = size(raw,1) + 1;



% Vehicles are initialized in the same lane with fixed platoon spacing.
initial_lane_id = 1;
direction_flag = 0; % 1 stands for changing to the left adjacent lane, 0 stands for keeping the current lane, -1 stands for changing

% Fixed lead scenario for all attack cases
Scenarios_config.set_Lead_Senarios("constant");




start_attack_senarios_index = 1;
last_attack_senarios_index = 6;  % All Mix_test cases (1-6) - matches Atk_Scenarios.m

attack_case_numbers = start_attack_senarios_index:last_attack_senarios_index;
total_num_attack_cases = numel(attack_case_numbers);

num_vehicles = 5;
all_vehicle_ids = 1:num_vehicles;
non_attacker_ids = all_vehicle_ids(all_vehicle_ids ~= attacker_vehicle_id);

% Override the 4-vehicle graph from Config for this 5-vehicle workflow.
graph = ones(num_vehicles) - eye(num_vehicles);
clear('Weight_Trust_module');
weight_trust_module = Weight_Trust_module(graph, trust_threshold, kappa);

vehicle_labels = arrayfun(@(x) sprintf('V%d', x), all_vehicle_ids, 'UniformOutput', false);
scenario_labels = arrayfun(@(x) sprintf("Case %d", x), attack_case_numbers, 'UniformOutput', false);
metric_labels = {'Distance Error (m)', 'Orientation Error (rad)', 'Velocity Error (m/s)','Acc Error (m/s)'};

mean_errors = zeros(num_vehicles, length(metric_labels), total_num_attack_cases); % vehicles x metrics x attack cases


is_plot_each_case = false;

initial_x_positions = 80:-20:(80 - 20 * (num_vehicles - 1));
initial_speeds = [23, repmat(26, 1, num_vehicles - 1)];
controller_types = ["None", repmat("IDM", 1, num_vehicles - 1)];
controller2_types = ["None", repmat("CACC", 1, num_vehicles - 1)];

% Give each vehicle a small fixed parameter mismatch, reused for all cases.
rng_state = rng;
rng(10);
param_spread = 0.03;
vehicle_params = cell(num_vehicles, 1);
for vehicle_id = all_vehicle_ids
    veh_param = param_sys;
    scale = @(spread) 1 + spread * (2 * rand - 1);

    veh_param.l_f = param_sys.l_f * scale(param_spread);
    veh_param.l_r = param_sys.l_r * scale(param_spread);
    veh_param.l_fc = param_sys.l_fc * scale(param_spread);
    veh_param.l_rc = param_sys.l_rc * scale(param_spread);
    veh_param.width = param_sys.width * scale(0.01);
    veh_param.tau = param_sys.tau * scale(param_spread);
    veh_param.tau_v = param_sys.tau_v * scale(param_spread);
    veh_param.mass = param_sys.mass * scale(0.05);
    veh_param.C1 = param_sys.C1 * scale(0.05);
    veh_param.C2 = param_sys.C2 * scale(0.05);

    veh_param.max_acceleration = param_sys.max_acceleration * scale(0.05);
    veh_param.min_acceleration = param_sys.min_acceleration * scale(0.05);

    max_steer = param_sys.max_steering_angle * scale(0.02);
    veh_param.max_steering_angle = max_steer;
    veh_param.min_steering_angle = -max_steer;

    vehicle_params{vehicle_id} = veh_param;
end
rng(rng_state);

for case_idx = 1:total_num_attack_cases
    case_nb_attack = attack_case_numbers(case_idx);
    fprintf('Running %s attack by V%d, Case %d/%d...\n', ...
        attack_type, attacker_vehicle_id, case_idx, total_num_attack_cases);

    attack_module = Attack_module(Scenarios_config.dt);
    attack_module = Atk_Scenarios(attack_module , attack_type ,data_type_attack,case_nb_attack , t_star, t_end, attacker_vehicle_id,victim_id );
    center_communication = CenterCommunication(attack_module);


    platton_vehicles = Vehicle.empty;
    for vehicle_id = all_vehicle_ids
        initial_state = [initial_x_positions(vehicle_id); 0.5 * lane_width; 0; initial_speeds(vehicle_id); 0];
        new_vehicle = Vehicle(vehicle_id, controller_types(vehicle_id), vehicle_params{vehicle_id}, initial_state, initial_lane_id, straightLanes, direction_flag, 0, Scenarios_config, weight_trust_module);
        platton_vehicles = [platton_vehicles; new_vehicle];
    end

    for vehicle_id = all_vehicle_ids
        platton_vehicles(vehicle_id).assign_neighbor_vehicle(platton_vehicles, [], controller2_types(vehicle_id), center_communication, graph);
    end


    %% define a simulator and start simulation
    simulator0 = Simulator(straightLanes, [] , platton_vehicles, Scenarios_config.dt , IsShowAnimation );
    [state_log, input_log] = simulator0.startSimulation(Scenarios_config.simulation_time,t_star, t_end, attacker_vehicle_id);

    % plot
    if (is_plot_each_case)
        for vehicle_id = all_vehicle_ids
            platton_vehicles(vehicle_id).plot_ground_error_global_est(platton_vehicles);
        end
    end
    % After the simulation, calculate errors for each vehicle
    % Select vehicles to evaluate based on IDs
    vehicles_to_evaluate = platton_vehicles(non_attacker_ids);

    all_global_dist_errors = []; % Initialize arrays to store global errors
    all_global_theta_errors = []; % global orientation errors of vehicles to evaluate
    all_global_vel_errors = [];
    all_global_acc_errors = [];
    
    % Trust-related metrics
    all_trust_scores = [];
    trust_degradation = [];
    attack_detection_times = [];

    for k = 1:length(vehicles_to_evaluate)
        v = vehicles_to_evaluate(k);
        [dist_err, theta_err, vel_err,global_acc_err] = v.observer.calculate_global_errors();  % Call global error calculation
        all_global_dist_errors = [all_global_dist_errors, dist_err];
        all_global_theta_errors = [all_global_theta_errors, theta_err];
        all_global_vel_errors = [all_global_vel_errors, vel_err];
        all_global_acc_errors = [all_global_acc_errors, global_acc_err];
        
        % Extract each evaluator's trust in the attacker from Vehicle.trust_log.
        trust_trace = squeeze(v.trust_log(1, :, attacker_vehicle_id));
        attack_start_idx = max(1, round(t_star / Scenarios_config.dt));
        attack_end_idx = min(length(trust_trace), round(t_end / Scenarios_config.dt));

        if attack_start_idx <= length(trust_trace) && attack_start_idx <= attack_end_idx
            pre_attack_end_idx = max(1, attack_start_idx - 1);
            pre_attack_trust = mean(trust_trace(1:pre_attack_end_idx), 'omitnan');
            during_attack_trust = mean(trust_trace(attack_start_idx:attack_end_idx), 'omitnan');

            all_trust_scores = [all_trust_scores; mean(trust_trace, 'omitnan')];
            trust_degradation = [trust_degradation; pre_attack_trust - during_attack_trust];

            trust_threshold_detection = 0.7;
            detection_idx = find(trust_trace(attack_start_idx:attack_end_idx) < trust_threshold_detection, 1);
            if ~isempty(detection_idx)
                detection_time = (attack_start_idx + detection_idx - 1) * Scenarios_config.dt;
                attack_detection_times = [attack_detection_times; detection_time];
            else
                attack_detection_times = [attack_detection_times; NaN];
            end
        else
            all_trust_scores = [all_trust_scores; NaN];
            trust_degradation = [trust_degradation; NaN];
            attack_detection_times = [attack_detection_times; NaN];
        end
    end





    % Calculate consensus RMSE across all observer vehicles for each target vehicle
    % Each observer vehicle estimates all other vehicles -> average their RMSE estimates
    % Result: mean_dist(i) = average RMSE for vehicle i across all observers
    mean_dist = mean(all_global_dist_errors,2);   % Average RMSE across observers (dim 2)
    mean_theta = mean(all_global_theta_errors,2); % Average RMSE across observers (dim 2)
    mean_vel = mean(all_global_vel_errors,2);     % Average RMSE across observers (dim 2)
    mean_acc = mean(all_global_acc_errors,2);     % Average RMSE across observers (dim 2)

    % write one row per evaluating vehicle
    for vi = 1:length(non_attacker_ids)
        target_id = non_attacker_ids(vi);
        % Handle trust metrics safely
        if isempty(all_trust_scores) || vi > length(all_trust_scores)
            trust_score = NaN;
        else
            trust_score = all_trust_scores(vi);
        end
        
        if isempty(trust_degradation) || vi > length(trust_degradation)
            trust_deg = NaN;
        else
            trust_deg = trust_degradation(vi);
        end
        
        if isempty(attack_detection_times) || vi > length(attack_detection_times)
            detection_time = NaN;
        else
            detection_time = attack_detection_times(vi);
        end
        
        row_data = {
            attack_type, ...
            sprintf('V%d', attacker_vehicle_id), ...
            sprintf('Case %d', case_nb_attack), ...
            sprintf('V%d', target_id), ...
            mean_dist(target_id), ...
            mean_theta(target_id), ...
            mean_vel(target_id), ...
            mean_acc(target_id), ...
            trust_score, ...
            trust_deg, ...
            detection_time
        };
        writecell(row_data, excel_filename, 'Sheet', sheet_name, ...
                  'Range', sprintf('A%d', row_index));
        row_index = row_index + 1;
    end



    % Store the errors for plotting
    mean_errors(:, :, case_idx) = [mean_dist, mean_theta, mean_vel, mean_acc];
    
    % Store trust data for direct analysis - COLLECT ALL CASES
    if ~exist('all_case_trust_logs', 'var')
        all_case_trust_logs = cell(total_num_attack_cases, 1);
        all_case_vehicles = cell(total_num_attack_cases, 1);
        all_case_scenarios = cell(total_num_attack_cases, 1);
    end
    
    % Store complete trust data for each case
    all_case_trust_logs{case_idx} = struct();
    all_case_vehicles{case_idx} = platton_vehicles;
    all_case_scenarios{case_idx} = struct('t_start', t_star, 't_end', t_end, 'dt', Scenarios_config.dt, 'case_number', case_nb_attack);
    
    % Collect trust logs from all vehicles for this case
    for v_idx = 1:length(platton_vehicles)
        vehicle = platton_vehicles(v_idx);
        if isprop(vehicle, 'trust_log')
            all_case_trust_logs{case_idx}.(sprintf('vehicle_%d', v_idx)) = vehicle.trust_log;
        end
        
        % Also store trip model data
        if isprop(vehicle, 'trip_models')
            for tm_idx = 1:length(vehicle.trip_models)
                if ~isempty(vehicle.trip_models{tm_idx})
                    trust_model = vehicle.trip_models{tm_idx};
                    field_name = sprintf('trust_model_v%d_to_v%d', v_idx, tm_idx);
                    all_case_trust_logs{case_idx}.(field_name) = struct( ...
                        'trust_samples', trust_model.trust_sample_log, ...
                        'final_scores', trust_model.final_score_log, ...
                        'gamma_cross', trust_model.gamma_cross_log, ...
                        'gamma_local', trust_model.gamma_local_log, ...
                        'v_score', trust_model.v_score_log, ...
                        'd_score', trust_model.d_score_log, ...
                        'a_score', trust_model.a_score_log);
                end
            end
        end
    end
end

%% ===================================================================
%% PAPER-QUALITY PLOTTING SECTION
%% ===================================================================

% Define Mix_test attack case descriptions for better labeling (complete list)
all_attack_descriptions = {
    'P Bias -5m', 'P Faulty 10m', 'V Bias -2m/s', 'V Faulty 2.5m/s', ...
    'A Faulty 1.0m/s²', 'DoS Attack'
};

% Select attack descriptions based on the scenario range
attack_descriptions = all_attack_descriptions(start_attack_senarios_index:last_attack_senarios_index);

%% 1. COMPREHENSIVE ESTIMATION ERROR ANALYSIS (WITHOUT ORIENTATION)
figure('Position', [100, 100, 1200, 600]);

% Subplot 1: Distance Estimation Error
subplot(1,3,1);
data = squeeze(mean_errors(non_attacker_ids, 1, :))';  % Distance errors for non-attackers
bar(data);
xticklabels(attack_descriptions(1:size(data,1)));
xtickangle(45);
set(gca, 'FontSize', 9);  % Make text smaller
ylabel('Distance Error (m)');
title('(a) Distance Estimation Error');
legend(cellstr("V" + string(non_attacker_ids)), 'Location', 'best');
grid on;

% Subplot 2: Velocity Estimation Error  
subplot(1,3,2);
data = squeeze(mean_errors(non_attacker_ids, 3, :))';  % Velocity errors
bar(data);
xticklabels(attack_descriptions(1:size(data,1)));
xtickangle(45);
set(gca, 'FontSize', 9);  % Make text smaller
ylabel('Velocity Error (m/s)');
title('(b) Velocity Estimation Error');
legend(cellstr("V" + string(non_attacker_ids)), 'Location', 'best');
grid on;

% Subplot 3: Acceleration Estimation Error
subplot(1,3,3);
data = squeeze(mean_errors(non_attacker_ids, 4, :))';  % Acceleration errors
bar(data);
xticklabels(attack_descriptions(1:size(data,1)));
xtickangle(45);
set(gca, 'FontSize', 9);  % Make text smaller
ylabel('Acceleration Error (m/s²)');
title('(c) Acceleration Estimation Error');
legend(cellstr("V" + string(non_attacker_ids)), 'Location', 'best');
grid on;

sgtitle(sprintf('Distributed Estimation Performance Under %s Attacks (Attacker: V%d)', attack_type, attacker_vehicle_id), 'FontSize', 14, 'FontWeight', 'bold');

%% 2. MEANINGFUL ATTACK IMPACT HEATMAPS (WITHOUT ORIENTATION)
% Option 1: Separate heatmaps for each error type (RECOMMENDED)
figure('Position', [200, 200, 1200, 600]);

% Select only non-orientation metrics: Distance(1), Velocity(3), Acceleration(4)
selected_metrics = [1, 3, 4];
selected_labels = {'Distance Error (m)', 'Velocity Error (m/s)', 'Acc Error (m/s²)'};

for plot_idx = 1:3
    metric_idx = selected_metrics(plot_idx);
    subplot(1, 3, plot_idx);
    
    % Extract data for this specific metric across all vehicles and cases
    metric_data = squeeze(mean_errors(non_attacker_ids, metric_idx, :));  % vehicles × cases
    
    % Create heatmap
    imagesc(metric_data);
    colorbar;
    
    % Customize labels and title
    xlabel('Attack Cases');
    ylabel('Vehicles');
    title(sprintf('%s Impact', selected_labels{plot_idx}));
    
    % Set tick labels
    if size(metric_data, 2) <= length(attack_descriptions)
        xticklabels(attack_descriptions(1:size(metric_data, 2)));
    end
    % Center the y-tick labels properly
    num_vehicles = length(non_attacker_ids);
    yticks(1:num_vehicles);
    vehicle_labels = arrayfun(@(x) sprintf('V%d', x), non_attacker_ids, 'UniformOutput', false);
    yticklabels(vehicle_labels);
    xtickangle(45);
    set(gca, 'FontSize', 9);  % Make text smaller for heatmaps
    % Force y-tick labels to be centered
    set(gca, 'TickLength', [0 0]);  % Remove tick marks
    ylim([0.5, num_vehicles + 0.5]);  % Set proper y-axis limits
    
    % Add text annotations with simple black text
    for i = 1:size(metric_data, 1)
        for j = 1:size(metric_data, 2)
            text(j, i, sprintf('%.3f', metric_data(i,j)), ...
                 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
                 'Color', 'black', 'FontWeight', 'bold', 'FontSize', 11, ...
                 'EdgeColor', 'none');
        end
    end
end
sgtitle(sprintf('Attack Impact Analysis by Error Type - %s Attacks by V%d', attack_type, attacker_vehicle_id), ...
        'FontSize', 14, 'FontWeight', 'bold');

% Option 2: Normalized Combined Impact Heatmap
figure('Position', [300, 100, 1000, 600]);

% Normalize each metric to [0,1] scale before combining
normalized_errors = zeros(size(mean_errors));
for metric_idx = 1:4
    metric_slice = mean_errors(:, metric_idx, :);
    min_val = min(metric_slice(:));           % Find minimum error for THIS metric
    max_val = max(metric_slice(:));           % Find maximum error for THIS metric
    if max_val > min_val
        normalized_errors(:, metric_idx, :) = (metric_slice - min_val) / (max_val - min_val);
    end
end

% Calculate weighted impact score with equal weighting for distance, velocity, and acceleration
weights = [1/3, 0.0, 1/3, 1/3];  % [distance, orientation(excluded), velocity, acceleration] - Equal weighting
impact_scores = squeeze(sum(normalized_errors .* reshape(weights, 1, 4, 1), 2));

% Create normalized impact heatmap
imagesc(impact_scores(non_attacker_ids, :));
colorbar;
xlabel('Attack Cases');
ylabel('Vehicles');
title(sprintf('Normalized Combined Impact Score - %s Attacks by V%d', attack_type, attacker_vehicle_id));
xticklabels(attack_descriptions(1:size(impact_scores, 2)));
% Center the y-tick labels properly
num_vehicles = length(non_attacker_ids);
yticks(1:num_vehicles);
vehicle_labels = arrayfun(@(x) sprintf('V%d', x), non_attacker_ids, 'UniformOutput', false);
yticklabels(vehicle_labels);
xtickangle(45);
set(gca, 'FontSize', 8);  % Make text smaller for heatmaps
% Force y-tick labels to be centered
set(gca, 'TickLength', [0 0]);  % Remove tick marks
ylim([0.5, num_vehicles + 0.5]);  % Set proper y-axis limits

% Add text annotations with simple black text
for i = 1:length(non_attacker_ids)
    for j = 1:size(impact_scores, 2)
        score = impact_scores(non_attacker_ids(i), j);
        
        text(j, i, sprintf('%.2f', score), ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
             'Color', 'black', 'FontWeight', 'bold', 'FontSize', 11, ...
             'EdgeColor', 'none');
    end
end

% %% Option 3: Raw Average Combined Error Heatmap (Non-normalized)
% figure('Position', [400, 200, 1000, 600]);

% % Calculate raw combined errors (excluding orientation) - sum all error types
% raw_selected_metrics = [1, 3, 4]; % Distance(1), Velocity(3), Acceleration(4) - excluding orientation(2)

% % Extract only the selected metrics for non-attacker vehicles
% raw_errors_selected = mean_errors(non_attacker_ids, raw_selected_metrics, :); % 3 vehicles × 3 metrics × 6 cases

% % Calculate weighted average for each vehicle and case
% raw_combined_errors = zeros(length(non_attacker_ids), size(mean_errors, 3));
% for i = 1:length(non_attacker_ids)
%     for j = 1:size(mean_errors, 3)
%         vehicle_errors = squeeze(raw_errors_selected(i, :, j)); % 3 metrics for this vehicle and case
%         raw_combined_errors(i, j) = sum(vehicle_errors); % Total combined RMSE (sum of all error types)
%     end
% end

% % Create raw error heatmap
% imagesc(raw_combined_errors);
% colorbar;
% xlabel('Attack Cases');
% ylabel('Vehicles');
% title(sprintf('Raw Average Combined Error (Distance + Velocity + Acceleration) - Attacks by V%d', attacker_vehicle_id));
% xticklabels(attack_descriptions(1:size(raw_combined_errors, 2)));
% % Center the y-tick labels properly
% num_vehicles = length(non_attacker_ids);
% yticks(1:num_vehicles);
% vehicle_labels = arrayfun(@(x) sprintf('V%d', x), non_attacker_ids, 'UniformOutput', false);
% yticklabels(vehicle_labels);
% xtickangle(45);
% set(gca, 'FontSize', 8);  % Make text smaller for heatmaps
% % Force y-tick labels to be centered
% set(gca, 'TickLength', [0 0]);  % Remove tick marks
% ylim([0.5, num_vehicles + 0.5]);  % Set proper y-axis limits

% % Add text annotations with simple black text
% for i = 1:length(non_attacker_ids)
%     for j = 1:size(raw_combined_errors, 2)
%         raw_error = raw_combined_errors(i, j);
        
%         text(j, i, sprintf('%.3f', raw_error), ...
%              'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
%              'Color', 'black', 'FontWeight', 'bold', 'FontSize', 11, ...
%              'EdgeColor', 'none');
%     end
% end

% %% 3. VULNERABILITY ANALYSIS PER VEHICLE (WITHOUT ORIENTATION)
% figure('Position', [300, 300, 1000, 400]);
% selected_metrics = [1, 3, 4]; % Distance, Velocity, Acceleration (excluding orientation)
% selected_labels = {'Distance Error (m)', 'Velocity Error (m/s)', 'Acc Error (m/s²)'};

% for v_idx = 1:length(non_attacker_ids)
%     subplot(1, length(non_attacker_ids), v_idx);
%     vehicle_errors = squeeze(mean_errors(non_attacker_ids(v_idx), selected_metrics, :));  % Selected metrics for this vehicle
%     bar(vehicle_errors');
%     xticklabels(attack_descriptions(1:size(vehicle_errors,2)));
%     xtickangle(45);
%     set(gca, 'FontSize', 8);  % Make text smaller
%     ylabel('Error Magnitude');
%     title(sprintf('V%d Vulnerability', non_attacker_ids(v_idx)));
%     legend(selected_labels, 'Location', 'best', 'FontSize', 8);
%     grid on;
% end
% sgtitle('Individual Vehicle Vulnerability Analysis', 'FontSize', 14, 'FontWeight', 'bold');

%% 4. HYBRID HEATMAP: NORMALIZED IMPACT COLORS WITH RAW ERROR VALUES
figure('Position', [500, 300, 1000, 600]);

% Use normalized impact scores for colormap, but show raw combined errors as text
imagesc(impact_scores(non_attacker_ids, :));
colorbar;
xlabel('Attack Cases');
ylabel('Vehicles');
title(sprintf('Hybrid Analysis: Impact Score Colors + Raw Error Values - %s Attacks by V%d', attack_type, attacker_vehicle_id));

% Set axis labels and formatting
xticklabels(attack_descriptions(1:size(impact_scores, 2)));
num_vehicles = length(non_attacker_ids);
yticks(1:num_vehicles);
vehicle_labels = arrayfun(@(x) sprintf('V%d', x), non_attacker_ids, 'UniformOutput', false);
yticklabels(vehicle_labels);
xtickangle(45);
set(gca, 'FontSize', 8);
set(gca, 'TickLength', [0 0]);
ylim([0.5, num_vehicles + 0.5]);

% Add text annotations showing RAW COMBINED ERROR values
for i = 1:length(non_attacker_ids)
    for j = 1:size(raw_combined_errors, 2)
        raw_error = raw_combined_errors(i, j);
        
        % Display raw error value as text
        text(j, i, sprintf('%.3f', raw_error), ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
             'Color', 'black', 'FontWeight', 'bold', 'FontSize', 11, ...
             'EdgeColor', 'none');
    end
end

% Add a colorbar title to clarify what the colors represent
c = colorbar;
c.Label.String = 'Normalized Impact Score (0-1)';
c.Label.FontSize = 10;
c.Label.FontWeight = 'bold';

% Add legend/explanation
annotation('textbox', [0.02, 0.95, 0.3, 0.05], ...
    'String', 'Colors: Normalized Impact | Numbers: Raw Combined RMSE', ...
    'FontSize', 10, 'FontWeight', 'bold', ...
    'BackgroundColor', 'white', 'EdgeColor', 'black', ...
    'HorizontalAlignment', 'left');

%% 5. SUMMARY STATISTICS FOR PAPER
fprintf('\n=== SUMMARY STATISTICS FOR PAPER ===\n');
fprintf('Attack Type: %s, Attacker: V%d\n', attack_type, attacker_vehicle_id);
fprintf('Total Attack Cases Tested: %d\n', total_num_attack_cases);
fprintf('Attack Duration: %.1f seconds (t=%.1fs to %.1fs)\n', t_end-t_star, t_star, t_end);

%% RAW COMBINED ERROR ANALYSIS
fprintf('\n=== RAW AVERAGE COMBINED ERROR ANALYSIS ===\n');

% Get the raw combined error data for analysis
raw_error_data = raw_combined_errors;

% Find attack case with highest raw error
[max_raw_per_case, ~] = max(raw_error_data, [], 1);  % Max raw error across vehicles for each case
[overall_max_raw, most_severe_raw_case] = max(max_raw_per_case);
fprintf('Highest Raw Error Attack: Case %d (%s) - Max Error: %.4f\n', ...
    most_severe_raw_case, attack_descriptions{most_severe_raw_case}, overall_max_raw);

% Find attack case with lowest raw error
[min_raw_per_case, ~] = min(raw_error_data, [], 1);  % Min raw error across vehicles for each case
[overall_min_raw, least_severe_raw_case] = min(min_raw_per_case);
fprintf('Lowest Raw Error Attack: Case %d (%s) - Min Error: %.4f\n', ...
    least_severe_raw_case, attack_descriptions{least_severe_raw_case}, overall_min_raw);

% Find vehicle with highest raw error across all attacks
[max_raw_per_vehicle, ~] = max(raw_error_data, [], 2);  % Max raw error across cases for each vehicle
[vehicle_max_raw, most_affected_vehicle_idx] = max(max_raw_per_vehicle);
most_affected_vehicle = non_attacker_ids(most_affected_vehicle_idx);
fprintf('Most Affected Vehicle: V%d - Max Raw Error: %.4f\n', ...
    most_affected_vehicle, vehicle_max_raw);

% Find vehicle with lowest raw error across all attacks
[min_raw_per_vehicle, ~] = min(raw_error_data, [], 2);  % Min raw error across cases for each vehicle
[vehicle_min_raw, least_affected_vehicle_idx] = min(min_raw_per_vehicle);
least_affected_vehicle = non_attacker_ids(least_affected_vehicle_idx);
fprintf('Most Robust Vehicle: V%d - Min Raw Error: %.4f\n', ...
    least_affected_vehicle, vehicle_min_raw);

% Average raw error per attack case
avg_raw_per_case = mean(raw_error_data, 1);
fprintf('\nAverage Raw Combined Error by Attack Case:\n');
for i = 1:length(attack_descriptions)
    fprintf('  Case %d (%s): %.4f\n', i, attack_descriptions{i}, avg_raw_per_case(i));
end

% Average raw error per vehicle
avg_raw_per_vehicle = mean(raw_error_data, 2);
fprintf('\nAverage Raw Combined Error by Vehicle:\n');
for i = 1:length(non_attacker_ids)
    fprintf('  V%d: %.4f\n', non_attacker_ids(i), avg_raw_per_vehicle(i));
end

% Raw error severity classification based on actual error magnitudes
critical_error_threshold = 2.0;   % Combined error > 2.0 is critical
high_error_threshold = 1.0;       % Combined error > 1.0 is high
moderate_error_threshold = 0.5;   % Combined error > 0.5 is moderate

fprintf('\n=== RAW ERROR SEVERITY CLASSIFICATION ===\n');
fprintf('Critical Error (>%.1f): System performance severely compromised\n', critical_error_threshold);
fprintf('High Error (%.1f-%.1f): Significant performance degradation\n', high_error_threshold, critical_error_threshold);
fprintf('Moderate Error (%.1f-%.1f): Noticeable but manageable impact\n', moderate_error_threshold, high_error_threshold);
fprintf('Low Error (<%.1f): Minimal impact on system performance\n', moderate_error_threshold);

critical_cases = find(max_raw_per_case > critical_error_threshold);
high_cases = find(max_raw_per_case > high_error_threshold & max_raw_per_case <= critical_error_threshold);
moderate_cases = find(max_raw_per_case > moderate_error_threshold & max_raw_per_case <= high_error_threshold);
low_cases = find(max_raw_per_case <= moderate_error_threshold);

if ~isempty(critical_cases)
    fprintf('\nCRITICAL ERROR CASES: ');
    for i = 1:length(critical_cases)
        fprintf('Case %d (%s) ', critical_cases(i), attack_descriptions{critical_cases(i)});
    end
    fprintf('\n');
end

if ~isempty(high_cases)
    fprintf('HIGH ERROR CASES: ');
    for i = 1:length(high_cases)
        fprintf('Case %d (%s) ', high_cases(i), attack_descriptions{high_cases(i)});
    end
    fprintf('\n');
end

if ~isempty(moderate_cases)
    fprintf('MODERATE ERROR CASES: ');
    for i = 1:length(moderate_cases)
        fprintf('Case %d (%s) ', moderate_cases(i), attack_descriptions{moderate_cases(i)});
    end
    fprintf('\n');
end

if ~isempty(low_cases)
    fprintf('LOW ERROR CASES: ');
    for i = 1:length(low_cases)
        fprintf('Case %d (%s) ', low_cases(i), attack_descriptions{low_cases(i)});
    end
    fprintf('\n');
end

% Error magnitude distribution analysis
fprintf('\n=== ERROR MAGNITUDE DISTRIBUTION ===\n');
overall_avg_raw = mean(raw_error_data(:));
overall_std_raw = std(raw_error_data(:));
overall_max_raw_all = max(raw_error_data(:));
overall_min_raw_all = min(raw_error_data(:));

fprintf('Overall Statistics:\n');
fprintf('  Mean Combined Error: %.4f\n', overall_avg_raw);
fprintf('  Standard Deviation: %.4f\n', overall_std_raw);
fprintf('  Maximum Error: %.4f\n', overall_max_raw_all);
fprintf('  Minimum Error: %.4f\n', overall_min_raw_all);
fprintf('  Error Range: %.4f\n', overall_max_raw_all - overall_min_raw_all);

%% ERROR COMPONENT CONTRIBUTION ANALYSIS
fprintf('\n=== ERROR COMPONENT CONTRIBUTION ANALYSIS ===\n');

% Calculate average contribution of each error type to total combined error
% Extract individual error components for non-attacker vehicles
dist_errors_only = squeeze(mean_errors(non_attacker_ids, 1, :));  % Distance errors
vel_errors_only = squeeze(mean_errors(non_attacker_ids, 3, :));   % Velocity errors  
acc_errors_only = squeeze(mean_errors(non_attacker_ids, 4, :));   % Acceleration errors

% Calculate average contribution percentages
avg_dist_error = mean(dist_errors_only(:));
avg_vel_error = mean(vel_errors_only(:));
avg_acc_error = mean(acc_errors_only(:));
total_avg_error = avg_dist_error + avg_vel_error + avg_acc_error;

% Calculate percentage contributions
dist_contribution = (avg_dist_error / total_avg_error) * 100;
vel_contribution = (avg_vel_error / total_avg_error) * 100;
acc_contribution = (avg_acc_error / total_avg_error) * 100;

fprintf('Average Error Component Contributions to RAW COMBINED ERROR:\n');
fprintf('  Distance Error: %.4f (%.1f%%)\n', avg_dist_error, dist_contribution);
fprintf('  Velocity Error: %.4f (%.1f%%)\n', avg_vel_error, vel_contribution);
fprintf('  Acceleration Error: %.4f (%.1f%%)\n', avg_acc_error, acc_contribution);
fprintf('  Total Combined: %.4f (100.0%%)\n', total_avg_error);

% Find dominant error component
[max_contribution, max_idx] = max([dist_contribution, vel_contribution, acc_contribution]);
component_names = {'Distance', 'Velocity', 'Acceleration'};
fprintf('\nDominant Error Component: %s (%.1f%% of total error)\n', component_names{max_idx}, max_contribution);

% Analyze contribution variability across cases
fprintf('\nContribution Variability Across Attack Cases:\n');
for case_idx = 1:size(dist_errors_only, 2)
    case_dist = mean(dist_errors_only(:, case_idx));
    case_vel = mean(vel_errors_only(:, case_idx));
    case_acc = mean(acc_errors_only(:, case_idx));
    case_total = case_dist + case_vel + case_acc;
    
    if case_total > 0  % Avoid division by zero
        case_dist_pct = (case_dist / case_total) * 100;
        case_vel_pct = (case_vel / case_total) * 100;
        case_acc_pct = (case_acc / case_total) * 100;
        
        fprintf('  %s: Dist %.1f%%, Vel %.1f%%, Acc %.1f%%\n', ...
            attack_descriptions{case_idx}, case_dist_pct, case_vel_pct, case_acc_pct);
    end
end

%% NORMALIZED HEATMAP ANALYSIS
fprintf('\n=== NORMALIZED COMBINED IMPACT ANALYSIS ===\n');

% Get the normalized impact scores for analysis
impact_data = impact_scores(non_attacker_ids, :);

% Find most severe attack case
[max_impact_per_case, ~] = max(impact_data, [], 1);  % Max impact across vehicles for each case
[overall_max_impact, most_severe_case] = max(max_impact_per_case);
fprintf('Most Severe Attack: Case %d (%s) - Max Impact Score: %.3f\n', ...
    most_severe_case, attack_descriptions{most_severe_case}, overall_max_impact);

% Find least severe attack case  
[min_impact_per_case, ~] = min(impact_data, [], 1);  % Min impact across vehicles for each case
[overall_min_impact, least_severe_case] = min(min_impact_per_case);
fprintf('Least Severe Attack: Case %d (%s) - Min Impact Score: %.3f\n', ...
    least_severe_case, attack_descriptions{least_severe_case}, overall_min_impact);

% Find most impacted vehicle across all attacks
[max_impact_per_vehicle, ~] = max(impact_data, [], 2);  % Max impact across cases for each vehicle
[vehicle_max_impact, most_impacted_vehicle_idx] = max(max_impact_per_vehicle);
most_impacted_vehicle = non_attacker_ids(most_impacted_vehicle_idx);
fprintf('Most Impacted Vehicle: V%d - Max Impact Score: %.3f\n', ...
    most_impacted_vehicle, vehicle_max_impact);

% Find least impacted vehicle across all attacks
[min_impact_per_vehicle, ~] = min(impact_data, [], 2);  % Min impact across cases for each vehicle  
[vehicle_min_impact, least_impacted_vehicle_idx] = min(min_impact_per_vehicle);
least_impacted_vehicle = non_attacker_ids(least_impacted_vehicle_idx);
fprintf('Most Resilient Vehicle: V%d - Min Impact Score: %.3f\n', ...
    least_impacted_vehicle, vehicle_min_impact);

% Average impact per attack case
avg_impact_per_case = mean(impact_data, 1);
fprintf('\nAverage Impact Score by Attack Case:\n');
for i = 1:length(attack_descriptions)
    fprintf('  Case %d (%s): %.3f\n', i, attack_descriptions{i}, avg_impact_per_case(i));
end

% Average impact per vehicle
avg_impact_per_vehicle = mean(impact_data, 2);
fprintf('\nAverage Impact Score by Vehicle:\n');
for i = 1:length(non_attacker_ids)
    fprintf('  V%d: %.3f\n', non_attacker_ids(i), avg_impact_per_vehicle(i));
end

% Severity classification
high_severity_threshold = 0.7;
medium_severity_threshold = 0.4;

fprintf('\n=== SEVERITY CLASSIFICATION ===\n');
fprintf('High Severity (>%.1f): Cases where normalized impact > %.1f\n', high_severity_threshold, high_severity_threshold);
fprintf('Medium Severity (%.1f-%.1f): Moderate impact\n', medium_severity_threshold, high_severity_threshold);
fprintf('Low Severity (<%.1f): Minimal impact\n', medium_severity_threshold);

high_severity_cases = find(max_impact_per_case > high_severity_threshold);
if ~isempty(high_severity_cases)
    fprintf('HIGH SEVERITY CASES: ');
    for i = 1:length(high_severity_cases)
        fprintf('Case %d (%s) ', high_severity_cases(i), attack_descriptions{high_severity_cases(i)});
    end
    fprintf('\n');
else
    fprintf('No high severity cases found.\n');
end

% Find most/least vulnerable cases
overall_errors = squeeze(mean(mean(mean_errors(non_attacker_ids, :, :), 1), 2));
[max_error, max_case] = max(overall_errors);
[min_error, min_case] = min(overall_errors);

fprintf('\nMost Severe Attack: Case %d (%s) - Average Error: %.4f\n', max_case, attack_descriptions{max_case}, max_error);
fprintf('Least Severe Attack: Case %d (%s) - Average Error: %.4f\n', min_case, attack_descriptions{min_case}, min_error);

% Vehicle resilience ranking
vehicle_resilience = squeeze(mean(mean(mean_errors(non_attacker_ids, :, :), 2), 3));
[~, resilience_rank] = sort(vehicle_resilience);
fprintf('\nVehicle Resilience Ranking (most to least resilient):\n');
for i = 1:length(resilience_rank)
    v_id = non_attacker_ids(resilience_rank(i));
    fprintf('  %d. V%d (Average Error: %.4f)\n', i, v_id, vehicle_resilience(resilience_rank(i)));
end

fprintf('\nResults saved to: %s\n', excel_filename);
fprintf('Use these figures and statistics in your paper!\n');
