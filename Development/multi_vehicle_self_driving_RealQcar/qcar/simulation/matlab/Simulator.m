classdef Simulator
    properties
        lanes; % Lane configuration for the simulation
        ego_vehicle; % The ego vehicle in the simulation
        other_vehicles; % List of other vehicles in the simulation
        dt; % Time step for the simulation
        show_animation;
    end
    methods
        % Constructor to initialize the Simulator object
        function self = Simulator(lanes, ego_vehicle, other_vehicles, dt,show_animation)
            self.lanes = lanes; % Set lanes
            self.ego_vehicle = ego_vehicle; % Set ego vehicle
            self.other_vehicles = other_vehicles; % Set other vehicles
            self.dt = dt; % Set time step
            self.show_animation = show_animation;
        end

        % Method to start the simulation
        function [state_log, input_log] = startSimulation(self, simulation_time,t_star, t_end, attacker_vehicle_id)
            num = simulation_time / self.dt; % Calculate number of simulation steps
            plot_interval = 5; % Plot every 20 iterations

            % Set up the figure once
            figure(1);
            set(gcf, 'Position', [200, 100, 720, 600]); % Larger figure window

            % grid on;
            % xlabel('X Position (m)', 'FontSize', 12);
            % ylabel('Y Position (m)', 'FontSize', 12);


            % % Set the x and y axis limits
            % xlim([0, 300]); % Set x-axis limits
            % ylim([-30, 30]); % Adjusted for better visibility

            % Start the timer
            tic;
            % Update other vehicles' positions
            num_car = size(self.other_vehicles, 1);
            % Define attack start and end times (in seconds)
            attack_start_time = t_star; % Example: attack starts at 10s
            attack_end_time = t_end;   % Example: attack ends at 15s
            attacker_id = attacker_vehicle_id;        % Example: vehicle 2 is the attacker (indexing starts at 1)

            attacker_x = self.other_vehicles(attacker_id).state(1); % X position of attacker

            for i = 1:num

                if num_car >= 1
                    for k = 1:num_car
                        self.other_vehicles(k).update(i); % Update vehicle state
                    end
                    for k = 1:num_car
                        self.other_vehicles(k).send_data(i); % Update vehicle state
                    end

                end

                if ~isempty(self.ego_vehicle)
                    % Update ego vehicle's position
                    self.ego_vehicle.update; % Update ego vehicle state

                end

                if (self.show_animation)
                    % Plot at specified intervals
                    if (mod(i, plot_interval) == 0)
                        clf; % Clear the current figure

                        self.lanes.plot_lanes; % Plot lanes
                        hold on; % Hold the plot for adding more elements

                        % Plot other vehicles
                        if num_car >= 1
                            for k = 1:num_car
                                self.other_vehicles(k).plot_vehicle; % Plot vehicles
                            end
                        end

                        if ~isempty(self.ego_vehicle)
                            self.ego_vehicle.plot_vehicle; % Plot ego vehicle
                        end

                        % Set the x and y axis limits
                        % xlim([0, 500]); % Set x-axis limits
                        % % Adjust xlim dynamically to follow the ego vehicle

                        if num_car >= 1
                            vehicle_x_positions = arrayfun(@(vehicle) vehicle.state(1), self.other_vehicles);
                            offset_x = mean(vehicle_x_positions);
                        else
                            offset_x = 0;
                        end
                        xlim([offset_x  - 100, offset_x + 200]); % Keep the ego vehicle in the center

                        ylim([-30, 30]); % Adjusted for better visibility

                        grid on;
                        % Add grid and labels for better visualization
                        xlabel('X Position (m)', 'FontSize', 14);
                        ylabel('Y Position (m)', 'FontSize', 14);

                        % Add time elapsed as a text annotation in the figure
                        elapsed_sim_time = (i-1) * self.dt;
                        time_str = sprintf('Time Elapsed: %.2f s', elapsed_sim_time);
                        % Place the text in the top right corner of the axes
                        ax = gca;
                        x_limits = ax.XLim;
                        y_limits = ax.YLim;


                        % Draw vertical red lines at attack start and end at the attacker's position
                        if num_car >= attacker_id
                            % Draw line at attack start time only during attack period
                            if elapsed_sim_time >= attack_start_time && elapsed_sim_time <= attack_end_time && attacker_x >= x_limits(1) && attacker_x <= x_limits(2)
                                plot([attacker_x attacker_x], y_limits, 'r-', 'LineWidth', 2);
                                % Optionally, add a label
                                text(attacker_x, y_limits(2), sprintf('Attack Start (V%d)', attacker_id), 'Color', 'r', 'FontWeight', 'bold', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center');
                            end
                            % Draw line at attack end time only at the moment attack ends
                            if abs(elapsed_sim_time - attack_end_time) < self.dt/2 && attacker_x >= x_limits(1) && attacker_x <= x_limits(2)
                                plot([attacker_x attacker_x], y_limits, 'r--', 'LineWidth', 2);
                                text(attacker_x, y_limits(1), sprintf('Attack End (V%d)', attacker_id), 'Color', 'r', 'FontWeight', 'bold', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'center');
                            end
                        end

                        % Change time string color to red during attack
                        if elapsed_sim_time >= attack_start_time && elapsed_sim_time <= attack_end_time
                            time_color = 'r';
                        else
                            time_color = 'k';
                        end
                        text(x_limits(2) - 0.05*diff(x_limits), y_limits(2) - 0.05*diff(y_limits), time_str, ...
                            'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', 'BackgroundColor', 'w', 'EdgeColor', 'k', 'Color', time_color);

                        % Update the plot
                        drawnow; % Ensures smooth rendering of the animation
                    end

                    % Synchronize with real-time
                    elapsed_time = toc;
                    expected_time = i * self.dt;
                    if elapsed_time < expected_time
                        % disp("pause")
                        pause(expected_time - elapsed_time - self.dt);
                    end

                end
            end
            % Log the state and input history of the ego vehicle
            if ~isempty(self.ego_vehicle)
                state_log = self.ego_vehicle.state_log; % Update states history
                input_log = self.ego_vehicle.input_log; % Update inputs history
            else
                state_log = [];
                input_log = [];
            end
        end

        %% Function for plot

        function [] = plot_movement_log(self,vehicles, scenarios_config, num_vehicles)
            figure;

            % Define vehicle labels for the legend
            vehicle_labels = arrayfun(@(i) sprintf('Vehicle %d', i), 1:num_vehicles, 'UniformOutput', false);

            % Plot position X history
            subplot(6, 1, 1)
            hold on
            for i = 1:num_vehicles
                plot(0:scenarios_config.dt :scenarios_config.simulation_time, vehicles(i).state_log(1, :),"LineWidth",1)
            end
            title('Position X history')
            ylabel('m')
            % xlabel('s')
            % legend(vehicle_labels)
            hold off

            % Plot position Y history
            subplot(6, 1, 2)
            hold on
            for i = 1:num_vehicles
                plot(0:scenarios_config.dt:scenarios_config.simulation_time, vehicles(i).state_log(2, :),"LineWidth",1)
            end
            title('Position Y history')
            ylabel('m')
            % xlabel('s')
            % legend(vehicle_labels)
            hold off

            % Plot velocity history
            subplot(6, 1, 3)
            hold on
            for i = 1:num_vehicles
                plot(0:scenarios_config.dt:scenarios_config.simulation_time, vehicles(i).state_log(4, :),"LineWidth",1)
            end
            title('Velocity history')
            ylabel('m/s')
            % xlabel('s')
            if scenarios_config.where == "Highway"
                ylim([14, 36])
            elseif scenarios_config.where == "Urban"
                ylim([7, 17])
            end
            legend(vehicle_labels)
            hold off


            % Plot accel history
            subplot(6, 1, 4)
            hold on
            for i = 1:num_vehicles
                plot(0:scenarios_config.dt:scenarios_config.simulation_time, vehicles(i).state_log(5, :),"LineWidth",1)
            end
            title('Acc history')
            ylabel('m/s^2')
            % xlabel('s')
            % ylim([-0.3, 0.3])
            legend(vehicle_labels)
            hold off

            % % Plot yaw history
            % subplot(6, 1, 4)
            % hold on
            % for i = 1:num_vehicles
            %     plot(0:scenarios_config.dt:scenarios_config.simulation_time, vehicles(i).state_log(3, :),"LineWidth",1)
            % end
            % title('Yaw history')
            % ylabel('rad')
            % % xlabel('s')
            % ylim([-0.3, 0.3])
            % legend(vehicle_labels)
            % hold off



            % Plot steering history
            subplot(6, 1, 5)
            hold on
            for i = 1:num_vehicles
                plot(0:scenarios_config.dt:scenarios_config.simulation_time, vehicles(i).input_log(1, :),"LineWidth",1)
            end
            title('Control acc history')
            ylabel('m/s^2')
            % xlabel('s')
            % ylim([-2, 2])
            % legend(vehicle_labels)
            hold off

            % Plot steering history
            subplot(6, 1, 6)
            hold on
            for i = 1:num_vehicles
                plot(0:scenarios_config.dt:scenarios_config.simulation_time, vehicles(i).input_log(2, :),"LineWidth",1)
            end
            title('Steering history')
            ylabel('rad')
            xlabel('s')
            ylim([-0.08, 0.08])
            % legend(vehicle_labels)


            hold off
        end

        function [] = plot_movement_log_simple(self,vehicles, scenarios_config, num_vehicles)
            figure;

            % Define vehicle labels for the legend
            vehicle_labels = arrayfun(@(i) sprintf('Vehicle %d', i), 1:num_vehicles, 'UniformOutput', false);

            % Plot position X history
            subplot(3, 1, 1)
            hold on
            for i = 1:num_vehicles
                plot(0:scenarios_config.dt :scenarios_config.simulation_time, vehicles(i).state_log(1, :),"LineWidth",1)
            end
            title('Position X history')
            ylabel('m')
            grid on
            hold off


            % Plot velocity history
            subplot(3, 1, 2)
            grid on
            hold on
            for i = 1:num_vehicles
                plot(0:scenarios_config.dt:scenarios_config.simulation_time, vehicles(i).state_log(4, :),"LineWidth",1)
            end
            title('Velocity history')
            ylabel('m/s')
            % xlabel('s')
            if scenarios_config.where == "Highway"
                ylim([14, 36])
            elseif scenarios_config.where == "Urban"
                ylim([7, 17])
            end
            legend(vehicle_labels)
            hold off


            % Plot accel history
            subplot(3, 1, 3)
            grid on
            hold on
            for i = 1:num_vehicles
                plot(0:scenarios_config.dt:scenarios_config.simulation_time, vehicles(i).state_log(5, :),"LineWidth",1)
            end
            title('Acc history')
            ylabel('m/s^2')
            % xlabel('s')
            % ylim([-0.3, 0.3])
            legend(vehicle_labels)
            hold off


        end

        %% Function for plotting relative position and speed history
        function [] = plot_relative_movement_log(self, vehicles, scenarios_config, num_vehicles)
            figure;
            title('Real relative State');

            % Define vehicle labels for legend
            vehicle_labels = arrayfun(@(i) sprintf('Vehicle %d - Vehicle %d', i-1 ,i ), 2:num_vehicles, 'UniformOutput', false);

            % Plot relative position X history
            subplot(3, 1, 1)
            hold on
            for i = 2:num_vehicles
                rel_x = vehicles(i-1).state_log(1, :) - vehicles(i).state_log(1, :) ;
                plot(0:scenarios_config.dt:scenarios_config.simulation_time, rel_x,"LineWidth",1)
            end
            title('Relative Position X history')
            ylabel('m')
            legend(vehicle_labels)
            hold off

            % Plot relative position Y history
            subplot(3, 1, 2)
            hold on
            for i = 2:num_vehicles
                rel_y = vehicles(i).state_log(2, :) - vehicles(i-1).state_log(2, :);
                plot(0:scenarios_config.dt:scenarios_config.simulation_time, rel_y,"LineWidth",1)
            end
            title('Relative Position Y history')
            ylabel('m')
            legend(vehicle_labels)
            hold off

            % Plot relative speed history
            subplot(3, 1, 3)
            hold on
            for i = 2:num_vehicles
                rel_speed = vehicles(i).state_log(4, :) - vehicles(i-1).state_log(4, :);
                plot(0:scenarios_config.dt:scenarios_config.simulation_time, rel_speed,"LineWidth",1)
            end
            title('Relative Speed history')
            ylabel('m/s')
            xlabel('s')
            legend(vehicle_labels)
            hold off
        end

        %% Function for plotting relative position and speed history
        function [] = plot_relative_movement_log_simple(self, vehicles, scenarios_config, num_vehicles)
            figure;
            title('Real relative State');

            % Define vehicle labels for legend
            vehicle_labels = arrayfun(@(i) sprintf('Vehicle %d - Vehicle %d', i-1 ,i ), 2:num_vehicles, 'UniformOutput', false);

            % Plot relative position X history
            subplot(3, 1, 1)
            hold on
            for i = 2:num_vehicles
                rel_x = vehicles(i-1).state_log(1, :) - vehicles(i).state_log(1, :) ;
                plot(0:scenarios_config.dt:scenarios_config.simulation_time, rel_x,"LineWidth",1)
            end
            title('Relative Position X history')
            ylabel('m')
            legend(vehicle_labels)
            hold off


            % Plot relative speed history
            subplot(3, 1, 2)
            hold on
            for i = 2:num_vehicles
                rel_speed = vehicles(i).state_log(4, :) - vehicles(i-1).state_log(4, :);
                plot(0:scenarios_config.dt:scenarios_config.simulation_time, rel_speed,"LineWidth",1)
            end
            title('Relative Speed history')
            ylabel('m/s')
            % xlabel('s')
            % legend(vehicle_labels)
            hold off

            % Plot relative speed history
            subplot(3, 1, 3)
            hold on
            for i = 2:num_vehicles
                rel_speed = vehicles(i).state_log(5, :) - vehicles(i-1).state_log(5, :);
                plot(0:scenarios_config.dt:scenarios_config.simulation_time, rel_speed,"LineWidth",1)
            end
            title('Relative Accel history')
            ylabel('$m/s^2$')
            xlabel('Time (s)')
            % legend(vehicle_labels)
            hold off
        end


        %% Function for plotting error global state log
        %%% Plotting Functions
        function plot_error_global_state_log(self, car_1, car_2)
            % Ensure est_global_state_log is not empty
            if isempty(car_1.observer.est_global_state_log) || isempty(car_2.observer.est_global_state_log)
                error('Global state logs are empty. No data to plot.');
            end

            num_vehicles = size(car_1.observer.est_global_state_log, 3); % Number of vehicles
            % num_states = 4; % Assuming X, Y, theta, and V as states
            num_states = size(car_1.observer.est_global_state_log, 1); % Number of states

            state_labels = {'Position X', 'Position Y', 'Theta', 'Velocity','Acc'};

            % Create time vector for plotting
            num_time_steps = size(car_1.observer.est_global_state_log, 2) - 1;
            dt_config = car_1.scenarios_config.dt;
            time_vector = 0:dt_config:(num_time_steps-1)*dt_config;

            figure("Name", "Error Global Position Estimates " + num2str(car_1.vehicle_number) + "and " + num2str(car_2.vehicle_number), "NumberTitle", "off");
            % title(['Global Position Estimates ' num2str(self.vehicle.vehicle_number)]);

            for state_idx = 1:num_states
                subplot(num_states, 1, state_idx);
                hold on;
                for v = 1:num_vehicles
                    plot(time_vector, squeeze(car_1.observer.est_global_state_log(state_idx, 1:end-1, v) - car_2.observer.est_global_state_log(state_idx, 1:end-1, v)), 'DisplayName', ['Vehicle ', num2str(v)]);
                end
                title(state_labels{state_idx});
                xlabel('Time (s)');
                ylabel(state_labels{state_idx});
                legend;
                grid on;
            end
        end

        %% Function for plot
        function plot_ground_error_global_est_ALL(~, collected_car)
            figure("Name", "Error global est ");
            nb_vehicles = length(collected_car);
            state_labels = {'Error Position X', 'Error Velocity','Error Acc'};
            num_states_show = length(state_labels); % Number of states
            
            % Create time vector for plotting
            num_time_steps = size(collected_car(1).observer.est_global_state_log, 2);
            dt_config = collected_car(1).scenarios_config.dt;
            time_vector = dt_config:dt_config:(num_time_steps)*dt_config; % Start from dt since we're using 2:end

            for v_subplot_horizontal = 1:nb_vehicles
                vehicles = collected_car(v_subplot_horizontal);

                for state_idx = [1, 4, 5] % Assuming we want to plot only X, V, and Acc
                    if (state_idx == 1)
                        v_subplot_vertical = 1; % Position X
                    elseif (state_idx == 4)
                        v_subplot_vertical = 2; % Velocity
                    elseif (state_idx == 5)
                        v_subplot_vertical = 3; % Acceleration
                    end
                    % Correct subplot indexing
                    subplot(num_states_show, nb_vehicles, (v_subplot_vertical-1)*nb_vehicles + v_subplot_horizontal);
                    for v_idx = 1:nb_vehicles
                        plot(time_vector, vehicles.observer.est_global_state_log(state_idx, 1:end, v_idx) - collected_car(v_idx).state_log(state_idx, 2:end), 'LineWidth', 1 , 'DisplayName', ['Vehicle ', num2str(v_idx)]);
                        hold on;
                    end
                    ylabel(state_labels{v_subplot_vertical});
                    legend;
                    grid on;
                end
            end

            xlabel('Time (s)');
        end

        function plot_all_trust_log(~, collected_car)
            nb_vehicles = length(collected_car);
            time_steps = size(collected_car(1).trust_log, 2);
            num_cols = ceil(sqrt(nb_vehicles));
            num_rows = ceil(nb_vehicles / num_cols);
            
            % Create time vector for plotting
            dt_config = collected_car(1).scenarios_config.dt;
            time_vector = 0:dt_config:(time_steps-1)*dt_config;

            figure("Name", "All Trust Values Over Time", "NumberTitle", "off");

            for v = 1:nb_vehicles
                subplot(num_rows, num_cols, v);
                hold on;
                for vehicle_idx = 1:nb_vehicles
                    plot(time_vector, squeeze(collected_car(v).trust_log(1, :, vehicle_idx)), ...
                        'DisplayName', ['Vehicle ' num2str(vehicle_idx)], 'LineWidth', 1);
                end
                hold off;
                % ylabel(['Trust Value of V' num2str(v)]);
                title(['Trust Log for V' num2str(v)]);
                if v == 1
                    legend show;
                end
                % Add horizontal reference line at y = 0.5
                yline(0.5, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Reference (0.5)');
                grid on;
            end
            xlabel('Time (s)');

            sgtitle('All Trust Values Over Time');
        end

        function plot_all_trust_log_exclude_attacker(~, collected_car)
            % Simple trust log plot that excludes attacker (vehicle 1)

            nb_vehicles = length(collected_car);
            attacker_id = 1; % Vehicle 1 is the attacker
            num_plots = max(1, nb_vehicles - 1);
            num_cols = ceil(sqrt(num_plots));
            num_rows = ceil(num_plots / num_cols);
            
            % Create main figure
            figure("Name", "Trust Values Over Time (Excluding Attacker)", "NumberTitle", "off");

            subplot_idx = 1;
            
            % Plot trust logs for non-attacker vehicles
            for v = 1:nb_vehicles
                if v ~= attacker_id
                    time_steps = size(collected_car(v).trust_log, 2);
                    
                    % Create time vector for plotting
                    dt_config = collected_car(v).scenarios_config.dt;
                    time_vector = 0:dt_config:(time_steps-1)*dt_config;
                    
                    subplot(num_rows, num_cols, subplot_idx);
                    hold on;
                    for vehicle_idx = 1:nb_vehicles
                        plot(time_vector, squeeze(collected_car(v).trust_log(1, :, vehicle_idx)), ...
                            'DisplayName', ['Vehicle ' num2str(vehicle_idx)], 'LineWidth', 1);
                    end
                    hold off;
                    title(['Trust Log for V' num2str(v)]);
                    legend show;
                    grid on;
                    subplot_idx = subplot_idx + 1;
                end
            end
            
            xlabel('Time (s)');
            sgtitle('Trust Values Over Time (Excluding Attacker)');
        end
    end
end
