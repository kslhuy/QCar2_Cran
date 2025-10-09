classdef Vehicle < handle
    properties
        % Vehicle identity and configuration
        vehicle_number;
        typeController;
        initial_lane_id; % initial lane id of the vehicle
        lane_id; % current lane id of the vehicle
        param; % Parameters of the vehicle
        lanes;
        direction_flag; % 1 stands for changing to the left adjacent lane, 0 stands for keeping the current lane, -1 stands for changing to the right adjacent lane
        acc_flag;
        scenarios_config;
        weight_module;

        % State and control
        state; % current state (X, Y, phi, velocity, acceleration)
        state_log; % history of states
        input; % current input
        input_log; % history of inputs (= to U_final)
        u1_log;
        u2_log;
        u_target_log;
        gamma;
        gamma_log;
        Param_opt; % Optimization parameters for the vehicle

        % Simulation
        dt; % time gap for simulation
        total_time_step; % total time steps for simulation

        % Networking and trust
        other_vehicles;
        num_vehicles;
        trip_models;
        center_communication; % Reference to CenterCommunication object
        graph;
        trust_log;
        other_log;

        % Observer
        observer;

        % Controllers
        controller;
        controller2;
    end
    methods
        function self = Vehicle(vehicle_number, typeController, veh_param, state_initial, initial_lane_id, lanes, direction_flag, acc_flag, scenarios_config, weight_module)
            % Constructor for Vehicle class
            self.scenarios_config = scenarios_config;
            self.dt = scenarios_config.dt;
            self.total_time_step = scenarios_config.simulation_time / self.dt;

            self.vehicle_number = vehicle_number;
            self.typeController = typeController;
            self.param = veh_param;
            self.state = state_initial;
            self.state_log = state_initial;
            self.input = [0; 0]; % initial input
            self.input_log = self.input;
            self.initial_lane_id = initial_lane_id;
            self.lane_id = initial_lane_id;
            self.lanes = lanes;
            self.direction_flag = direction_flag;
            self.acc_flag = acc_flag;
            self.other_log = [initial_lane_id; 1];
            self.weight_module = weight_module;
            self.gamma = 0.5; % initial gamma value
            % Preallocate logs for efficiency (if sizes known)
            self.u1_log = [];
            self.u2_log = [];
            self.u_target_log = [];
            self.gamma_log = [];
        end



        %% Assign the other vehicles to the ego vehicle, and initialize controllers, observer, trust, etc.
        function assign_neighbor_vehicle(self, other_vehicles, controller_goal, typeController_2, center_communication, graph)
            self.graph = graph;
            self.other_vehicles = other_vehicles;

            % Local and global state initialization
            state_initial = self.state;
            inital_local_state = zeros(length(state_initial), 1);


            %-------- To get global state
            inital_global_state = state_initial;
            for i = 1:length(other_vehicles) - 1
                inital_global_state = [inital_global_state , state_initial];
            end
            %--------
            %% Create an observer for the vehicle
            self.observer = Observer(self, self.param ,inital_global_state, inital_local_state);

            %% Create controller

            connected_vehicles_idx = find(self.graph(self.vehicle_number, :) == 1); % Get indices of connected vehicles

            % Optimization parameters
            self.Param_opt = ParamOptEgo(self.dt);

            % Controller 1
            if self.typeController == "None"
                self.controller = [];
            else
                self.controller = Controller(self, controller_goal, self.typeController, self.Param_opt, self.param, self.lanes, connected_vehicles_idx); % Create a controller for the vehicle
            end

            % Controller 2
            if typeController_2 == "None"
                self.controller2 = [];
            else
                self.controller2 = Controller(self, controller_goal, typeController_2, self.Param_opt, self.param, self.lanes, connected_vehicles_idx); % Create a controller for the vehicle
            end

            % Trust and communication setup
            self.num_vehicles = length(self.other_vehicles);
            self.trip_models = arrayfun(@(x) TriPTrustModel(), 1:self.num_vehicles, 'UniformOutput', false); % Trust models
            self.trust_log = zeros(1, self.total_time_step, self.num_vehicles); % Trust log
            self.trust_log(1, 1, :) = 1; % Initial trust is 1 for all vehicles

            % Center communication
            self.center_communication = center_communication; % Initialize the CenterCommunication reference
            self.center_communication.register_vehicle(self); % Register the vehicle with the central communication hub

        end

        function update(self,instant_index)

            % Identify vehicles connected with the ego vehicle
            connected_vehicles_idx = find(self.graph(self.vehicle_number, :) == 1); % Get indices of connected vehicles

            % Calculate trust and opinion scores directly in self.trust_log
            self.calculateTrustAndOpinion3(instant_index, connected_vehicles_idx);

            % Weight trust update
            if (self.scenarios_config.using_weight_trust && instant_index * self.dt >= 3)
                weights_Dis = self.weight_module.calculate_weights_Trust(self.vehicle_number, self.trust_log(1, instant_index, :), "equal");
            else
                weights_Dis = self.weight_module.calculate_weights_Defaut(self.vehicle_number);
            end

            % Update normal car dynamics, like controller and observer
            self.normal_car_update(instant_index, weights_Dis);
            % TODO: update the input log, currently it is updated in the SIMULATOR class
            % self.send_data(instant_index)
        end

        function send_data(self,instant_index)

            %% Send to center communication
            self.center_communication.update_local_state(self.vehicle_number, self.observer.est_local_state_current, instant_index);
            self.center_communication.update_global_state(self.vehicle_number, self.observer.est_global_state_current, instant_index);
            self.center_communication.update_trust(self.vehicle_number, self.trust_log(1, instant_index, :));
            self.center_communication.update_input(self.vehicle_number, self.input);

        end


        function normal_car_update(self,instant_index , weights)
            self.get_lane_id(self.state);

            if self.typeController ~= "None"
                % disp('Controller is not empty');

                %% Controller
                [~, u_1, e_1] = self.controller.get_optimal_input(self.vehicle_number, self.observer.est_local_state_current, self.input, self.lane_id, self.input_log, self.initial_lane_id, self.direction_flag,"true", 0);
                [~, u_2, e_2] = self.controller2.get_optimal_input(self.vehicle_number ,self.observer.est_local_state_current, self.input, self.lane_id, self.input_log, self.initial_lane_id, self.direction_flag, self.scenarios_config.data_type_for_u2, 0);


                if (self.scenarios_config.gamma_type == "max")
                    trust_log_excluded = self.trust_log(1, instant_index, :);
                    trust_log_excluded(self.vehicle_number) = []; % Remove the element at vehicle_number
                    self.gamma = max(trust_log_excluded);
                elseif (self.scenarios_config.gamma_type == "min")
                    self.gamma = min(self.trust_log(1, instant_index, :));
                elseif (self.scenarios_config.gamma_type == "mean") %% = mean
                    trust_log_excluded = self.trust_log(1, instant_index, :);
                    trust_log_excluded(self.vehicle_number) = []; % Remove the element at vehicle_number
                    self.gamma = mean(trust_log_excluded);
                else %% self_belief
                    self.gamma = self.observer.self_belief;
                end

                %% calculate the optimal input of the vehicle
                %using low pass filter to smooth the input

                U_target = [0,0];
                if (self.scenarios_config.controller_type == "local")
                    U_final = u_1 ;
                elseif (self.scenarios_config.controller_type == "coop")
                    U_final = u_2 ;
                else % "mix" case
                    U_target(1) = (1 - self.gamma)*u_1(1) + self.gamma*u_2(1);
                    U_final = u_1;
                    U_final(1) = self.input(1) + self.Param_opt.tau_filter*(U_target(1) - self.input(1)) ; % self.input is last input
                end

                %% Update new state
                if (self.scenarios_config.model_vehicle_type == "delay_v")
                    self.Bicycle_delay_v(self.state, U_final);
                elseif (self.scenarios_config.model_vehicle_type == "delay_a")
                    self.Bicycle_delay_a(self.state, U_final);
                elseif (self.scenarios_config.model_vehicle_type == "normal")
                    self.Bicycle(self.state, U_final);
                else
                    self.Bicycle_no_theta(self.state, U_final); % calculate dX through normal model
                end
                self.input = U_final; % update the input


                %% Observer
                self.observer.Local_observer(self.state , instant_index);
                self.observer.Distributed_Observer(instant_index , weights);

            else
                %% NO Controller = vehicle 1 (lead)

                u_1 = 0;
                u_2 = 0;
                self.gamma = 0;
                
                e_1 = 0;
                e_2 = 0;

                %% We have direct input from the lead vehicle
                acceleration = self.scenarios_config.get_LeadInput(instant_index); % acceleration is the input of the lead vehicle
                U_target = [acceleration;0];
                self.input = U_target ; % update the input

                %% Update new state
                if (self.scenarios_config.model_vehicle_type == "delay_v")
                    self.Bicycle_delay_v(self.state, U_target);
                elseif (self.scenarios_config.model_vehicle_type == "delay_a")
                    self.Bicycle_delay_a(self.state, U_target);
                elseif (self.scenarios_config.model_vehicle_type == "normal")
                    self.Bicycle(self.state, U_target);
                else
                    self.Bicycle_no_theta(self.state, U_target); % calculate dX through normal model
                end

                % %% ---------UPDATE STATE OLD
                % self.state(5) = acceleration; % State 5 is also acceleration 

                % self.state(4) = self.state(4) + acceleration * self.dt; % new speed
                % % speed limit according to different scenarios
                % [ulim, llim] = self.scenarios_config.getLimitSpeed();

                % if self.state(4) >= ulim
                %     self.state(4) = ulim;
                % elseif self.state(4) <= llim
                %     self.state(4) = llim;
                % end
                % dx = self.state(4) * self.dt + 0.5 * acceleration * self.dt^2; %dx=v*dt+0.5*a*dt^2
                % self.state = [self.state(1) + dx; self.state(2); self.state(3); self.state(4) ; self.state(5)]; % new state of normal cars
                % %  no need update input , beacuse the input is constant
                % self.input = [acceleration;0]; % update the input

                % %% ---------UPDATE STATE OLD





                %% Observer
                self.observer.Local_observer(self.state,instant_index);
                self.observer.Distributed_Observer(instant_index, weights);
            end

            %% Log control input
            self.u1_log = [self.u1_log, u_1];
            self.u2_log = [self.u2_log, u_2];
            self.u_target_log = [self.u_target_log, U_target];
            self.gamma_log = [self.gamma_log, self.gamma];

            %% Save the state and input
            self.state_log = [self.state_log, self.state]; % update the state history
            self.input_log = [self.input_log, self.input]; % update the input final history

            self.other_log = [self.other_log(1, :), self.lane_id; self.other_log(2, :), e_1];


        end



        %% Bicycle model

        function Bicycle(self, state, input)

            l_f = self.param.l_f;
            l_r = self.param.l_r;
            l = l_f + l_r;
            [x, y, phi] = self.unpack_state(state);
            [a, delta_f] = self.unpack_input(input); % input is V beacause its aldready convert in calling fucntion
            % v = self.state(4) + self.dt * a;
            v = self.state(4);
            xdot = v * cos(phi); % velocity in x direction
            ydot = v * sin(phi); % velocity in y direction
            phidot = v * tan(delta_f) / l_r; % yaw rate
            vdot = a; % acceleration

            dX = [xdot; ydot; phidot;vdot ; 0];
            self.state = self.state + self.dt*dX ; % update the state of the vehicle
            self.input = input; % update the input
            self.state(5) = a; % just put the acceleration in the state
        end

        function  Bicycle_delay_v(self, state, input)

            l_f = self.param.l_f;
            l_r = self.param.l_r;
            l = l_f + l_r;

            [x, y, phi , v] = self.unpack_state_v2(state);
            [a, delta_f] = self.unpack_input(input); % input is V beacause its aldready convert in calling fucntion
            v_dot =  (1/self.param.tau_v)*a - (1/self.param.tau_v)*v;

            xdot = v * cos(phi); % velocity in x direction
            ydot = v * sin(phi); % velocity in y direction
            phidot = v * tan(delta_f) / l_r; % yaw rate
            dX = [xdot; ydot; phidot ; v_dot ; 0];
            self.state = self.state + self.dt .* dX;

            self.input = input; % update the input
            self.state(5) = a; % just put the acceleration in the state

        end

        function Bicycle_delay_a(self, state , input)
            % Unpack parameters
            l_f = self.param.l_f;
            l_r = self.param.l_r;
            l = l_f + l_r;
            tau = self.param.tau;

            [x, y, phi, v] = self.unpack_state_v2(state(1:4));

            theta = 0;
            A = [   0 0 -v*sin(theta) cos(theta) 0;
                0 0 v*cos(theta) sin(theta) 0;
                0 0 0 0 0;
                0 0 0 0 1;
                0 0 0 0 -1/tau];


            B = [0 0;
                0 0;
                0 1;
                0 0;
                1/tau 0];


            a = state(5);
            u_a = input(1); % Jerk
            delta_f = input(2); % Front steering angle

            % xdot = v * cos(phi);
            % ydot = v * sin(phi);
            % phidot = v * tan(delta_f) / l;
            % vdot = a;
            % adot = (1/tau) * u_a - (1/tau) * a;

            % dX = [xdot; ydot; phidot; vdot; adot];

            dX = A*self.state + B*input;



            self.state = self.state + self.dt * dX;
            self.input = input;
        end

        function Bicycle_no_theta(self, state , input)
            % Unpack parameters
            l_f = self.param.l_f;
            l_r = self.param.l_r;
            l = l_f + l_r;
            tau = self.param.tau;

            [x, y, phi, v] = self.unpack_state_v2(state(1:4));

            Ai_conti = [0 0 0 1 0;
                0 0 0 0 0;
                0 0 0 0 0;
                0 0 0 0 1;
                0 0 0 0 -1/tau];
            Bi_conti = [0 0;
                0 0;
                0 0;
                0 0;
                1/tau 0];
            % Discretization of the state-space model
            A = (eye(length(Ai_conti))+self.dt*Ai_conti);
            B = self.dt*Bi_conti;

            self.state = A*self.state + B * self.input;
            self.input = input;



        end

        function [] = get_lane_id(self, state)
            if state(2) <= self.lanes.lane_width - 0.5 * self.param.width
                self.lane_id = 1;
            else if state(2) <= self.lanes.lane_width + 0.5 * self.param.width
                    self.lane_id = 1.5;
            else if state(2) <= 2 * self.lanes.lane_width - 0.5 * self.param.width
                    self.lane_id = 2;
            else if state(2) <= 2 * self.lanes.lane_width + 0.5 * self.param.width
                    self.lane_id = 2.5;
            else if state(2) <= 3 * self.lanes.lane_width - 0.5 * self.param.width
                    self.lane_id = 3;
            else if state(2) <= 3 * self.lanes.lane_width + 0.5 * self.param.width
                    self.lane_id = 3.5;
            else if state(2) <= 4 * self.lanes.lane_width - 0.5 * self.param.width
                    self.lane_id = 4;
            end
            end
            end
            end
            end
            end
            end
        end

        function [speed, beta] = unpack_input(self, input)
            speed = input(1);
            beta = input(2);
        end

        function [x, y, phi] = unpack_state(self, state)
            x = state(1);
            y = state(2);
            phi = state(3);
        end

        function [x, y, phi,v] = unpack_state_v2(self, state)
            x = state(1);
            y = state(2);
            phi = state(3);
            v = state(4);
        end






        function calculateTrustAndOpinion3(self, instant_index, connected_vehicles_num)

            %% Step 1: Calculate trust for direct vehicles
            received_trusts = []; % Store received trust scores of all connected vehicles
            received_vehicles = []; % Store vehicle indices providing trust scores
            ego_trust_in_connected = []; % Store ego vehicle's trust in connected vehicles

            %% Get all the vehicles that are connected to the ego vehicle
            connected_vehicles = [];
            for v = connected_vehicles_num
                connected_vehicles = [connected_vehicles; self.other_vehicles(v)];
            end

            for vehicle_id = connected_vehicles_num
                %% Identify vehicles directly measurable by the ego vehicle's sensor
                % that is cloest to the ego vehicle
                if abs(vehicle_id - self.vehicle_number) == 1
                    %% Calculate trust for direct vehicle
                    [self.trust_log(1, instant_index, vehicle_id), ~, ~, ~] = ...
                        self.trip_models{vehicle_id}.calculateTrust(...
                        self, self.other_vehicles(vehicle_id), self.other_vehicles(1), connected_vehicles,true, instant_index);
                else
                    % for other connected vehicles
                    [self.trust_log(1, instant_index, vehicle_id), ~, ~, ~] = ...
                        self.trip_models{vehicle_id}.calculateTrust(...
                        self, self.other_vehicles(vehicle_id), self.other_vehicles(1), connected_vehicles,false, instant_index);

                end

                %% Store received trust values and ego's trust for opinion calculation
                trust_score = self.center_communication.get_trust_score(vehicle_id);

                % TODO : Index here is not right , need to change later
                ego_trust_in_connected = [ego_trust_in_connected; self.trust_log(1, instant_index, vehicle_id)];

                if ~isempty(trust_score)
                    received_trusts = [received_trusts; trust_score];
                    received_vehicles = [received_vehicles; vehicle_id];
                end
            end




            %% Step 2: Calculate opinion-based trust for non-direct vehicles

            % If host vehicle received some data
            if ~isempty(received_trusts)
                for vehicle_id = 1:self.num_vehicles
                    if vehicle_id == self.vehicle_number
                        self.trust_log(1, instant_index, vehicle_id) = 1; % Ego vehicle trust is 1
                        continue;
                    end

                    %% If vehicle_id is not directly connected to the ego vehicle
                    if abs(vehicle_id - self.vehicle_number) ~= 1
                        if (self.scenarios_config.opinion_type == "neighbor_based")
                            opinion_neigbor = neighbor_based_opinion(self,vehicle_id,instant_index,received_trusts,received_vehicles,ego_trust_in_connected);
                            self.trust_log(1, instant_index, vehicle_id) = 0.6*(self.trust_log(1, instant_index, vehicle_id)) + 0.4*opinion_neigbor ;
                        elseif (self.scenarios_config.opinion_type == "distance_based")
                            opinion_distance = distance_based_opinion(self,vehicle_id,instant_index,received_trusts,received_vehicles,ego_trust_in_connected);
                            self.trust_log(1, instant_index, vehicle_id) = 0.6*(self.trust_log(1, instant_index, vehicle_id)) + 0.4*opinion_distance;
                        elseif (self.scenarios_config.opinion_type == "both")
                            opinion_neigbor = neighbor_based_opinion(self,vehicle_id,instant_index,received_trusts,received_vehicles,ego_trust_in_connected);
                            opinion_distance = distance_based_opinion(self,vehicle_id,instant_index,received_trusts,received_vehicles,ego_trust_in_connected);
                            self.trust_log(1, instant_index, vehicle_id) =  0.5*opinion_neigbor + 0.5*opinion_distance ;
                            % self.trust_log(1, instant_index, vehicle_id) = (opinion_neigbor + opinion_distance )/ 2;
                        else
                            %% mix non_nearby trust
                            opinion_neigbor = neighbor_based_opinion(self,vehicle_id,instant_index,received_trusts,received_vehicles,ego_trust_in_connected);
                            opinion_distance = distance_based_opinion(self,vehicle_id,instant_index,received_trusts,received_vehicles,ego_trust_in_connected);
                            self.trust_log(1, instant_index, vehicle_id) = 0.6*(self.trust_log(1, instant_index, vehicle_id)) + 0.2*opinion_neigbor + 0.2*opinion_distance ;
                            % self.trust_log(1, instant_index, vehicle_id) = (opinion_neigbor + opinion_distance )/ 2;
                        end

                    end
                end
            end
        end

        function opinion_score = distance_based_opinion(self,vehicle_id,instant_index,received_trusts,received_vehicles,ego_trust_in_connected)
            opinion_score = 0;

            received_trusts_vehicle = received_trusts(:, vehicle_id); % Trust scores for vehicle_id
            valid_indices = received_vehicles ~= vehicle_id;

            new_received_trusts_vehicle = received_trusts_vehicle(valid_indices);
            new_received_vehicles = received_vehicles(valid_indices); %%

            % received_trusts_vehicle = received_trusts(:,vehicle_id); % Trust scores for vehicle_id
            % non_zero_trusts = received_trusts_vehicle(received_trusts_vehicle ~= 0); % Filter zeros
            % non_zero_and_one_trusts = non_zero_trusts(non_zero_trusts ~= 1); % Filter ones

            % new_received_vehicles = received_vehicles(received_trusts_vehicle ~= 0); % Filter zeros
            % new_received_vehicles = new_received_vehicles(non_zero_trusts ~= 1); % Filter ones

            if ~isempty(new_received_trusts_vehicle)
                %% Opinion-based trust aggregation (Design 1)
                opinion_weights = 1 ./ (1 + abs(new_received_vehicles - vehicle_id)); % Weight by distance
                normalized_weights = opinion_weights / sum(opinion_weights); % Normalize weights
                opinion_score = sum(normalized_weights .* new_received_trusts_vehicle, 1); % Weighted sum
            else
                opinion_score = 0; % Default to zero if no valid trust scores
            end

        end



        function opinion_score = neighbor_based_opinion(self,vehicle_id,instant_index,received_trusts,received_vehicles,ego_trust_in_connected)
            opinion_score = 0;

            received_trusts_vehicle = received_trusts(:, vehicle_id); % Trust scores for vehicle_id
            % valid_indices = received_trusts_vehicle ~= 0 & received_trusts_vehicle ~= 1;
            valid_indices = received_vehicles ~= vehicle_id;

            new_received_trusts_vehicle = received_trusts_vehicle(valid_indices);
            new_received_vehicles = received_vehicles(valid_indices); %%
            weights = ego_trust_in_connected(valid_indices); %% look the trust in host vehicle ,

            if ~isempty(new_received_trusts_vehicle)
                %% Use trust-based weights instead of distance-based
                if sum(weights) > 0
                    normalized_weights = weights / length(weights); % Normalize trust-based weights
                    opinion_score = sum(normalized_weights .* new_received_trusts_vehicle); % Weighted sum
                else
                    opinion_score = 0; % Default to zero if no valid weights
                end
            else
                opinion_score = 0; % Default to zero if no valid trust scores
            end

        end




        %% Function for plot
        function plot_ground_truth_vs_estimated(self)
            figure;
            subplot(3,1,1);
            % plot( self.state_log(1,:), 'b', 'LineWidth', 1.5); hold on;
            % plot( self.observer.est_global_state_log(1, 1:end-1, self.vehicle_number), 'r--', 'LineWidth', 1.5);
            % legend('Ground Truth X', 'Estimated X');

            plot( self.observer.est_global_state_log(1, 1:end, self.vehicle_number) - self.state_log(1,1:end-1), 'b', 'LineWidth', 1.5);

            legend('error X');

            title(['Comparison global est car' num2str(self.vehicle_number) 'state with local true ' ]);

            subplot(3,1,2);
            % plot( self.state_log(2,:), 'b', 'LineWidth', 1.5); hold on;
            % plot( self.observer.est_global_state_log(2, 1:end-1, self.vehicle_number), 'r--', 'LineWidth', 1.5);
            % legend('Ground Truth Y', 'Estimated Y');


            plot( self.observer.est_global_state_log(2, 1:end, self.vehicle_number)-self.state_log(2,1:end-1), 'b', 'LineWidth', 1.5);
            legend('error Y');
            % title('Y Position Comparison');

            subplot(3,1,3);
            plot( self.state_log(4,:), 'b', 'LineWidth', 1.5); hold on;
            plot( self.observer.est_global_state_log(4, 1:end-1, self.vehicle_number), 'r--', 'LineWidth', 1.5);
            legend('Ground Truth Speed', 'Estimated Speed');
            % title('Speed Comparison');
            xlabel('Time (s)');
        end



        %% Function for plot
        function plot_ground_error_global_est(self , vehicles)
            figure("Name", "Error global est " + num2str(self.vehicle_number), "NumberTitle", "off");
            nb_vehicles = length(vehicles);
            state_labels = {'Error Position X', ' Error Position Y', 'Error Theta', 'Error Velocity','Error Acc'};
            num_states = size(self.observer.est_global_state_log, 1); % Number of states


            for state_idx = 1:num_states
                subplot(num_states, 1, state_idx);
                for v = 1:nb_vehicles
                    plot( self.observer.est_global_state_log(state_idx, 1:end, v) - vehicles(v).state_log(state_idx, 1:end-1), 'LineWidth', 1 , 'DisplayName', ['Vehicle ', num2str(v)]);
                    % plot(vehicles(v).state_log(state_idx, 1:end-1), 'DisplayName', ['Vehicle ', num2str(v)]);
                    hold on;
                    % plot(self.observer.est_global_state_log(state_idx, 1:end, v), 'DisplayName', ['Vehicle ', num2str(v)]);
                    ylim([-1, 1]); % Set y-axis limits for better visibility
                end
                title(state_labels{state_idx});
                % xlabel('Time (s)');
                ylabel(state_labels{state_idx});
                legend;
                grid on;
            end


            % subplot(4,1,1);

            % vehicle_labels = arrayfun(@(i) sprintf('Vehicle %d', i), 1:nb_vehicles, 'UniformOutput', false);

            % for i = 1:nb_vehicles
            %     plot( self.observer.est_global_state_log(1, 1:end, i) - vehicles(i).state_log(1, 1:end-1), 'LineWidth', 1);
            %     hold on;
            % end

            % % % Assuming Is_ok is a logical array with the same time steps as the logs
            % % is_not_ok = ~self.observer.Is_ok_log;
            % % bad_time_steps = find(is_not_ok); % Indices where Is_ok is true

            % legend(vehicle_labels);

            % title(['Comparison global est state with local true platoon car ' num2str(self.vehicle_number)]);

            % % % Add vertical lines where Is_ok is false
            % % for t = bad_time_steps
            % %     xline(t, '--r', 'LineWidth', 1.2, 'Alpha', 0.3); % Red dashed line
            % % end


            % subplot(4,1,2);

            % for i = 1:nb_vehicles
            %     plot( self.observer.est_global_state_log(2, 1:end, i) - vehicles(i).state_log(2, 1:end-1), 'LineWidth', 1);
            %     hold on;
            % end
            % % legend('error Y');

            % subplot(4,1,3);

            % for i = 1:nb_vehicles
            %     plot( self.observer.est_global_state_log(3, 1:end, i) - vehicles(i).state_log(3, 1:end-1), 'LineWidth', 1);
            %     hold on;
            % end

            % subplot(4,1,4);

            % for i = 1:nb_vehicles
            %     plot( self.observer.est_global_state_log(4, 1:end, i) - vehicles(i).state_log(4, 1:end-1), 'LineWidth', 1);
            %     hold on;
            % end

            xlabel('Time (s)');
        end

        %% Function for plot
        function plot_u1_u2_gamma(self)
            figure("Name", "Controller " + num2str(self.vehicle_number), "NumberTitle", "off");

            subplot(5,1,1);

            plot( self.u1_log(1,:), 'r', 'LineWidth', 1.5);

            legend('U1');

            title(["car " num2str(self.vehicle_number)]);

            subplot(5,1,2);

            plot( self.u2_log(1,:), 'b', 'LineWidth', 1.5);
            legend('U2');

            subplot(5,1,3);

            plot( self.u_target_log(1,:), 'b', 'LineWidth', 1.5);
            legend('U target');
            subplot(5,1,4);

            plot( self.input_log(1,1:end-1), 'LineWidth', 1.5);
            legend('U final');

            subplot(5,1,5);
            plot( self.gamma_log,  'LineWidth', 1.5);
            legend('Gamma');
            xlabel('Time (s)');
        end



        function plot_trust_log(self)
            % PLOT_TRUST_LOG Plots the trust values over time for each vehicle.
            %
            % Inputs:
            %   trust_log: 3D array of trust values (1 x time_steps x num_vehicles)
            %   num_vehicles: Number of vehicles (integer)

            % Get the number of time steps
            time_steps = size(self.trust_log, 2);

            % Create a time vector for the x-axis
            time_vector = 1:time_steps;

            % Create a new figure

            figure("Name", "Trust Value " + num2str(self.vehicle_number), "NumberTitle", "off");

            % Plot the trust values for each vehicle
            for vehicle_idx = 1:self.num_vehicles
                hold on;
                plot(time_vector, squeeze(self.trust_log(1, :, vehicle_idx)), 'DisplayName', ['Vehicle ' num2str(vehicle_idx)] , 'LineWidth',1);
            end
            hold off;

            % Add labels and legend
            xlabel('Time Step');
            ylabel(['Trust Value of' num2str(self.vehicle_number)]);
            title(['Trust Values Over Time of' num2str(self.vehicle_number)]);
            legend show;
            grid on;

        end


        function h =  plot_vehicle(self)
            L = self.param.l_fc + self.param.l_rc;
            H = self.param.width;
            theta = self.state(3);
            center1 = self.state(1);
            center2 = self.state(2);
            % Rotation matrix based on the vehicle's orientation
            R = ([cos(theta), -sin(theta); sin(theta), cos(theta)]);
            % Define the corners of the vehicle in its local frame
            X = ([-L / 2, L / 2, L / 2, -L / 2]);
            Y = ([-H / 2, -H / 2, H / 2, H / 2]);

            % Rotate the corners to the global frame
            for i = 1:4
                T(:, i) = R * [X(i); Y(i)];
            end
            % Calculate the coordinates of the vehicle's corners in the global frame

            x_lower_left = center1 + T(1, 1);
            x_lower_right = center1 + T(1, 2);
            x_upper_right = center1 + T(1, 3);
            x_upper_left = center1 + T(1, 4);
            y_lower_left = center2 + T(2, 1);
            y_lower_right = center2 + T(2, 2);
            y_upper_right = center2 + T(2, 3);
            y_upper_left = center2 + T(2, 4);

            % Combine the coordinates into arrays for plotting
            x_coor = [x_lower_left, x_lower_right, x_upper_right, x_upper_left];
            y_coor = [y_lower_left, y_lower_right, y_upper_right, y_upper_left];

            if (self.vehicle_number == 1)
                % color = [0, 1, 0]; % green
                color = [1, 0, 0]; % red
            else
                color = [0, 0, 1]; % blue
                % color = [1, 0, 1]; % magenta
            end

            h = patch('Vertices', [x_coor; y_coor]', 'Faces', [1, 2, 3, 4], 'Edgecolor', color, 'Facecolor', color, 'Linewidth', 1.5);
            % axis equal;
        end

        function calculateTrustAndOpinion(self, instant_index, connected_vehicles_num)
            %% Step 1: Calculate trust for direct vehicles
            received_trusts = []; % Store received trust scores of all direct vehicles
            received_vehicles = []; % Store vehicle indices providing trust scores

            for vehicle_direct = connected_vehicles_num
                %% Identify vehicles directly measurable by the ego vehicle's sensor
                if abs(vehicle_direct - self.vehicle_number) == 1

                    %% Get all the vehicles that are connected to the ego vehicle
                    connected_vehicles = [];
                    for v = connected_vehicles_num
                        connected_vehicles = [connected_vehicles; self.other_vehicles(v)];
                    end

                    %% Calculate trust for direct vehicle
                    [self.trust_log(1, instant_index, vehicle_direct), ~,~, ~, ~, ~] = ...
                        self.trip_models{vehicle_direct}.calculateTrust(...
                        self, self.other_vehicles(vehicle_direct), self.other_vehicles(1),connected_vehicles, self.dt, self.dt);
                end

                %% Store received trust values for opinion calculation
                trust_score = self.center_communication.get_trust_score(vehicle_direct);
                if ~isempty(trust_score)
                    received_trusts = [received_trusts; trust_score];
                    received_vehicles = [received_vehicles; vehicle_direct];
                end
            end

            %% Step 2: Calculate opinion-based trust for non-direct vehicles
            if ~isempty(received_trusts)
                for vehicle_id = 1:self.num_vehicles
                    if vehicle_id == self.vehicle_number
                        self.trust_log(1, instant_index, vehicle_id) = 1; % Ego vehicle trust is 1
                        continue;
                    end

                    %% If trust score is zero, compute opinion-based trust
                    if self.trust_log(1, instant_index, vehicle_id) == 0
                        received_trusts_vehicle = received_trusts(:,vehicle_id); % Trust scores for vehicle_id
                        non_zero_trusts = received_trusts_vehicle(received_trusts_vehicle ~= 0); % Filter zeros
                        non_zero_and_one_trusts = non_zero_trusts(non_zero_trusts ~= 1); % Filter ones

                        new_received_vehicles = received_vehicles(received_trusts_vehicle ~= 0); % Filter zeros
                        new_received_vehicles = new_received_vehicles(non_zero_trusts ~= 1); % Filter ones

                        if ~isempty(non_zero_and_one_trusts)
                            %% Opinion-based trust aggregation (Design 1)
                            opinion_weights = 1 ./ (1 + abs(new_received_vehicles - vehicle_id)); % Weight by distance
                            normalized_weights = opinion_weights / sum(opinion_weights); % Normalize weights
                            opinion_score = sum(normalized_weights .* non_zero_and_one_trusts, 1); % Weighted sum
                        else
                            opinion_score = 0; % Default to zero if no valid trust scores
                        end

                        self.trust_log(1, instant_index, vehicle_id) = opinion_score;
                    end
                end
            end
        end


        function calculateTrustAndOpinion2(self, instant_index, connected_vehicles_num)

            %% Step 1: Calculate trust for direct vehicles
            received_trusts = []; % Store received trust scores of all connected vehicles
            received_vehicles = []; % Store vehicle indices providing trust scores
            ego_trust_in_connected = []; % Store ego vehicle's trust in connected vehicles

            for vehicle_direct = connected_vehicles_num
                %% Identify vehicles directly measurable by the ego vehicle's sensor
                if abs(vehicle_direct - self.vehicle_number) == 1
                    %% Get all the vehicles that are connected to the ego vehicle
                    connected_vehicles = [];
                    for v = connected_vehicles_num
                        connected_vehicles = [connected_vehicles; self.other_vehicles(v)];
                    end

                    %% Calculate trust for direct vehicle
                    [self.trust_log(1, instant_index, vehicle_direct), ~, ~, ~, ~, ~] = ...
                        self.trip_models{vehicle_direct}.calculateTrust(...
                        self, self.other_vehicles(vehicle_direct), self.other_vehicles(1), connected_vehicles, self.dt, self.dt);
                end

                %% Store received trust values and ego's trust for opinion calculation
                trust_score = self.center_communication.get_trust_score(vehicle_direct);
                if ~isempty(trust_score)
                    received_trusts = [received_trusts; trust_score];
                    received_vehicles = [received_vehicles; vehicle_direct];
                    %% Store ego's trust in this connected vehicle (only for direct neighbors)
                    if abs(vehicle_direct - self.vehicle_number) == 1
                        ego_trust_in_connected = [ego_trust_in_connected; self.trust_log(1, instant_index, vehicle_direct)];
                    else
                        %% For non-direct vehicles, append a placeholder (e.g., 0) since trust isn't directly available
                        ego_trust_in_connected = [ego_trust_in_connected; 0];
                    end
                end
            end


            %% Step 2: Calculate opinion-based trust for non-direct vehicles
            if ~isempty(received_trusts)
                for vehicle_id = 1:self.num_vehicles
                    if vehicle_id == self.vehicle_number
                        self.trust_log(1, instant_index, vehicle_id) = 1; % Ego vehicle trust is 1
                        continue;
                    end

                    %% If trust score is zero or not calculated, compute opinion-based trust
                    if self.trust_log(1, instant_index, vehicle_id) == 0 || isnan(self.trust_log(1, instant_index, vehicle_id))
                        received_trusts_vehicle = received_trusts(:, vehicle_id); % Trust scores for vehicle_id
                        valid_indices = received_trusts_vehicle ~= 0 & received_trusts_vehicle ~= 1;
                        non_zero_and_one_trusts = received_trusts_vehicle(valid_indices);
                        new_received_vehicles = received_vehicles(valid_indices);
                        weights = ego_trust_in_connected(valid_indices);

                        if ~isempty(non_zero_and_one_trusts)
                            %% Use trust-based weights instead of distance-based
                            if sum(weights) > 0
                                normalized_weights = weights / sum(weights); % Normalize trust-based weights
                                opinion_score = sum(normalized_weights .* non_zero_and_one_trusts); % Weighted sum
                            else
                                opinion_score = 0; % Default to zero if no valid weights
                            end
                        else
                            opinion_score = 0; % Default to zero if no valid trust scores
                        end

                        self.trust_log(1, instant_index, vehicle_id) = opinion_score;
                    end
                end
            end
        end


        function calculateTrustAndOpinion2_NEW(self, instant_index, connected_vehicles_num)

            %% Step 1: Calculate trust for direct vehicles
            received_trusts = []; % Store received trust scores of all connected vehicles
            received_vehicles = []; % Store vehicle indices providing trust scores
            ego_trust_in_connected = []; % Store ego vehicle's trust in connected vehicles

            for vehicle_direct = connected_vehicles_num
                %% Identify vehicles directly measurable by the ego vehicle's sensor
                if abs(vehicle_direct - self.vehicle_number) == 1
                    %% Get all the vehicles that are connected to the ego vehicle
                    connected_vehicles = [];
                    for v = connected_vehicles_num
                        connected_vehicles = [connected_vehicles; self.other_vehicles(v)];
                    end

                    %% Calculate trust for direct vehicle
                    [self.trust_log(1, instant_index, vehicle_direct), ~, ~, ~, ~, ~] = ...
                        self.trip_models{vehicle_direct}.calculateTrust(...
                        self, self.other_vehicles(vehicle_direct), self.other_vehicles(1), connected_vehicles, self.dt, self.dt);
                end

                %% Store received trust values and ego's trust for opinion calculation
                trust_score = self.center_communication.get_trust_score(vehicle_direct);
                if ~isempty(trust_score)
                    received_trusts = [received_trusts; trust_score];
                    received_vehicles = [received_vehicles; vehicle_direct];
                    %% Store ego's trust in this connected vehicle (only for direct neighbors)
                    if abs(vehicle_direct - self.vehicle_number) == 1
                        ego_trust_in_connected = [ego_trust_in_connected; self.trust_log(1, instant_index, vehicle_direct)];
                    else
                        %% For non-direct vehicles, append a placeholder (e.g., 0) since trust isn't directly available
                        ego_trust_in_connected = [ego_trust_in_connected; 0];
                    end
                end
            end


            %% Step 2: Calculate opinion-based trust for non-direct vehicles

            % If host vehicle received some data
            if ~isempty(received_trusts)
                for vehicle_id = 1:self.num_vehicles
                    if vehicle_id == self.vehicle_number
                        self.trust_log(1, instant_index, vehicle_id) = 1; % Ego vehicle trust is 1
                        continue;
                    end

                    %% If vehicle_id is not directly connected to the ego vehicle
                    if abs(vehicle_id - self.vehicle_number) ~= 1
                        opinion_neigbor = neighbor_based_opinion(self,vehicle_id,instant_index,received_trusts,received_vehicles,ego_trust_in_connected);
                        self.trust_log(1, instant_index, vehicle_id) = opinion_neigbor ;
                    end
                end
            end
        end



    end
end