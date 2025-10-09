classdef TriPTrustModel < handle


    %{
    Overview how of this ChatGPT
  https://discord.com/channels/1123389035713400902/1326223031667789885/1327306982218141727
    %}

    properties
        wv = 12;  % Weight for velocity
        wd = 2;  % Weight for distance
        wa = 0.3;  % Weight for acceleration
        wj = 1.0;  % Weight for jerkiness
        wh = 1.0;  % Weight for heading


        wv_nearby = 12;  % Weight for velocity
        wd_nearby = 8;  % Weight for distance
        wa_nearby = 0.3;  % Weight for acceleration
        wj_nearby = 1.0;  % Weight for jerkiness
        wh_nearby = 1.0;  % Weight for heading

        wt = 0.1; % Trust decay weight
        wt_global = 0.5; % Trust decay weight


        C = 0.2;   % Regularization constant
        tacc = 1.2;% Trust-based acceleration scaling factor
        k = 5;     % Number of trust levels
        rating_vector; % Trust rating vector
        rating_vector_global;

        %% Score components
        alpha_v_score = 0.7; % Weight for velocity score

        % New properties for global estimate checks
        sigma2 = 1; % Sensitivity parameter for cross-validation trust factor
        tau2 = 0.5;   % Sensitivity parameter for local consistency trust factor
        last_d;
        last_time_d = 0; % Last time step when distance was updated
        Period_a_score_distane = 10; % Time step for the simulation
        buffer_size = 50; % Number of time steps for moving average (e.g., n=5)
        distance_buffer ; % Initialize buffer

        trust_sample_log;
        gamma_cross_log;
        gamma_local_log;
        gamma_expected_log;
        v_score_log;
        d_score_log;
        a_score_log;
        h_score_log;
        beacon_score_log;
        final_score_log;

        flag_glob_est_check_log ;
        flag_taget_attk_log ;
        flag_local_est_check_log ;

        flag_taget_attk = false;
        flag_glob_est_check = false;

        flag_local_est_check = false;
        lead_state_lastest = [];
        lead_state_lastest_timestamp = 0;

        D_pos_log = [];          % Log for position discrepancies
        D_vel_log = [];          % Log for velocity discrepancies
        anomaly_pos_log = [];    % Log for position anomaly flags
        anomaly_vel_log = [];    % Log for velocity anomaly flags
        anomaly_gamma_log = [];  % Already included from previous request

        % Parameters for anomaly detection
        w = 10;  % Sliding window size
        Threshold_anomalie  = 3;   % Threshold for cumulative anomalies
        reduce_factor = 0.5;  % Trust reduction factor

        previous_state;
        tau2_matrix_gamma_local = []; % Diagonal matrix for local consistency factor
        sigma2_matrix_gamma_cross = []; % Diagonal matrix for cross-host trust factor

        v_rel_log = []; % Log for relative velocity
        d_add_log = []; % Log for additional distance
        acc_rel_log = []; % Log for relative acceleration
        delta_acc_expected_log = []; % Log for expected acceleration difference
        distance_log = []; % Log for distance measurements

        scale_d_expected_log = []; % Scale factor for expected distance
        delta_d_log = []; % Log for delta distance (measured - epxected )

        a_score_expected_diff_acc_log = []; % Log for expected acceleration score
        a_score_diff_acc_log = []; % Log for acceleration difference score
        a_score_vrel_dis_log = []; % Log for relative velocity and distance score
        a_score_defaut_log = []; % Log for default acceleration score



    end

    methods
        function self = TriPTrustModel()

            % Initialize trust rating vector R_y is accumulates all past outcomes element of vector r_y^x
            % R_y = [R_y(1) ,R_y(2), R_y(3) , R_y(4) , R_y(5)]
            % 1 2 3 4 5 is each trust level

            self.rating_vector = zeros(1, self.k);
            self.rating_vector(5) = 1;

            % for global bal
            self.rating_vector_global = zeros(1, self.k);
            self.rating_vector_global(5) = 1;


            self.last_d = 20;
            self.last_time_d = 0; % Last time step when distance was updated

            self.buffer_size = 5; % Number of time steps for moving average (e.g., n=5)
            self.distance_buffer = zeros(1, self.buffer_size); % Initialize buffer

            % Initialize logs for trust samples and scores
            self.trust_sample_log = [];
            self.gamma_cross_log = [];
            self.gamma_local_log = [];
            self.v_score_log = [];
            self.d_score_log = [];
            self.a_score_log = [];
            self.beacon_score_log = [];
            self.final_score_log = [];

            self.lead_state_lastest = zeros(4,1);
            self.previous_state = zeros(4,1);

            tau2_diag_element = [1.5 , 0.5];
            self.tau2_matrix_gamma_local = diag(tau2_diag_element);

            % Compute covariance matrix (adaptive variance)
            sigma2_diag_element = [1.5, 1 ,0.01, 0.5 , 0.1];
            self.sigma2_matrix_gamma_cross = diag(sigma2_diag_element);


            self.v_rel_log = [];
            self.acc_rel_log = [];
            self.d_add_log = [];
            self.delta_acc_expected_log = [];



            % % Parameters for anomaly detection
            % self.w = 10;  % Sliding window size
            % self.Threshold_anomalie  = 3;   % Threshold for cumulative anomalies
            % self.reduce_factor = 0.5;  % Trust reduction factor
        end

        function v_score_final_with_exp = evaluate_velocity(self,host_id , target_id, v_y, v_host, v_leader, a_leader, b_leader , is_nearby ,tolerance)
            if nargin < 9
                tolerance = 0.1; % Default tolerance if not provided
            end

            % Get current leader velocity ,
            v_ref = v_leader + b_leader * a_leader;


            %{
            Evaluate velocity with a tolerance range for minor deviations.

            Parameters:
                v_y: Reported velocity of the follower vehicle.
                v_leader: Leader's velocity.
                a_leader: Leader's acceleration.
                b_leader: Beacon interval (time since the last beacon).
                tolerance: Acceptable fraction of v_ref deviation without penalty.

            Returns:
                Velocity trust score (0 to 1).

            Explain in https://discord.com/channels/1123389035713400902/1326223031667789885/1327291670911254528
            %}
            if (host_id - target_id) > 0 % Target is ahead host
                alpha = self.alpha_v_score;
            else
                alpha = 1- self.alpha_v_score; % Target is follwing host
            end

            % Meaning that leader is move back (brake) or stop
            if v_ref == 0 || v_leader == 0 || sign(v_ref*v_leader) < 0 || sign(v_ref*v_host) < 0
                % Handle edge case for v_ref <= 0
                % TODO : why abs(v_y)
                % Exemple v_y positive , try to run , so score is 0
                % v_y negative , try to brake , so is ?????
                v_score_final =  max(1 - abs(v_y), 0);

                return;
            end
            % Calculate absolute deviation as a fraction of reference velocity
            % meaning report vehicle is faster or slower than reference velocity
            deviation_ref = (abs(v_y - v_ref) + 1) / v_ref;
            deviation_host = abs(v_y - v_host) / v_host;

            if deviation_ref <= tolerance
                % Within tolerance, give max score
                v_score_ref = 1.0;
            else
                % Scale penalty for deviations beyond tolerance
                scaled_penalty = (deviation_ref - tolerance) / (1 - tolerance);
                v_score_ref =  max(1 - scaled_penalty, 0);
            end

            v_score_host = max(1 - deviation_host, 0);

            v_score_final = (1-alpha)*v_score_ref +  (alpha)* v_score_host ;
            if (is_nearby)
                v_score_final_with_exp = v_score_final^self.wv_nearby ;
            else
                v_score_final_with_exp = v_score_final^self.wv ;
            end

        end

        function d_score = evaluate_distance(self, d_y, d_measured , is_nearby)
            if is_nearby
                % For nearby vehicles, use a stricter distance evaluation
                d_score = (max(1 - abs((d_y - d_measured)/d_measured), 0))^self.wd_nearby;
            else
                % For distant vehicles, allow more tolerance
                % TODO : need to change the distance measure for the far away vehicle
                % d_y is the reported distance , d_measured is the measured distance
                % so if the reported distance is close to the measured distance, then score is high
                % d_score = 0;
                d_score = max(1 - abs((d_y - d_measured)/d_measured), 0)^self.wd;
            end
        end

        function a_score = evaluate_acceleration(self,host_vehicle, host_id,target_id,a_y, a_host, d, ts, is_nearby)
            % Update distance buffer
            self.distance_buffer = [d(1), self.distance_buffer(1:end-1)]; % Shift and add new distance



            % Compute relative velocity using distance difference over n time steps
            %% Relative acceleration calculation
            if all(self.distance_buffer ~= 0) % Ensure buffer is filled
                v_rel = (self.distance_buffer(end-1) - self.distance_buffer(1)) / ((self.buffer_size-1) * ts);
                v_rel_m1 = (self.distance_buffer(end) - self.distance_buffer(2)) / ((self.buffer_size-1) * ts);
                expected_a_relative_diff  = (v_rel - v_rel_m1)/(ts);
            else
                % v_rel = 0; % Default if buffer not yet filled
                v_rel = diff(d) / ts;
                expected_a_relative_diff = 0;
            end
            d_norm = 19; % Normalization factor for distance

            %% Calculate the relative acceleration difference
            a_relative_diff = a_y - a_host;

            diff_rel_acc = expected_a_relative_diff - a_relative_diff;

            %% Test another writing
            a_y_expected = a_host + expected_a_relative_diff; % Expected acceleration difference
            delta_acc = a_y_expected - a_y;
            %%%


            constance_regulation = 1; % depending on the sign of the acceleration difference , and position index


            if  ~is_nearby
                d_add = d_norm*abs(target_id-host_id)  / (d(1));
            else
                % id_diff = host_id - target_id;
                % 0.9*(target_id-1)
                if target_id > 1
                    % Calculate sign mismatch
                    Signe_mismatch_relative = (sign(a_host) ~= sign(a_relative_diff));
                    % Update constance regulation based on sign mismatch
                    if (Signe_mismatch_relative == 0)
                        % Sign consistent
                        constance_regulation = 0.7;
                    else
                        % Sign inconsistent
                        constance_regulation = 0.8;
                    end

                    % Calculate additional distance based on target and host IDs
                    d_add = d_norm*constance_regulation / (d(1));
                    % if (target_id > host_id)
                    %     d_add = d_norm*constance_regulation / (d(1));
                    % else
                    %     d_add = d_norm / (d(1));
                    % end
                else
                    % special case : target_id <= 1 and host_id = 2
                    d_add = d_norm*constance_regulation / (d(1));
                end


                % d_add = 1; % Default value for distant vehicles
            end


            %%% Calculate distance expected depending  Host ID , gamma mix distance
            hi = host_vehicle.Param_opt.hi ; %% Time gap ( T )
            ri = host_vehicle.Param_opt.ri; % Minimum gap distance ( s0 )
            v_local_host = host_vehicle.observer.est_local_state_current(4);
            v_0 = host_vehicle.Param_opt.v0; % Desired velocity in free flow
            delta = host_vehicle.Param_opt.delta;
            if host_vehicle.scenarios_config.controller_type == "mix"
                s_CACC_expected = ri + hi * v_local_host/(host_id-1); % Expected spacing based on CACC model
                s_IDM_expected  = ri + hi * v_local_host / (sqrt(1 - (v_local_host/v_0)^delta));
                gamma_control = host_vehicle.gamma; % Control parameter for Mix controller
                d_expected = (1 - gamma_control)*s_CACC_expected + (gamma_control) * s_IDM_expected;

            elseif host_vehicle.scenarios_config.controller_type == "local"
                d_expected = (ri + hi*v_local_host) /(sqrt(1 - (v_local_host/v_0)^delta ));

            else %"coop"
                d_expected = ri + hi * v_local_host/(host_id-1); % Expected spacing based on CACC model
            end

            % Compute expected spacing using the mixing formula:

            scale_d_expected = d(1) /d_expected; % Normalize expected distance by current distance

            %%% NEW: Adjust d_add based on scenario flags



            delta_d = d(1) - d_expected;

            % Define scenario flags
            target_decelerating = a_y < 0;
            host_decelerating = a_host < 0;
            both_accelerating = a_y > 0 && a_host > 0;
            both_decelerating = a_y < 0 && a_host < 0;
            target_acc_host_dec = a_y > 0 && a_host < 0;

            % Initialize d_add_scale
            d_add_scale = 1;

            % Apply scenario-based adjustments to d_add_scale
            if target_decelerating
                d_add_scale = d_add_scale * 0.8; % Penalize when target is decelerating
            end
            if both_decelerating && delta_d < 0
                d_add_scale = d_add_scale * 0.7; % Stronger penalty when both are decelerating and distance is too small
            end
            if target_acc_host_dec && delta_d < 0
                d_add_scale = d_add_scale * 0.8; % Penalize when target is accelerating, host is decelerating, and distance is too small
            end
            if (both_accelerating || both_decelerating) && abs(delta_d) > 0.2 * d_expected
                d_add_scale = d_add_scale * 0.9; % Mild penalty for significant distance deviation
            end

            % Compute adjusted d_add
            d_add_adjusted = d_add * d_add_scale;


            %%% Calculate the acceleration score

            %% Using the relative acceleration difference
            % a_score = max(1 - abs( d_add * diff_rel_acc), 0);

            %% Using the expected acceleration difference
            a_score_expected_diff_acc = max(1 - abs( d_add_adjusted * delta_acc), 0);

            %% Using the acceleration difference
            a_score_diff_acc =  max(1 - abs(v_rel /d_add_adjusted * a_relative_diff), 0);
            a_score_vrel_dis = max(1 - abs(v_rel / d(1) * a_relative_diff), 0);
            a_score_defaut = max(1 - abs(v_rel / ((self.buffer_size - 1)  * ts) * a_relative_diff), 0);


            a_score = a_score_expected_diff_acc;

            % Apply weighting based on proximity
            if is_nearby
                a_score = a_score^self.wa_nearby; % Tune wa_nearby
            else
                a_score = a_score^self.wa; % Tune wa
            end


            %log
            self.v_rel_log = [self.v_rel_log, v_rel]; % Log relative velocity
            self.distance_log = [self.distance_log, d(1)]; % Log distance
            self.d_add_log = [self.d_add_log, d_add_adjusted]; % Log additional distance
            self.acc_rel_log = [self.acc_rel_log, a_relative_diff]; % Log relative acceleration
            self.delta_acc_expected_log = [self.delta_acc_expected_log, delta_acc]; % Log expected acceleration difference
            self.scale_d_expected_log = [self.scale_d_expected_log, scale_d_expected]; % Log scale factor for expected distance
            self.delta_d_log = [self.delta_d_log, delta_d]; % Log delta distance (measured - expected)

            self.a_score_expected_diff_acc_log = [self.a_score_expected_diff_acc_log, a_score_expected_diff_acc];
            self.a_score_diff_acc_log = [self.a_score_diff_acc_log, a_score_diff_acc];
            self.a_score_vrel_dis_log = [self.a_score_vrel_dis_log, a_score_vrel_dis];
            self.a_score_defaut_log = [self.a_score_defaut_log, a_score_defaut];
        end

        function j_score = evaluate_jerkiness(~, j_y, j_thresh)
            if nargin < 3
                j_thresh = 2.0;
            end
            if abs(j_y) > j_thresh
                j_score = min(j_thresh / abs(j_y), 1);
            else
                j_score = 1;
            end
        end

        function beacon_score = evaluate_beacon_timeout(~, beacon_received)
            if (beacon_received)
                beacon_score = 1;
            else
                beacon_score = 0;
            end
        end



        function h_score = evaluate_heading(self, target_pos_X, target_pos_Y, reported_heading, instant_idx)
            % evaluate_heading - Computes trust score for target's reported heading
            %
            % Inputs:
            %   host_vehicle       - Host vehicle object
            %   target_vehicle     - Target vehicle object
            %   target_id          - Target vehicle ID
            %   host_id            - Host vehicle ID
            %   target_pos_X       - Target's current X position
            %   target_pos_Y       - Target's current Y position
            %   reported_heading   - Target's reported heading (radians)
            %   instant_idx        - Current simulation time step
            %
            % Output:
            %   h_score            - Heading trust score (0 to 1)

            % Retrieve previous position from target's state history
            if isequal(self.previous_state, zeros(4,1))
                h_score = 1; % Cannot compute heading without previous position
                self.previous_state = [target_pos_X, target_pos_Y, 0, 0]; % Initialize with current position
                return;
            else

                % Extract previous position
                prev_target_pos_X = self.previous_state(1);
                prev_target_pos_Y = self.previous_state(2);

                % Step 2: Calculate estimated heading (theta_est)
                delta_Y = target_pos_Y - prev_target_pos_Y;
                delta_X = target_pos_X - prev_target_pos_X;
                theta_est = atan2(delta_Y, delta_X); % Estimated heading in radians

                % Step 3: Compare with reported heading
                theta_diff = abs(reported_heading - theta_est);
                % Ensure smallest angle difference (handle circular nature of angles)
                theta_diff = min(theta_diff, 2 * pi - theta_diff);

                % Step 4: Compute trust value
                theta_max = pi / 18; % 10 degrees threshold
                h_score = max(1 - theta_diff / theta_max, 0);
                self.previous_state = [target_pos_X, target_pos_Y, 0, 0]; % Initialize with current position

            end
        end

        %%% Trust sample calculation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   %%%%%%%%%%%%%%%%%%%
        function trust_sample = calculate_trust_sample_wo_Acc(self, v_score, d_score, a_score, beacon_score,h_score,is_nearby)
            trust_sample = beacon_score * (v_score) * (d_score) ;
        end

        function trust_sample = calculate_trust_sample_w_Jek(self, v_score, d_score, a_score, j_score, beacon_score)
            trust_sample = beacon_score * (v_score^self.wv) * (d_score^self.wd) * (a_score^self.wa) * (j_score^self.wj);
        end

        function trust_sample = calculate_trust_sample_normal(self, v_score, d_score, a_score, beacon_score,h_score,is_nearby)
            if (is_nearby)
                trust_sample = beacon_score * (v_score^self.wv_nearby) * (d_score^self.wd_nearby) * h_score^self.wh_nearby;
            else
                trust_sample = beacon_score * (v_score^self.wv)* (d_score^self.wd) ;
            end
        end

        function trust_sample = calculate_trust_sample(self, v_score, d_score, a_score, beacon_score,h_score,is_nearby)
            % Calculate trust sample based on the provided scores and beacon status
            trust_sample = beacon_score * (v_score) * (d_score) * (a_score) ;
        end

        %%% Trust rating vector update %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   %%%%%%%%%%%%%%%%%%%

        function update_rating_vector(self, trust_sample , type)
            %  Map trust sample to a specific trust level in the rating vector

            trust_level = min(round(trust_sample * (self.k - 1) + 1)  , (self.k) );
            trust_vector = zeros(1, self.k); % trust_vector = r_y^x
            trust_vector(trust_level) = 1;

            % Compute current trust score (sigma_y) to use in lambda_y calculation

            if (type == "local")
                current_trust_score = self.calculate_trust_score( self.rating_vector);
                lambda_y = current_trust_score * self.wt;
            else
                current_trust_score = self.calculate_trust_score(self.rating_vector_global);
                lambda_y = current_trust_score * self.wt_global;
            end

            % Define lambda_y as per equation (9): lambda_y = sigma_y * w_t

            % Update the rating vector (R_y) using the aging factor lambda_y
            if type == "local"
                self.rating_vector = (1 - lambda_y) * self.rating_vector + trust_vector;
            else
                self.rating_vector_global = (1 - lambda_y) * self.rating_vector_global + trust_vector;
            end
            % self.rating_vector = (1 - lambda_y) * self.rating_vector + trust_vector;
            % self.rating_vector
            % lambda_y
        end


        function trust_score = calculate_trust_score(self,rating_vector)
            %Explain : https://discord.com/channels/1123389035713400902/1327310432381435914/1327403771663224873

            % Normalize the rating vector to ensure it represents probabilities
            % a = (1/self.k) = 1/5 = 0.2 in  self.C / self.k
            S_y = (rating_vector + self.C / self.k) / (self.C + sum(rating_vector));

            % Define weights with a small epsilon to avoid zero weight for the lowest level
            epsilon = 0.01; % Small positive constant
            weights = ((0:(self.k-1)) + epsilon) / (self.k-1 + epsilon);

            % Calculate the trust score as a weighted average
            trust_score = sum(weights .* S_y);
        end



        %%% Main %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   %%%%%%%%%%%%%%%%%%%
        function [final_score,local_trust_sample,gamma_cross, v_score ,d_score,a_score,beacon_score] = calculateTrust(self , host_vehicle, target_vehicle, leader_vehicle, neighbors, is_nearby, instant_idx)
            % calculateTrust - Computes trust scores for a specific vehicle
            %
            % Inputs:
            %   host_id       - ID of the current vehicle
            %   x_local          - Local state of the current vehicle (3x1 vector)
            %   neighbors_states - A matrix where each column is the state of a neighbor vehicle (3xN matrix)
            %   neighbors_ids    - IDs of the neighbor vehicles (1xN vector)
            %   instant_idx        - Current simulation time step (scalar)
            %
            % Outputs:
            %   trust_scores     - Trust scores for each neighbor (1xN vector)

            host_id = host_vehicle.vehicle_number;
            target_id = target_vehicle.vehicle_number;
            % Reported data for trust evaluation

            % half_lenght_vehicle = target_vehicle.param.l_r; % distance between vehicle's c.g. and rear axle
            vehicle_length = target_vehicle.param.l_r + target_vehicle.param.l_f ; % total length of a vehicle

            % leader_velocity = leader_vehicle.observer.est_local_state_current(4);
            % leader_state = host_vehicle.center_communication.get_local_state(leader_vehicle.vehicle_number , host_id);
            leader_state = host_vehicle.other_vehicles(1).state; % Get the leader state from the log
            if ( isnan(leader_state))
                % Use the lastest state of leader vehicle
                leader_state = self.lead_state_lastest ;
                leader_beacon_interval = (instant_idx - self.lead_state_lastest_timestamp)*host_vehicle.dt;
            else
                % save the lastest state of leader vehicle
                self.lead_state_lastest = leader_state;
                self.lead_state_lastest_timestamp = instant_idx;
                leader_beacon_interval = 0;
            end
            %% TODO : need to get the recveived data from the leader vehicle (not the real data)
            leader_velocity = leader_state(4);
            leader_acceleration = leader_state(5);

            % Measurement part host (radar, lidar  )
            host_pos_X = host_vehicle.observer.est_local_state_current(1);
            host_velocity = host_vehicle.observer.est_local_state_current(4);
            host_acceleration = host_vehicle.observer.est_local_state_current(5);

            %%

            % if (host_id - neighbors_ids) < 0  =>  host is Front , else = Behind

            % host_distance_measurement = (host_id - target_id)*(target_vehicle.observer.est_local_state_current(1) - host_vehicle.observer.est_local_state_current(1)) - half_lenght_vehicle;

            % Why target_vehicle.state(1) : because is the host_distance_measurement , its in the point view of Host
            % So need to be acurate , not disturb by attack like "target_pos_X" (below )

            if is_nearby
                delta_X = target_vehicle.state(1) - host_pos_X;  % center-to-center distance

                if delta_X >= 0
                    % Host is behind target → rear-to-bumper (e.g., vehicle 1 to 4)
                    host_distance_measurement = delta_X + vehicle_length;
                    % host_distance_measurement = abs(delta_X);
                else
                    % Host is in front of target → bumper-to-rear (e.g., vehicle 4 to 1)
                    host_distance_measurement = abs(delta_X) - vehicle_length;
                    % host_distance_measurement = abs(delta_X);
                end
                % host_distance_measurement = (host_id - target_id)*(target_vehicle.state(1) - host_pos_X) - vehicle_length;
            else
                %% So why we in test case , we know the real distance between host and target (even its not nearby)
                if host_vehicle.scenarios_config.is_know_data_not_nearby == true
                    delta_X = target_vehicle.state(1) - host_pos_X;  % center-to-center distance

                    if delta_X >= 0
                        % Host is behind target → rear-to-bumper (e.g., vehicle 1 to 4)
                        host_distance_measurement = delta_X + vehicle_length;
                        % host_distance_measurement = abs(delta_X);
                    else
                        % Host is in front of target → bumper-to-rear (e.g., vehicle 4 to 1)
                        host_distance_measurement = abs(delta_X) - vehicle_length;
                        % host_distance_measurement = abs(delta_X);
                    end
                else
                    %% TODO : If we don't know the distance between host and target (dont have the real measurement)
                    % Use the estimated distance based on the host's position and target's position

                    nb_space = abs(host_id - target_id); % sign of the host and target id
                    T = host_vehicle.Param_opt.hi ; %% Time gap , time headway (s)
                    s0 = host_vehicle.Param_opt.ri; % Minimum gap distance
                    s_acc_expected = nb_space*(s0 + T*host_velocity); % h_base = T

                    host_distance_measurement = s_acc_expected - vehicle_length;
                end
            end






            %% TODO : need to get real distance between host and target

            target_state = host_vehicle.center_communication.get_local_state(target_id,host_id);
            if ( isnan(target_state))
                beacon_score = 0;
                v_score = 0;
                d_score = 0;
                a_score = 0;
                h_score = 0;
                local_trust_sample = 0;
            else
                beacon_score = 1;
                % target_input = host_vehicle.center_communication.get_input(target_id);

                target_pos_X = target_state(1);
                target_pos_Y = target_state(2);

                %% distance reporting
                delta_X = target_pos_X - host_pos_X;  % center-to-center distance

                if delta_X >= 0
                    % Host is behind target → rear-to-bumper (e.g., vehicle 1 to 4)
                    target_reported_distance = delta_X + vehicle_length;
                    % target_reported_distance = abs(delta_X) ;
                else
                    % Host is in front of target → bumper-to-rear (e.g., vehicle 4 to 1)
                    target_reported_distance = abs(delta_X) - vehicle_length;
                    % target_reported_distance = abs(delta_X);
                end

                target_reported_velocity = target_state(4);
                target_reported_acceleration = target_state(5);


                % Trust evaluation

                v_score = self.evaluate_velocity( host_id , target_id , target_reported_velocity, host_velocity  , leader_velocity, leader_acceleration, leader_beacon_interval,is_nearby,0.1);
                % Evaluate distance
                d_score = self.evaluate_distance(target_reported_distance, host_distance_measurement,is_nearby);
                % Evaluate acceleration
                a_score = self.evaluate_acceleration(host_vehicle, host_id, target_id, target_reported_acceleration, host_acceleration, [host_distance_measurement, self.last_d], host_vehicle.dt, is_nearby);
                % if instant_idx - self.last_time_d == self.Period_a_score_distane
                %     self.last_time_d = instant_idx;
                % end
                self.last_d = host_distance_measurement;


                % Evaluate heading
                target_reported_heading = target_state(3); % Reported heading (radians)
                h_score = self.evaluate_heading( target_pos_X, target_pos_Y, target_reported_heading, instant_idx);

                % Compute trust sample for local estimation
                % local_trust_sample = self.calculate_trust_sample_wo_Acc(v_score, d_score, a_score, beacon_score , h_score , is_nearby);
                local_trust_sample = self.calculate_trust_sample(v_score, d_score, a_score, beacon_score , h_score , is_nearby);

            end



            if (isnan( target_vehicle.center_communication.get_global_state(target_id,host_id)))
                % Drop packet , not available
                beacon_score = 0;
                gamma_cross = 0;
                gamma_local = 0;
                global_trust_sample = 0;
            else
                % Compute global estimate trust factors
                [gamma_cross, D_pos, D_vel, D_acc] = self.compute_cross_host_target_factor(host_id,host_vehicle, target_id,target_vehicle);
                gamma_local = self.compute_local_consistency_factor(host_vehicle, target_vehicle, neighbors);
                global_trust_sample = gamma_cross*gamma_local;
            end

            %% not use
            % gamma_expected = self.compute_cross_host_expected_2_factor(host_id,host_vehicle, target_id,target_vehicle);


            % Compute extended trust sample
            % trust_sample_ext = 0.5*trust_sample  +  0.5*(gamma_cross * gamma_local);

            if (host_vehicle.scenarios_config.Monitor_sudden_change == true)
                beta = self.monitor_sudden(gamma_cross , D_pos,D_vel,D_acc);
            else
                beta = 1; % Default value
            end

            %% Separate trust sample for local and global estimates
            if (host_vehicle.scenarios_config.Dichiret_type == "Single")
                trust_sample_ext = local_trust_sample  * global_trust_sample ;
                self.update_rating_vector(trust_sample_ext , "local");
                final_score = self.calculate_trust_score(self.rating_vector);

            else % "Dual"
                self.update_rating_vector(local_trust_sample , "local");
                local_trust_sample = self.calculate_trust_score(self.rating_vector);

                self.update_rating_vector(global_trust_sample , "global");
                global_trust_sample = self.calculate_trust_score(self.rating_vector_global);

                final_score = local_trust_sample * global_trust_sample;
            end

            final_score = final_score * beta;



            %% Flag Check
            self.flag_taget_attk = false;
            self.flag_glob_est_check = false;
            self.flag_local_est_check = false;
            if (gamma_local > 0.5 && gamma_cross < 0.5)
                self.flag_taget_attk = true;
            end
            if (gamma_local < 0.5 && gamma_cross > 0.5)
                self.flag_glob_est_check = true;
            end

            %% importance
            if (local_trust_sample < 0.5 )
                self.flag_local_est_check = true;
            end


            % final_score = trust_sample_ext ;

            % Log data for analysis
            self.trust_sample_log = [self.trust_sample_log, local_trust_sample];
            self.gamma_cross_log = [self.gamma_cross_log, gamma_cross];
            self.gamma_local_log = [self.gamma_local_log, gamma_local];
            % self.gamma_expected_log = [self.gamma_expected_log, gamma_expected];

            self.v_score_log = [self.v_score_log, v_score];
            self.d_score_log = [self.d_score_log, d_score];
            self.a_score_log = [self.a_score_log, a_score];
            self.h_score_log = [self.h_score_log, h_score];

            self.beacon_score_log = [self.beacon_score_log, beacon_score];

            self.final_score_log = [self.final_score_log, final_score];

            self.flag_taget_attk_log = [self.flag_taget_attk_log, self.flag_taget_attk];
            self.flag_glob_est_check_log = [self.flag_glob_est_check_log, self.flag_glob_est_check];
            self.flag_local_est_check_log = [self.flag_local_est_check_log, self.flag_local_est_check];

        end

        function beta = monitor_sudden(self,gamma_cross,D_pos,D_vel,D_acc)
            % --- New Code: Anomaly Detection and Trust Adjustment ---

            %% TODO in the case DOS ,so need to forget about the anomaly pos and velocity
            % Anomaly detection for gamma_cross
            anomaly_gamma = 0;  % Default value
            if length(self.gamma_cross_log) >= self.w
                window = self.gamma_cross_log(end-self.w+1:end);
                mu_gamma = mean(window);
                sigma_gamma = std(window);
                if sigma_gamma > 0 && abs(gamma_cross - mu_gamma) > 2 * sigma_gamma
                    anomaly_gamma = 1;
                end
            end
            self.anomaly_gamma_log = [self.anomaly_gamma_log, anomaly_gamma];

            % Log discrepancies
            self.D_pos_log = [self.D_pos_log, D_pos];
            self.D_vel_log = [self.D_vel_log, D_vel];
            self.D_acc_log = [self.D_acc_log, D_acc];

            % Anomaly detection for position
            anomaly_pos = 0;
            if length(self.D_pos_log) >= self.w
                window = self.D_pos_log(end-self.w+1:end);
                mu_pos = mean(window);
                sigma_pos = std(window);
                if sigma_pos > 0 && abs(D_pos - mu_pos) > 2 * sigma_pos
                    anomaly_pos = 1;
                end
            end
            self.anomaly_pos_log = [self.anomaly_pos_log, anomaly_pos];

            % Anomaly detection for velocity
            anomaly_vel = 0;
            if length(self.D_vel_log) >= self.w
                window = self.D_vel_log(end-self.w+1:end);
                mu_vel = mean(window);
                sigma_vel = std(window);
                if sigma_vel > 0 && abs(D_vel - mu_vel) > 2 * sigma_vel
                    anomaly_vel = 1;
                end
            end
            % Log the anomaly velocity

            self.anomaly_vel_log = [self.anomaly_vel_log, anomaly_vel];


            % Anomaly detection for acceleration
            anomaly_acc = 0;
            if length(self.D_acc_log) >= self.w
                window = self.D_acc_log(end-self.w+1:end);
                mu_acc = mean(window);
                sigma_acc = std(window);
                if sigma_acc > 0 && abs(D_acc - mu_acc) > 2 * sigma_acc
                    anomaly_acc = 1;
                end
            end
            % Log the anomaly acceleration
            self.anomaly_acc_log = [self.anomaly_acc_log, anomaly_acc];



            beta = 1; % Default value
            % Cumulative check for trust adjustment
            if length(self.anomaly_gamma_log) >= self.w
                % self.w - 1 because length(self.beacon_score_log) only have w-1 elements
                anomaly_drop_packet = (self.w - 1) - sum(self.beacon_score_log(end-self.w+2:end));
                anomaly_count_gamma = sum(self.anomaly_gamma_log(end-self.w+1:end));
                anomaly_count_pos = sum(self.anomaly_pos_log(end-self.w+1:end));
                anomaly_count_vel = sum(self.anomaly_vel_log(end-self.w+1:end));
                anomaly_count_acc = sum(self.anomaly_acc_log(end-self.w+1:end));
                min_anomali = min([anomaly_count_gamma, anomaly_count_pos, anomaly_count_vel, anomaly_count_acc, anomaly_drop_packet]);
                if min_anomali > self.Threshold_anomalie
                    beta = (1-self.reduce_factor*min_anomali / self.w);
                end
            end
        end



        % Only compare with the directed neighbor (not all neighbors) , or more specific is the target vehicle
        function [gamma_cross, D_pos, D_vel, D_acc] = compute_cross_host_target_factor(self, host_id,host_vehicle, target_id,target_vehicle)
            % Inputs:
            %   host_vehicle: The vehicle evaluating trust
            %   target_vehicle: The neighbor whose global estimate is being evaluated
            %   neighbors: Array of neighbor vehicle objects
            %
            % Output:
            %   gamma_cross: Trust factor based on cross-validation (0 to 1)

            host_global_estimate = host_vehicle.observer.est_global_state_current;

            % Get target vehicle's global estimate
            target_global_estimate = host_vehicle.center_communication.get_global_state(target_id,host_id);


            % Compute total discrepancy D_i,l(k)
            D = 0;
            D_pos = 0;
            D_vel = 0;
            D_acc = 0;
            num_vehicles = size(target_global_estimate,2);
            for j = 1:num_vehicles

                % Position difference
                pos_diff = target_global_estimate(1:2, j) - host_global_estimate(1:2, j);
                D_pos = D_pos + pos_diff' * inv(self.sigma2_matrix_gamma_cross(1:2,1:2)) * pos_diff;

                % Velocity difference
                vel_diff = target_global_estimate(4, j) - host_global_estimate(4, j);
                D_vel = D_vel + vel_diff' * inv(self.sigma2_matrix_gamma_cross(4,4)) * vel_diff;

                % Acceleration difference
                acc_diff = target_global_estimate(5, j) - host_global_estimate(5, j);
                D_acc = D_acc + acc_diff' * inv(self.sigma2_matrix_gamma_cross(5,5)) * acc_diff;


                % that is in the paper
                x_diff = target_global_estimate(:, j) - host_global_estimate(:, j);
                D = D + x_diff' * inv(self.sigma2_matrix_gamma_cross) * x_diff; % Mahalanobis distance
            end

            % Compute trust factor
            gamma_cross = exp(-D);
        end







        function gamma_local = compute_local_consistency_factor(self, host_vehicle, target_vehicle, neighbors)
            % Inputs:
            %   host_vehicle: The vehicle evaluating trust
            %   target_vehicle: The neighbor whose global estimate is being evaluated
            %   local_measurements: Struct with sensor data (e.g., relative state to predecessor)
            %
            % Output:
            %   gamma_local: Trust factor based on local consistency (0 to 1)

            half_lenght_vehicle = target_vehicle.param.l_r; % distance between vehicle's c.g. and rear axle

            M_i = 0; % Set of vehicles used for local consistency check (max = 2 for predecessor and successor)
            predecessor = [];
            successor = [];
            host_id = host_vehicle.vehicle_number;
            target_global_estimate = host_vehicle.center_communication.get_global_state(target_vehicle.vehicle_number , host_id);

            x_l_i = target_global_estimate([1,4], host_id); % In L target vehicle , get the estimate of the host vehicle (i is host)
            % If position errors are around 2m and velocity errors around 1m/s

            E = 0; % Total error

            for m = 1:length(neighbors)
                car_idx = neighbors(m).vehicle_number;
                if abs(car_idx - host_vehicle.vehicle_number) == 1
                    if (car_idx > host_vehicle.vehicle_number)
                        M_i = M_i + 1;
                        successor = neighbors(m);
                    end
                    if (car_idx < host_vehicle.vehicle_number)
                        M_i = M_i + 1;
                        predecessor = neighbors(m); % Mean following vehicle
                    end
                end
            end

            % If no predecessor or successor is found, return maximum trust (no comparison possible)
            if isempty(predecessor) && isempty(successor)
                gamma_local = 1;
                return;
            end


            if (~isempty(predecessor))
                % Get local measurement (e.g., relative position and velocity to predecessor)
                % Assume local_measurements.predecessor is a vector [rel_pos; rel_vel]
                host_distance_measurement = (predecessor.state(1) - host_vehicle.state(1)) - half_lenght_vehicle;
                velocity_diff_measurement = abs(predecessor.state(4) - host_vehicle.state(4));
                y_i_predecessor = [host_distance_measurement; velocity_diff_measurement];

                pred_id = predecessor.vehicle_number;

                x_l_pred = target_global_estimate([1,4], pred_id); % In L car , get the estimate of the predceding of host vehicle (i + 1 is pred)

                % Compute  relative state from global estimate received
                rel_state_est = abs(x_l_pred - x_l_i - [half_lenght_vehicle;0]);

                % Compute consistency error e_i,l^(j)(k)
                e = rel_state_est - y_i_predecessor;

                E =  e' * inv(self.tau2_matrix_gamma_local) * e + E;

            end

            if (~isempty(successor))
                % Get local measurement (e.g., relative position and velocity to successor)
                % Assume local_measurements.successor is a vector [rel_pos; rel_vel]
                host_distance_measurement = (host_vehicle.state(1) - successor.state(1) ) - half_lenght_vehicle;
                velocity_diff_measurement = abs(successor.state(4) - host_vehicle.state(4));
                y_i_successor = [host_distance_measurement; velocity_diff_measurement]; % Host measurement

                successor_id = successor.vehicle_number;

                x_l_successor = target_global_estimate([1,4], successor_id); % In L car , get the estimate of the successor of host vehicle (i - 1 is succ)

                % Compute expected relative state from global estimate
                rel_state_est = abs(x_l_i - x_l_successor - [half_lenght_vehicle;0]);

                % Compute consistency error e_i,l^(j)(k)
                e = (rel_state_est - y_i_successor);
                E = e' * inv(self.tau2_matrix_gamma_local) * e + E;

            end

            % Compute trust factor
            gamma_local = exp(-E);
        end



        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Plotting function %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   %%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function plot_details_acc_score(self, host_vehicle_number, target_vehicle_number)
            % Plot acceleration score over time in a 3x3 grid layout
            figure("Name", "Details Acc Score " + host_vehicle_number + "->" + target_vehicle_number);

            tiledlayout(3, 3, 'TileSpacing', 'compact'); % use compact spacing

            % Subplot 1: Relative Velocity
            nexttile;
            plot(self.v_rel_log, 'DisplayName', 'Relative Velocity', 'LineWidth', 1.5);
            title('$v_{rel}$', 'Interpreter', 'latex');
            ylim([-2 2]);
            grid on;

            % Subplot 2: Additional Distance
            nexttile;
            plot(self.d_add_log, 'DisplayName', 'Additional Distance', 'LineWidth', 1.5);
            title('$d_{add}$', 'Interpreter', 'latex');
            grid on;

            % Subplot 3: Relative Acceleration
            nexttile;
            plot(self.acc_rel_log, 'DisplayName', 'Relative Acceleration', 'LineWidth', 1.5);
            title('$acc_{rel}$', 'Interpreter', 'latex');
            grid on;

            % Subplot 4: Delta Acc Expected
            nexttile;
            plot(self.delta_acc_expected_log, 'DisplayName', 'Expected Acceleration Difference', 'LineWidth', 1.5);
            title('$\Delta acc_{expec}$', 'Interpreter', 'latex');
            grid on;

            % Subplot 5: Distance
            nexttile;
            plot(self.distance_log, 'DisplayName', 'Distance Log', 'LineWidth', 1.5, 'Color', 'magenta');
            title('Distance');
            ylabel('Meters');
            grid on;

            % Subplot 6: Scale d Expected
            nexttile;
            plot(self.scale_d_expected_log, 'DisplayName', 'Scaled Expected Distance', 'LineWidth', 1.5);
            xlabel('Time Step');
            title(['Scale $$d_{expected}$$ ' num2str(host_vehicle_number) ' $->$ ' num2str(target_vehicle_number)], 'Interpreter', 'latex');
            grid on;

            % Subplot 7: Delta d
            nexttile;
            plot(self.delta_d_log, 'DisplayName', 'Delta Distance', 'LineWidth', 1.5);
            title('$\Delta d$', 'Interpreter', 'latex');
            ylabel('Meters');
            grid on;

            % If you want, you can leave the last two tiles blank or use them for legends, summary text, etc.
        end


        function plot_diff_acc_score(self , host_vehicle_number, target_vehicle_number)

            figure("Name", "Diff Acc Score"+ host_vehicle_number + "->" + target_vehicle_number);
            subplot(4,1,1);
            plot(self.a_score_defaut_log, 'DisplayName', 'Default Acc Score', 'LineWidth', 1.5);
            title('Default Acc Score $\frac{v_{rel}}{T_s}$', 'Interpreter', 'latex');
            grid on;
            subplot(4,1,2);
            plot(self.a_score_diff_acc_log , 'DisplayName', 'Diff Acc Score', 'LineWidth', 1.5);
            title('Diff Acc Score');
            grid on;
            subplot(4,1,3);
            plot(self.a_score_expected_diff_acc_log, 'DisplayName', 'Expected Acceleration', 'LineWidth', 1.5);
            title('Expected Diff Acceleration $\Delta acc_{expec}$', 'Interpreter', 'latex');
            grid on;
            subplot(4,1,4);
            plot(self.a_score_vrel_dis_log, 'DisplayName', 'Vrel Dist', 'LineWidth', 1.5);
            title('Vrel Dist $\frac{v_{rel}}{d(1)}$', 'Interpreter', 'latex');


        end


        function plot_trust_log(self,nb_host_car , nb_target_car)
            figure("Name", num2str(nb_host_car) +  " Trust for " + num2str(nb_target_car), "NumberTitle", "off");



            % hold on;
            global_trust = self.gamma_local_log.*self.gamma_cross_log;
            subplot(3,1,1);

            plot(self.gamma_cross_log, 'DisplayName', 'Gamma Cross', 'LineWidth', 1);
            hold on;
            plot(self.gamma_local_log, 'DisplayName', 'Gamma Local','LineWidth', 1);
            plot(global_trust, 'DisplayName', 'Global Trust','LineWidth', 1.5);
            grid on;
            legend show;

            %% Not use
            % plot(self.gamma_expected_log, 'DisplayName', 'Gamma expect', 'LineWidth', 1);


            subplot(3,1,2);
            plot(self.a_score_log, 'DisplayName', 'A Score', 'LineWidth', 1);
            hold on;
            plot(self.v_score_log, 'DisplayName', 'V Score', 'LineWidth', 1);
            plot(self.d_score_log, 'DisplayName', 'D Score', 'LineWidth', 1);
            plot(self.trust_sample_log, 'DisplayName', 'Local Trust', 'LineWidth', 1.5);
            grid on;

            legend show;


            subplot(3,1,3);
            plot(self.final_score_log, 'DisplayName', 'Final Score' , 'LineWidth', 1.5);

            % plot(self.beacon_score_log, 'DisplayName', 'Beacon Score');

            xlabel('Time Step');
            ylabel('Value');
            title([num2str(nb_host_car) '-> Trust and Score Logs Over Time for car '  num2str(nb_target_car)],"LineWidth",1);
            legend show;
            grid on;

            hold off;
        end


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Not Use %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function gamma_cross_expected = compute_cross_host_expected_factor(self, host_id, host_vehicle, target_id, target_vehicle)
            % Get target vehicle's global estimate
            target_global_estimate = target_vehicle.center_communication.get_global_state(target_id, host_id);

            % Define variances for covariance matrix
            var_x = 3;         % Variance for X-difference
            var_y = 1;         % Variance for Y-difference
            var_angle = 0.01;  % Variance for angle-difference
            var_velocity = 1;  % Variance for velocity-difference
            sigma2_matrix = diag([var_x, var_y, var_angle, var_velocity]);

            % Retrieve control parameters (assumed available as properties or from the vehicle)
            s0      = 8;       % Minimum spacing (m)
            h_base  = 0.5;   % Base time headway (s) for the first follower (vehicle 2)
            delta_h = 0.1;  % Headway reduction per position in the platoon (s)
            if ~isempty(host_vehicle.gamma_log)
                gamma_control = host_vehicle.gamma_log(end);
            else
                gamma_control = 1;
            end
            % Initialize total discrepancy
            D = 0;
            num_vehicles = size(target_global_estimate, 2);

            % Loop over consecutive vehicle pairs in the global estimate
            for j = 1:num_vehicles - 1
                % Difference between vehicle j and j+1 in the global state vector
                state_diff = target_global_estimate(:, j) - target_global_estimate(:, j+1);

                % Compute adaptive expected spacing for the follower (vehicle n = j+1)
                n = j + 1; % Vehicle index in platoon (leader is n=1)
                % Adaptive headway for this vehicle
                h_n = h_base - delta_h * (n - 2);

                % Assume state vector: [x; y; angle; velocity]
                % Use the velocity of the follower vehicle (j+1) for the spacing calculation
                v = target_global_estimate(4, j+1);

                % Compute expected spacing using the mixing formula:
                % s_expected = s0 + (gamma*h_base + (1-gamma)*h_n)*v
                s_expected = s0 + ((1 - gamma_control) * h_base + gamma_control * h_n) * v;

                % Define the expected difference vector for this gap. Only the X-component is adaptive.
                mu_diff_gap = [s_expected; 0; 0; 0];

                % Calculate difference from expected value for this pair
                diff_from_expected = state_diff - mu_diff_gap;

                % Accumulate the discrepancy using the Mahalanobis distance
                D = D + diff_from_expected' * inv(sigma2_matrix) * diff_from_expected;
            end

            % Compute the cross expected trust factor as an exponential decay with the total discrepancy
            gamma_cross_expected = exp(-D);
        end

        function gamma_cross_expected = compute_cross_host_expected_2_factor(self, host_id, host_vehicle, target_id, target_vehicle)
            % Get target vehicle's global estimate
            target_global_estimate = target_vehicle.center_communication.get_global_state(target_id, host_id);

            % Define variances for covariance matrix
            var_x = 3;         % Variance for X-difference
            var_y = 1;         % Variance for Y-difference
            var_angle = 0.01;  % Variance for angle-difference
            var_velocity = 0.5;  % Variance for velocity-difference
            sigma2_matrix = diag([var_x, var_y, var_angle, var_velocity]);

            % Retrieve control parameters (assumed available as properties or from the vehicle)
            s0      = 8;       % Minimum spacing (m)
            h_base  = 0.4;   % Base time headway (s) for the first follower (vehicle 2)
            T = 0.5;
            delta_h = 0.1;  % Headway reduction per position in the platoon (s)
            if ~isempty(host_vehicle.gamma_log)
                gamma_control = host_vehicle.gamma_log(end);
            else
                gamma_control = 1;
            end
            % Initialize total discrepancy
            D = 0;
            num_vehicles = size(target_global_estimate, 2);

            % Loop over consecutive vehicle pairs in the global estimate
            for j = 1:num_vehicles - 1
                % Difference between vehicle j and j+1 in the global state vector
                state_diff = target_global_estimate(:, j) - target_global_estimate(:, j+1);

                % Assume state vector: [x; y; angle; velocity]
                % Use the velocity of the follower vehicle (j+1) for the spacing calculation
                v = target_global_estimate(4, j+1);

                s_acc = s0 + T*v; % h_base = T
                s_i = s0 + h_base*v/j;
                % Compute expected spacing using the mixing formula:
                % s_expected = s0 + (gamma*h_base + (1-gamma)*h_n)*v
                s_expected =  ((1 - gamma_control) * s_acc + gamma_control * s_i) ;
                % U_final = self.input + tau_filter*(U_target - self.input) ; % self.input is last input

                % Define the expected difference vector for this gap. Only the X-component is adaptive.
                mu_diff_gap = [s_expected; 0; 0; 0];

                % Calculate difference from expected value for this pair
                diff_from_expected = state_diff - mu_diff_gap;

                % Accumulate the discrepancy using the Mahalanobis distance
                D = D + diff_from_expected' * inv(sigma2_matrix) * diff_from_expected;
            end

            % Compute the cross expected trust factor as an exponential decay with the total discrepancy
            gamma_cross_expected = exp(-D);
        end



        % function gamma_cross_expected = compute_cross_host_expected_factor(self, host_id, host_vehicle, target_id, target_vehicle)
        %     % Get target vehicle's global estimate
        %     target_global_estimate = target_vehicle.center_communication.get_global_state(target_id, host_id);

        %     % Define expected differences
        %     expected_diff_x = 20;      % Expected X-spacing (m)
        %     expected_diff_y = 0;       % Expected Y-offset (m)
        %     expected_diff_angle = 0;   % Expected angle difference (rad)
        %     expected_diff_velocity = 0; % Expected velocity difference (m/s)
        %     mu_diff = [expected_diff_x; expected_diff_y; expected_diff_angle; expected_diff_velocity];

        %     % Define variances for covariance matrix
        %     var_x = 1;         % Variance for X-difference
        %     var_y = 1;         % Variance for Y-difference
        %     var_angle = 0.01;  % Variance for angle-difference
        %     var_velocity = 1;  % Variance for velocity-difference
        %     sigma2_matrix = diag([var_x, var_y, var_angle, var_velocity]);

        %     % Compute total discrepancy
        %     D = 0;
        %     num_vehicles = size(target_global_estimate, 2);
        %     for j = 1:num_vehicles - 1
        %         x_diff = target_global_estimate(:, j) - target_global_estimate(:, j+1);
        %         diff_from_expected = x_diff - mu_diff;
        %         D = D + diff_from_expected' * inv(sigma2_matrix) * diff_from_expected; % Mahalanobis distance
        %     end

        %     % Compute trust factor
        %     gamma_cross_expected = exp(-D);
        % end



        % function sigma2_matrix = get_adaptive_covariance(self, host_vehicle, target_vehicle)
        %     % Compute adaptive covariance matrix based on past estimation differences
        %     history_length = min(length(self.trust_history), 50); % Use the last 50 time steps
        %     if history_length < 2
        %         sigma2_matrix = eye(size(host_vehicle.observer.est_global_state_current, 1)); % Default to identity matrix
        %         return;
        %     end

        %     diff_history = []; % Store state differences
        %     for t = length(self.trust_history) - history_length + 1 : length(self.trust_history)
        %         diff_t = self.trust_history{t}.(target_vehicle.vehicle_number) - self.trust_history{t}.(host_vehicle.vehicle_number);
        %         diff_history = [diff_history, diff_t];
        %     end

        %     sigma2_matrix = cov(diff_history'); % Compute covariance from history

        %     % Ensure it's positive definite
        %     if rcond(sigma2_matrix) < 1e-6
        %         sigma2_matrix = sigma2_matrix + 1e-3 * eye(size(sigma2_matrix));
        %     end
        % end

        % function beta = compute_dynamic_weight(self, host_vehicle, target_vehicle)
        %     % Compute dynamic weight beta based on trust history
        %     history_length = min(length(self.trust_history), 50);
        %     if history_length < 2
        %         beta = 0.5; % Default equal weighting
        %         return;
        %     end

        %     error_sum = 0;
        %     for t = length(self.trust_history) - history_length + 1 : length(self.trust_history)
        %         error_t = norm(self.trust_history{t}.(target_vehicle.vehicle_number) - self.trust_history{t}.(host_vehicle.vehicle_number));
        %         error_sum = error_sum + error_t;
        %     end

        %     avg_error = error_sum / history_length;
        %     beta = exp(-avg_error); % Higher error -> lower trust in global estimate
        % end


        % Only compare with the directed neighbor (not all neighbors) , or more specific is the target vehicle
        function gamma_cross = compute_cross_validation_factor(self, host_vehicle, target_vehicle, neighbors)
            % Inputs:
            %   host_vehicle: The vehicle evaluating trust
            %   target_vehicle: The neighbor whose global estimate is being evaluated
            %   neighbors: Array of neighbor vehicle objects
            %
            % Output:
            %   gamma_cross: Trust factor based on cross-validation (0 to 1)

            % Collect global estimates from all neighbors
            num_neighbors = length(neighbors);
            if num_neighbors == 0
                gamma_cross = 1; % Default to full trust if no neighbors
                return;
            end

            % Assume each vehicle's observer provides the global estimate as a matrix
            % Columns represent vehicle states, rows represent state variables (e.g., position, velocity)
            global_estimates = cell(num_neighbors, 1);
            for m = 1:num_neighbors
                % host_vehicle.center_communication.get_global_state(neighbors(m).vehicle_number);
                global_estimates{m} = neighbors(m).observer.est_global_state_current;
            end

            % Get target vehicle's global estimate
            target_global_estimate = target_vehicle.observer.est_global_state_current;
            [state_dim, num_vehicles] = size(target_global_estimate);

            % Compute average estimate across all neighbors for each vehicle
            average_estimate = zeros(state_dim, num_vehicles);
            for m = 1:num_neighbors
                average_estimate = average_estimate + global_estimates{m};
            end
            average_estimate = average_estimate / num_neighbors;

            % Compute total discrepancy D_i,l(k)
            D = 0;
            for j = 1:num_vehicles
                d_j = norm(target_global_estimate(:, j) - average_estimate(:, j))^2;
                D = D + d_j;
            end

            % Compute trust factor
            gamma_cross = exp(-D / self.sigma2);
        end

        function following_distance = determine_following_distance(~, trust_score, ds, dacc)
            if trust_score > 0.8
                following_distance = ds;
            elseif trust_score > 0.2
                following_distance = ds + (dacc - ds) * (0.8 - trust_score);
            else
                following_distance = dacc;
            end
        end
    end
end