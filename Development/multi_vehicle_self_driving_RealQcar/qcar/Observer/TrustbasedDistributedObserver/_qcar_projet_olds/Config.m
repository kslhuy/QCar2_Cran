clc
close all
clear

addpath ../test/core/

addpath('Function');
addpath('core/observer');
addpath('core/Controller');
addpath('core/Trust');
addpath('core/communication');


%% Params affect the simulation
param_sys = ParamVeh();

%% Simulation and Senarios related  
dt = 0.01; % time step
simulation_time = 25; % simulation time
Road_type = "Highway"; % "Highway" , "Urban"
Scenarios_config = Scenarios_config(dt, simulation_time,  Road_type  );


% is_lead_input_change = false; % if the lead input is changing (for different senarios)
Scenarios_config.lead_senario = "constant"; % "constant" , "Acceleration" , "Deceleration" , "Lane_change"

%%%% Observer related
Scenarios_config.Use_predict_observer = true; % Will override distributed observer with a prediction model

Scenarios_config.predict_controller_type = "true_other"; % "self" , "true_other" , "predict_other"

Scenarios_config.Local_observer_type = "kalman"; % "mesurement" , "kalman" , "observer"

Scenarios_config.Is_noise_mesurement = true; % if the measurement is noisy
Scenarios_config.noise_probability = 0.2; % Probability of adding measurement noise
Scenarios_config.Use_smooth_filter = true; % Enable/disable smooth filtering for noise and observer output
Scenarios_config.Use_smooth_filter_in_local_observer = false; % Enable/disable smooth filtering in local observer

Scenarios_config.use_local_data_from_other = true; % if the local data from other vehicles is used (true = ourpaper , false = another paper)
% - PREDICTION in observer PARAMETERS
Scenarios_config.MAX_PREDICT_ONLY_TIME = 3; % seconds
Scenarios_config.N_good = 3; % Number of consecutive good steps to exit predict_only
Scenarios_config.blend_thresh = 3; % You can tune this threshold

Scenarios_config.rollback_enabled = true; % Enable/disable rollback functionality




%%%% Attack related

attack_module = Attack_module(Scenarios_config.dt);
% Define a time-based attack scenario
% Attack from t_star to t_end
% a vehicle with attacker_vehicle_id will be the attacker

t_star = 10;
t_end = 15;
attacker_vehicle_id = 1;
victim_id = -1;                 % -1 mean every vehicle
case_nb_attack = 4;             % case number of attack scenario
data_type_attack = "global"; % "local" , "global", "none"
attack_type = "Mix_test"; % "DoS"  , "Collusion" ,"Bogus", "None" , "POS" , "VEL" , "ACC"


Scenarios_config.attacker_update_locally = true; % if the attacker does update observer by only using the local data   



%%% Controller related
Scenarios_config.control_use_accel = true; % if using acceleration control

Scenarios_config.gamma_type = "min"; % type gamma for switching control = " min" , " max " , " mean " , "self_belief"
Scenarios_config.controller_type = "mix"; % type of controller for the ego vehicle: "local" , "coop" , "mix" 
Scenarios_config.CACC_bidirectional = false; % If true, the CACC controller will consider both leading and following vehicles in the control law

Scenarios_config.data_type_for_u2 = "true"; % "est" , "true" using the estimated or true data for u2 (CACC)

%%% Trust related

Scenarios_config.using_weight_trust_observer = true; % if using weight trust for observer

Scenarios_config.opinion_type = "mix_non_nearby"; % opinion type " distance" , " trust" , " both" , "mix_non_nearby"
Scenarios_config.Dichiret_type = "Single" ; % "Single" , "Dual"
Scenarios_config.Use_weight_local_trust = false; % if using weight trust for local data
Scenarios_config.Use_weight_global_trust = false; % if using weight trust for global data

Scenarios_config.acceleration_trust_score_method = "vrel_dis_adjusted"; % 'mathematical' - Use exact mathematical formula from paper
                                                                        % 'enhanced' - Use enhanced implementation with reduced sensitivity
                                                                        % 'default' - Use default implementation
                                                                        % 'vrel_dis_adjusted' - Use adjusted relative distance
                                                                        % 'vrel_dis_real' - Use real relative distance
                                                                        % 'hybrid' - Combine both methods

% -- option trust (no imporant yet)
Scenarios_config.Monitor_sudden_change = false; % if the sudden change is monitored

Scenarios_config.is_know_data_not_nearby = true ; % just for test purpose, Use that we have better Trust score , meaning that we know the data all of the other vehicles

% New validation controls
Scenarios_config.Use_physical_constraints_check = true; % Enable/disable physical constraints validation (recommended: true)
Scenarios_config.Use_temporal_consistency_check = true; % Enable/disable temporal consistency evaluation (recommended: true)

%%% Model related
Scenarios_config.model_vehicle_type = "delay_a"; % "delay_v" , "delay_a" , "normal","paper"

%% ------------------  Graph related
graph = [0 1 1 1;  % Adjacency matrix
        1 0 1 1;
        1 1 0 1;
        1 1 1 0];

trust_threshold = 0.5; % for cut the communication in the graph
kappa = 1; % parameter in the design weigts matrix
Weight_Trust_module = Weight_Trust_module(graph, trust_threshold, kappa);



% ------------------ Define driving Senarios lanes
% Create a straight lane with specified width and length
lane_width = Scenarios_config.getLaneWidth();% width of each single lane

num_lanes = 3; % number of the lanes
max_length = 750; % maximum length of the lanes
straightLanes = StraightLane(num_lanes, lane_width, max_length);
