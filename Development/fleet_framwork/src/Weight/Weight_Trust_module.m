classdef Weight_Trust_module < handle
    % TrustManager - A class to manage trust scores, trusted neighbors, and weights
    % for a platoon of vehicles under potential attacks.

    properties
        graph           % Adjacency matrix of the original graph (n x n)
        % trust_scores    % Cell array of trust scores, trust_scores{i} for vehicle i's neighbors
        trust_threshold % Threshold for determining trusted neighbors (e.g., 0.5)
        kappa           % Parameter to limit neighbor influence (kappa > 0)
        num_vehicles    % Number of vehicles in the platoon
    end

    methods
        % Constructor
        function self = Weight_Trust_module(graph, trust_threshold, kappa)
            % Initialize the TrustManager with graph and parameters
            %
            % Parameters:
            %   graph              - Adjacency matrix (n x n)
            %   initial_trust_scores - Cell array of initial trust scores
            %   trust_threshold    - Trust score threshold (e.g., 0.5)
            %   kappa              - Influence limit parameter
            self.graph = graph;
            self.trust_threshold = trust_threshold;
            self.kappa = kappa;
            self.num_vehicles = size(graph, 1);
        end


        % Compute trusted neighbors
        function neighbors = get_trusted_neighbors(self ,car_idx ,trust_scores)
            % Compute trusted neighbors for each vehicle based on current trust scores
            %
            % Returns:
            %   trusted_neighbors_set - Cell array where trusted_neighbors_set{i} contains indices
            %                       (1 to n) of trusted neighbors of vehicle i

            trust_scores_matrix = [];
            for i = 1:self.num_vehicles
                trust_scores_matrix = [trust_scores_matrix ,trust_scores(i) ];
            end
            neighbors = find(self.graph(car_idx, :)); % Initially set as direct neighbors


            % Remove neighbors whose trust score is below the threshold
            low_trust_mask = trust_scores_matrix(neighbors) <= self.trust_threshold;
            neighbors(low_trust_mask) = []; % Remove untrusted neighbors

        end



        % Compute weight matrix based on trusted neighbors
        function weights_Dis = calculate_weights_Trust(self,vehicle_index , trust_scores,type)

            % Check if the 'type' argument is provided
            if nargin < 4
                type = "local"; % Default value
            end


            virtual_graph = self.generate_virtual_graph(self.graph, vehicle_index);
            trusted_neighbors_set = self.get_trusted_neighbors(vehicle_index,trust_scores);
            num_nodes = size(virtual_graph, 1);
            weights_Dis = zeros(1, num_nodes);


            N_i_t = trusted_neighbors_set; % Trusted neighbors (indices 1 to n)
            n_w_i = max(self.kappa, length(N_i_t) + 1 + 1); % +1 for self virtual node 
            weight = 1 / n_w_i;

            % Set self-weight: W(i+1, i+1)
            weights_Dis(1, vehicle_index+1) = weight;

            % Set weights for trusted neighbors: W(1, l+1)
            for l = N_i_t
                weights_Dis(1, l+1) = weight;
            end

            % Set weight to virtual node: W(1, 1)   
        
            if(type == "local")
                %% Using weight matrix priotizing local estimation
                weights_Dis(1, 1) = 1 - (length(N_i_t)+1) * weight;

            else
                %% Using weight matrix distributed equally

                weights_Dis(1, 1) = weight;
            end
            %% NOTE : Either using local or distributed weight matrix
            %% One problem is the weight matrix W(1, 1) still need to be ajusted based on the trust 
            %% See more details in Observer class      
        end





        % % Compute weight matrix based on trusted neighbors
        % function W = calculate_weights_Trust(self,vehicle_index , trust_scores)
        %     % Compute weight matrix W based on trusted neighbors for a virtual graph
        %     %
        %     % Parameters:
        %     %   virtual_graph - Adjacency matrix of the virtual graph (n+1 x n+1)
        %     %
        %     % Returns:
        %     %   W            - Weight matrix (n+1 x n+1)

        %     virtual_graph = self.generate_virtual_graph(self.graph, vehicle_index);
        %     trusted_neighbors_set = self.get_trusted_neighbors( vehicle_index , trust_scores);
        %     num_nodes = size(virtual_graph, 1);
        %     W = zeros(num_nodes, num_nodes);

        %     for i = 1:self.num_vehicles
        %         N_i_t = trusted_neighbors_set{i}; % Trusted neighbors (indices 1 to n)
        %         n_w_i = max(self.kappa, length(N_i_t) + 1); % +1 for self
        %         weight = 1 / n_w_i;

        %         % Set self-weight: W(i+1, i+1)
        %         W(i+1, i+1) = weight;

        %         % Set weights for trusted neighbors: W(i+1, l+1)
        %         for l = N_i_t
        %             W(i+1, l+1) = weight;
        %         end

        %         % Set weight to virtual node: W(i+1, 1)
        %         W(i+1, 1) = 1 - length(N_i_t) * weight;
        %     end
        % end


        function weights_Dis = calculate_weights_Defaut(self ,  vehicle_index )
            % Function to calculate consensus weights for a given virtual graph
            %
            % Parameters:
            %   Vj - Adjacency matrix representing the virtual graph. It is a square
            %        matrix where Vj(i, j) = 1 if there is an edge between node i and
            %        node j, and 0 otherwise.
            %
            % Returns:
            %   W - Weight matrix where W(i, j) represents the weight assigned to the
            %       edge between node i and node j. The weights are calculated based
            %       on the degree of each node.
            %
            % The function initializes a weight matrix W with zeros. For each node i
            % (starting from the second node), it calculates the degree d_i of the node.
            % Then, for each node l, if there is an edge between node i and node l or
            % if i equals l (self-loop), it assigns a weight of 1 / (d_i + 1) to W(i, l).

            % https://discord.com/channels/1123389035713400902/1326223031667789885/1328426310963564564

            Vj = self.generate_virtual_graph(self.graph, vehicle_index);
            num_nodes = size(Vj, 1);
            W = zeros(num_nodes); % Initialize weights matrix

            for i = 2:num_nodes % Include node 0 (index 1 in MATLAB)
                d_i = sum(Vj(i, :)); % Degree of node i
                for l = 1:num_nodes % Include all nodes in the virtual graph
                    if Vj(i, l) == 1 || i == l % Neighbor or self
                        W(i, l) = 1 / (d_i + 1);
                    end
                end
            end

            weights_Dis = W(vehicle_index + 1,:); % array (1 x 'nb_vehicles + 1' )

        end

        function Vj = generate_virtual_graph(self,graph, vehicle_index)

            %{
            % This function generates a virtual graph by adding an extra node (node 0)
            % and connecting it to the specified vehicle and its neighbors.
            % The input 'graph' is the adjacency matrix of the original graph.
            % The input 'vehicle_index' is the index of the vehicle to which node 0 will be connected.
            %}

            % Function to generate virtual graph for a given vehicle
            Vj = zeros(self.num_vehicles + 1); % Initialize virtual graph with an extra node

            % Copy the adjacency matrix to the virtual graph
            Vj(2:end, 2:end) = graph;

            % Set the bidirectional edge between node 0 and the specified vehicle
            Vj(1, vehicle_index + 1) = 1;
            Vj(vehicle_index + 1, 1) = 1;

            % Set the edges between node 0 and node j’s neighbors
            neighbors = find(graph(vehicle_index, :));
            for neighbor = neighbors
                Vj(1, neighbor + 1) = graph(vehicle_index, neighbor);
                Vj(neighbor + 1, 1) = graph(neighbor, vehicle_index);
            end
        end

    end
end