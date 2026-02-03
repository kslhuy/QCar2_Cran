%% Description
%   Save the observer gain L_i, the consensus gain gamma*inv(P_i) and
%   the controller gain K_ij into files that can be loaded from Python.
%
%   A time-stamped folder "gain_for_python_YYYYMMDD_HHMMSS" is created
%   under the folder where this script is located. The following files
%   are written into that folder:
%     - K_all_vehicles.txt : JSON with keys K10, K20, K21, K30, K31, K32
%     - car1.yaml ~ car3.yaml : YAML configs for each vehicle
%
%   Usage in MATLAB (after running the gain-design script so that
%   L1, L2, L3, P1, P2, P3, gamma, K10, K20, K21, K30, K31, K32 exist
%   in the workspace):
%
%       save_gain2python

%% Validate required variables in the workspace
required_vars = {'L1','L2','L3', ...
				 'P1','P2','P3', ...
				 'gamma', ...
				 'K10','K20','K21','K30','K31','K32'};

missing = {};
for k = 1:numel(required_vars)
	name = required_vars{k};
	if ~exist(name,'var') || isempty(eval(name)) %#ok<EVLDIR>
		missing{end+1} = name; %#ok<AGROW>
	end
end

if ~isempty(missing)
	error(['Missing or empty variables: ' strjoin(missing, ', ') '.\n' ...
		   'Please run the gain design script (solve_gian_distribtued_observer.m) ' ...
		   'before calling save_gain2python.']);
end

%% Compute consensus gains gamma * inv(Pi)
G1 = gamma * inv(P1);
G2 = gamma * inv(P2);
G3 = gamma * inv(P3);

%% Prepare output directory (time-stamped under this script's folder)
scriptFullPath = mfilename('fullpath');
if isempty(scriptFullPath)
	scriptDir = pwd;
else
	scriptDir = fileparts(scriptFullPath);
end

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
outDirName = ['gain_for_python_' timestamp];
outDir = fullfile(scriptDir, outDirName);
if ~exist(outDir, 'dir')
	mkdir(outDir);
end

%% Save controller gains into a single TXT file (Python snippet format)
% The file K_all_vehicles.txt contains a Python code snippet:
%   self.K_all_vehicles = {
%       1: { 0: np.array([[... ]]) },
%       2: { 0: np.array([[... ]]), 1: np.array([[... ]]) },
%       3: { 0: np.array([[... ]]), 1: ..., 2: ... }
%   }
% with the numeric values taken from K10, K20, K21, K30, K31, K32.

fmtRow = @(K) formatRowForPython(K);
K10_str = fmtRow(K10);
K20_str = fmtRow(K20);
K21_str = fmtRow(K21);
K30_str = fmtRow(K30);
K31_str = fmtRow(K31);
K32_str = fmtRow(K32);

K_lines = {
	'self.K_all_vehicles = {', ...
	'    1: {', ...
	['        0: np.array([', K10_str, '])'], ...
	'    },', ...
	'    2: {', ...
	['        0: np.array([', K20_str, ']),'], ...
	['        1: np.array([', K21_str, '])'], ...
	'    },', ...
	'    3: {', ...
	['        0: np.array([', K30_str, ']),'], ...
	['        1: np.array([', K31_str, ']),'], ...
	['        2: np.array([', K32_str, '])'], ...
	'    }', ...
	'}' ...
};

Kfile = fullfile(outDir, 'K_all_vehicles.txt');
fidK = fopen(Kfile, 'w');
if fidK == -1
	error('Could not open file %s for writing.', Kfile);
end
for iLine = 1:numel(K_lines)
	fprintf(fidK, '%s\n', K_lines{iLine});
end
fclose(fidK);

%% Save YAML config files car1.yaml, car2.yaml, car3.yaml
% Each file has the structure:
%   observer:
%     observer_gain: [[...], [...], ...]
%     consensus_gain: [[...], [...], ...]

for iCar = 1:3
	switch iCar
		case 1
			Li = L1;
			Gi = G1;
		case 2
			Li = L2;
			Gi = G2;
		case 3
			Li = L3;
			Gi = G3;
	end

	observerStr = formatMatrixAsList(Li);
	consensusStr = formatMatrixAsList(Gi);

	yamlFile = fullfile(outDir, sprintf('car%d.yaml', iCar));
	fidYaml = fopen(yamlFile, 'w');
	if fidYaml == -1
		error('Could not open file %s for writing.', yamlFile);
	end

	% network block
	fprintf(fidYaml, 'network:\n');
	fprintf(fidYaml, '  car_id: %d\n', iCar);
	fprintf(fidYaml, '  base_port: 5000\n');

	% observer block
	fprintf(fidYaml, 'observer:\n');
	fprintf(fidYaml, '  fleet_estimator_type: distributed_luenberger\n');
	fprintf(fidYaml, '  local_estimator_type: ekf\n');
	fprintf(fidYaml, '  observer_gain: %s\n', observerStr);
	fprintf(fidYaml, '\n  consensus_gain: %s\n', consensusStr);
	fclose(fidYaml);
end

fprintf(['Observer, consensus, and controller gains saved to folder "%s" ' ...
	'(K_all_vehicles.txt and car1.yaml~car3.yaml).\n'], outDir);

%% Local helper function
function s = formatMatrixAsList(M)
	[rows, ~] = size(M);
	rowStrs = cell(rows, 1);
	for r = 1:rows
		vals = arrayfun(@(x) sprintf('%.4f', x), M(r, :), 'UniformOutput', false);
		rowStrs{r} = ['[', strjoin(vals, ','), ']'];
	end
	s = ['[', strjoin(rowStrs, ','), ']'];
end

function s = formatRowForPython(K)
	% Flatten to a row, round, and format as [k1,k2,k3,...]
	vals = round(K(:).', 4);
	strVals = arrayfun(@(x) sprintf('%.4f', x), vals, 'UniformOutput', false);
	s = ['[', strjoin(strVals, ','), ']'];
end

