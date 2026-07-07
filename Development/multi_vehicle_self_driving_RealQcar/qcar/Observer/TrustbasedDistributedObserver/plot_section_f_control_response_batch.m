function summaryAll = plot_section_f_control_response_batch(varargin)
%PLOT_SECTION_F_CONTROL_RESPONSE_BATCH Select many vehicle logs and summarize all.
%
% This wrapper calls plot_section_f_control_response once per selected CSV.
% It is useful for the paper table: include every host/vehicle log in the
% metric aggregation, while keeping one readable figure per host.
%
% Common usage:
%   plot_section_f_control_response_batch
%   plot_section_f_control_response_batch('ResultDate', '06-07-26')
%   plot_section_f_control_response_batch('TurnWindow', [12.0 18.5])
%   plot_section_f_control_response_batch('AttackWindow', [10 15; 21 26])
%   plot_section_f_control_response_batch('ShowControllerGaps', true)
%   plot_section_f_control_response_batch('Files', ["results/06-07-26/trust_weight_log_V1_16-10-42_211014.csv"; ...
%       "results/06-07-26/trust_weight_log_V2_16-10-42_211014.csv"])

parser = inputParser;
addParameter(parser, 'Files', strings(0, 1), @(x) isstring(x) || iscell(x) || ischar(x));
addParameter(parser, 'ResultDate', '', @(x) ischar(x) || isstring(x));
addParameter(parser, 'Attacker', NaN, @(x) isscalar(x) && isnumeric(x));
addParameter(parser, 'Peer', NaN, @(x) isscalar(x) && isnumeric(x));
addParameter(parser, 'SelectFiles', true, @(x) islogical(x) || isnumeric(x));
addParameter(parser, 'Save', true, @(x) islogical(x) || isnumeric(x));
addParameter(parser, 'OutputDir', '', @(x) ischar(x) || isstring(x));
addParameter(parser, 'Formats', "png", @(x) isstring(x) || iscell(x) || ischar(x));
addParameter(parser, 'AttackWindow', [NaN NaN], @isWindowSpec);
addParameter(parser, 'TurnWindow', [NaN NaN], @isWindowSpec);
addParameter(parser, 'ShowTurnSections', true, @(x) islogical(x) || isnumeric(x));
addParameter(parser, 'ShowControllerGaps', false, @(x) islogical(x) || isnumeric(x));
addParameter(parser, 'SpacingTolerance', 0.15, @(x) isscalar(x) && isnumeric(x));
addParameter(parser, 'GapStabilityStdMax', 0.25, @(x) isscalar(x) && isnumeric(x));
addParameter(parser, 'GoodCoveragePercent', 80.0, @(x) isscalar(x) && isnumeric(x));
addParameter(parser, 'FusionAlphaAttackMax', 0.50, @(x) isscalar(x) && isnumeric(x));
addParameter(parser, 'FusionAlphaDropMinPercent', 30.0, @(x) isscalar(x) && isnumeric(x));
parse(parser, varargin{:});
args = parser.Results;

scriptDir = fileparts(mfilename('fullpath'));
files = normalizeStringArray(args.Files);
if isempty(files)
    if ~logical(args.SelectFiles)
        error('No files were provided. Pass ''Files'' or enable ''SelectFiles''.');
    end
    files = selectManyTrustLogs(scriptDir, string(args.ResultDate));
end

summaryAll = table();
for k = 1:numel(files)
    fprintf('\n[%d/%d] Section F control plot: %s\n', k, numel(files), files(k));
    summaryOne = plot_section_f_control_response( ...
        'File', files(k), ...
        'Attacker', double(args.Attacker), ...
        'Peer', double(args.Peer), ...
        'Save', logical(args.Save), ...
        'OutputDir', args.OutputDir, ...
        'Formats', args.Formats, ...
        'AttackWindow', args.AttackWindow, ...
        'TurnWindow', args.TurnWindow, ...
        'ShowTurnSections', logical(args.ShowTurnSections), ...
        'ShowControllerGaps', logical(args.ShowControllerGaps), ...
        'SpacingTolerance', args.SpacingTolerance, ...
        'GapStabilityStdMax', args.GapStabilityStdMax, ...
        'GoodCoveragePercent', args.GoodCoveragePercent, ...
        'FusionAlphaAttackMax', args.FusionAlphaAttackMax, ...
        'FusionAlphaDropMinPercent', args.FusionAlphaDropMinPercent);
    summaryAll = [summaryAll; summaryOne]; %#ok<AGROW>
end

if logical(args.Save)
    outputDir = resolveBatchOutputDir(string(args.OutputDir), files(1), scriptDir);
    if ~isfolder(outputDir)
        mkdir(outputDir);
    end
    out = fullfile(outputDir, 'section_f_control_response_all_metrics.csv');
    writetable(summaryAll, out);
    fprintf('\nSaved combined Section F metrics to: %s\n', out);
end
end

function ok = isWindowSpec(x)
ok = isnumeric(x) && (isempty(x) || mod(numel(x), 2) == 0);
end

function files = selectManyTrustLogs(scriptDir, resultDate)
rootDir = fullfile(scriptDir, 'results');
resultDate = strtrim(resultDate);
if strlength(resultDate) > 0 && isfolder(fullfile(rootDir, resultDate))
    startDir = fullfile(rootDir, resultDate);
elseif isfolder(rootDir)
    startDir = rootDir;
else
    startDir = scriptDir;
end

[selected, selectedPath] = uigetfile(fullfile(startDir, 'trust_weight_log_V*.csv'), ...
    'Select one or more trust_weight_log CSV files', 'MultiSelect', 'on');
if isequal(selected, 0)
    error('File selection cancelled.');
end
if ischar(selected)
    selected = {selected};
end

files = strings(numel(selected), 1);
for k = 1:numel(selected)
    files(k) = string(fullfile(selectedPath, selected{k}));
end
end

function values = normalizeStringArray(values)
if ischar(values)
    values = string({values});
elseif iscell(values)
    values = string(values(:));
else
    values = string(values(:));
end
values = values(strlength(values) > 0);
end

function outputDir = resolveBatchOutputDir(outputDirArg, firstFile, scriptDir)
outputDirArg = strtrim(outputDirArg);
if strlength(outputDirArg) > 0
    outputDir = char(outputDirArg);
    if ~isfolder(outputDir) && ~contains(outputDir, filesep) && ~contains(outputDir, '/')
        outputDir = fullfile(scriptDir, outputDir);
    end
    return;
end

firstFile = char(firstFile);
if isfile(firstFile)
    outputDir = fullfile(fileparts(firstFile), 'section_f_analysis');
else
    candidate = fullfile(scriptDir, firstFile);
    if isfile(candidate)
        outputDir = fullfile(fileparts(candidate), 'section_f_analysis');
    else
        outputDir = fullfile(scriptDir, 'section_f_analysis');
    end
end
end
