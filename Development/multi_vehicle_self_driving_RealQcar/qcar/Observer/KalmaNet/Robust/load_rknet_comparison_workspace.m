function [rknetTable, rknetData, timeSeconds, csvPath] = load_rknet_comparison_workspace(csvName, copyColumnsToBaseWorkspace)
%LOAD_RKNET_COMPARISON_WORKSPACE Load one RKNet comparator CSV into MATLAB.
%
% Usage:
%   load_rknet_comparison_workspace
%   load_rknet_comparison_workspace("rknet_comparison_20260501_221129.csv")
%   load_rknet_comparison_workspace("C:\path\to\rknet_comparison_20260501_221129.csv")
%
% Output variables are also assigned to the base workspace:
%   rknetTable  - full readtable result
%   rknetData   - struct with one field per CSV column
%   timeSeconds - timestamp normalized to start at zero
%   csvPath     - resolved CSV path

if nargin < 1 || strlength(strtrim(string(csvName))) == 0
    csvName = "rknet_comparison_20260501_221129.csv";
end
if nargin < 2
    copyColumnsToBaseWorkspace = true;
end

robustDir = fileparts(mfilename("fullpath"));
if strlength(string(robustDir)) == 0
    robustDir = pwd;
end
logDir = fullfile(robustDir, "logs", "comparator");
csvPath = resolveCsvPath(csvName, robustDir, logDir);

opts = detectImportOptions(csvPath, ...
    "FileType", "text", ...
    "Delimiter", ",", ...
    "VariableNamingRule", "preserve");

textColumns = [
    "source"
    "mask_selected_branch"
    "sensor_failure_branches"
    "sensor_failure_branch_types"
    "sensor_failure_gps_type"
    "K_matrix_json"
    "K_effective_matrix_json"
    "ekf_K_matrix_json"
    "ekf_K_measurement_labels"
    "meas_mask_json"
    "innovation_json"
    "masked_innovation_json"
    "update_correction_json"
];
presentText = intersect(textColumns, string(opts.VariableNames), "stable");
if ~isempty(presentText)
    opts = setvartype(opts, cellstr(presentText), "string");
end

rknetTable = readtable(csvPath, opts);
if height(rknetTable) == 0
    error("No data rows found in %s", csvPath);
end

rknetData = tableToStructOfColumns(rknetTable);
if isfield(rknetData, "timestamp")
    timeSeconds = double(rknetData.timestamp) - double(rknetData.timestamp(1));
else
    timeSeconds = (0:height(rknetTable) - 1).';
end

assignin("base", "rknetTable", rknetTable);
assignin("base", "rknetData", rknetData);
assignin("base", "timeSeconds", timeSeconds);
assignin("base", "csvPath", csvPath);

if copyColumnsToBaseWorkspace
    originalNames = string(rknetTable.Properties.VariableNames);
    workspaceNames = string(matlab.lang.makeUniqueStrings(matlab.lang.makeValidName(cellstr(originalNames))));
    for idx = 1:numel(originalNames)
        assignin("base", char(workspaceNames(idx)), rknetTable.(char(originalNames(idx))));
    end
end

fprintf("Loaded %d rows from %s\n", height(rknetTable), csvPath);
fprintf("Base workspace variables: rknetTable, rknetData, timeSeconds, csvPath");
if copyColumnsToBaseWorkspace
    fprintf(", plus one variable per CSV column");
end
fprintf(".\n");
end

function data = tableToStructOfColumns(tableData)
data = struct();
names = string(tableData.Properties.VariableNames);
fieldNames = string(matlab.lang.makeUniqueStrings(matlab.lang.makeValidName(cellstr(names))));
for idx = 1:numel(names)
    data.(char(fieldNames(idx))) = tableData.(char(names(idx)));
end
end

function csvPath = resolveCsvPath(csvName, robustDir, logDir)
csvName = string(csvName);
candidates = buildCandidates(csvName, robustDir, logDir);
for idx = 1:numel(candidates)
    if exist(candidates(idx), "file")
        csvPath = char(candidates(idx));
        return;
    end
end
error("Comparator CSV not found: %s", csvName);
end

function candidates = buildCandidates(csvName, robustDir, logDir)
withExt = csvName;
if ~endsWith(lower(csvName), ".csv")
    withExt = csvName + ".csv";
end

if isAbsolutePath(csvName)
    candidates = unique([csvName, withExt], "stable");
    return;
end

candidates = [
    fullfile(pwd, csvName)
    fullfile(pwd, withExt)
    fullfile(logDir, csvName)
    fullfile(logDir, withExt)
    fullfile(robustDir, csvName)
    fullfile(robustDir, withExt)
];
candidates = unique(candidates, "stable");
end

function tf = isAbsolutePath(pathValue)
pathValue = string(pathValue);
tf = startsWith(pathValue, filesep) ...
    || ~isempty(regexp(char(pathValue), "^[A-Za-z]:[\\/]", "once")) ...
    || startsWith(pathValue, "\\");
end
