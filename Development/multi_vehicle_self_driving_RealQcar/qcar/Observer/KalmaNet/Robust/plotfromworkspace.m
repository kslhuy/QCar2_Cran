%PLOTFROMWORKSPACE Plot RKNet comparator figures from workspace variables.
%
% Run after load_rknet_comparison_workspace. If you edit a copied column
% variable in the current MATLAB workspace, for example:
%
%   K_x_x(K_x_x > 0.2) = K_x_x(K_x_x > 0.2) + 0.1;
%
% this script uses that edited variable when generating the plots.
%
% Optional variables before running:
%   outputDir = "logs/plots";
%   noShow = true;
%   verboseWorkspaceOverrides = true;

close all

if ~exist("outputDir", "var")
    outputDir = "";
end
if ~exist("noShow", "var") || isempty(noShow)
    noShow = false;
end
if ~exist("verboseWorkspaceOverrides", "var") || isempty(verboseWorkspaceOverrides)
    verboseWorkspaceOverrides = true;
end

workspaceVars = captureCurrentWorkspace();
workspaceTable = buildWorkspaceComparatorTable(workspaceVars, verboseWorkspaceOverrides);

robustDir = fileparts(mfilename("fullpath"));
if strlength(string(robustDir)) == 0
    robustDir = pwd;
end

snapshotDir = fullfile(robustDir, "logs", "workspace");
if ~exist(snapshotDir, "dir")
    mkdir(snapshotDir);
end

snapshotName = "rknet_workspace_" + string(datetime("now", "Format", "yyyyMMdd_HHmmss")) + ".csv";
workspaceCsvPath = fullfile(snapshotDir, snapshotName);
writetable(workspaceTable, workspaceCsvPath);

fprintf("Saved workspace snapshot: %s\n", workspaceCsvPath);
workspaceGridPng = plotWorkspaceGridFigure(workspaceTable, workspaceCsvPath, outputDir, noShow, robustDir, workspaceVars);
rknetArtifacts = struct('workspace_grid_png', workspaceGridPng);
assignin("base", "rknetWorkspaceTable", workspaceTable);
assignin("base", "rknetWorkspaceCsvPath", workspaceCsvPath);
assignin("base", "rknetArtifacts", rknetArtifacts);

rknetArtifacts = plot_rknet_comparator(workspaceCsvPath, outputDir, noShow);
rknetArtifacts.workspace_grid_png = workspaceGridPng;
assignin("base", "rknetWorkspaceTable", workspaceTable);
assignin("base", "rknetWorkspaceCsvPath", workspaceCsvPath);
assignin("base", "rknetArtifacts", rknetArtifacts);

function workspaceVars = captureCurrentWorkspace()
names = evalin("caller", "who");
workspaceVars = struct();
for idx = 1:numel(names)
    name = names{idx};
    workspaceVars.(name) = evalin("caller", name);
end
end

function tableData = buildWorkspaceComparatorTable(workspaceVars, verboseWorkspaceOverrides)
if hasWorkspaceVar(workspaceVars, "rknetTable")
    tableData = getWorkspaceVar(workspaceVars, "rknetTable");
    if ~istable(tableData)
        error("Workspace variable rknetTable exists but is not a table.");
    end
    tableData = applyWorkspaceColumnOverrides(tableData, workspaceVars, verboseWorkspaceOverrides);
    return;
end

if hasWorkspaceVar(workspaceVars, "rknetData")
    data = getWorkspaceVar(workspaceVars, "rknetData");
    if ~isstruct(data)
        error("Workspace variable rknetData exists but is not a struct.");
    end
    tableData = structColumnsToTable(data);
    tableData = applyWorkspaceColumnOverrides(tableData, workspaceVars, verboseWorkspaceOverrides);
    return;
end

error("No rknetTable or rknetData found. Run load_rknet_comparison_workspace first.");
end

function tableData = applyWorkspaceColumnOverrides(tableData, workspaceVars, verboseWorkspaceOverrides)
rowCount = height(tableData);
originalNames = string(tableData.Properties.VariableNames);
workspaceNames = string(matlab.lang.makeUniqueStrings(matlab.lang.makeValidName(cellstr(originalNames))));
hasRknetData = hasWorkspaceVar(workspaceVars, "rknetData");
if hasRknetData
    rknetData = getWorkspaceVar(workspaceVars, "rknetData");
else
    rknetData = struct();
end

for idx = 1:numel(originalNames)
    originalName = char(originalNames(idx));
    workspaceName = char(workspaceNames(idx));
    originalValues = normalizeColumn(tableData.(originalName), rowCount);
    baseValues = [];
    structValues = [];
    hasBaseValues = false;
    hasStructValues = false;
    baseChanged = false;
    structChanged = false;

    if hasWorkspaceVar(workspaceVars, workspaceName)
        baseValues = getWorkspaceVar(workspaceVars, workspaceName);
        hasBaseValues = isCompatibleColumn(baseValues, rowCount);
        if hasBaseValues
            baseValues = normalizeColumn(baseValues, rowCount);
            baseChanged = ~columnsEqual(baseValues, originalValues);
        end
    end
    if isfield(rknetData, workspaceName)
        structValues = rknetData.(workspaceName);
        hasStructValues = isCompatibleColumn(structValues, rowCount);
        if hasStructValues
            structValues = normalizeColumn(structValues, rowCount);
            structChanged = ~columnsEqual(structValues, originalValues);
        end
    end

    values = [];
    sourceName = "";
    if hasBaseValues
        values = baseValues;
        sourceName = workspaceName;
    elseif hasStructValues
        values = structValues;
        sourceName = "rknetData." + string(workspaceName);
    else
        if hasWorkspaceVar(workspaceVars, workspaceName) || isfield(rknetData, workspaceName)
            warning("Skipping %s because its workspace size does not match %d rows.", workspaceName, rowCount);
        end
        continue;
    end

    tableData.(originalName) = values;
    if verboseWorkspaceOverrides && (baseChanged || structChanged)
        if hasBaseValues && structChanged && ~columnsEqual(baseValues, structValues)
            fprintf("Using standalone %s for CSV column %s; rknetData.%s is different and ignored.\n", ...
                workspaceName, originalName, workspaceName);
        elseif hasBaseValues && baseChanged
            fprintf("Using edited standalone %s for CSV column %s.\n", sourceName, originalName);
        else
            fprintf("Using edited %s for CSV column %s.\n", sourceName, originalName);
        end
    end
end
end

function tf = hasWorkspaceVar(workspaceVars, name)
tf = isstruct(workspaceVars) && isfield(workspaceVars, char(name));
end

function value = getWorkspaceVar(workspaceVars, name)
value = workspaceVars.(char(name));
end

function tableData = structColumnsToTable(data)
fieldNames = string(fieldnames(data));
rowCount = inferRowCount(data, fieldNames);
if rowCount == 0
    error("rknetData does not contain any vector columns.");
end

tableData = table();
for idx = 1:numel(fieldNames)
    fieldName = char(fieldNames(idx));
    values = data.(fieldName);
    if isCompatibleColumn(values, rowCount)
        tableData.(fieldName) = normalizeColumn(values, rowCount);
    end
end
end

function rowCount = inferRowCount(data, fieldNames)
rowCount = 0;
for idx = 1:numel(fieldNames)
    values = data.(char(fieldNames(idx)));
    if isvector(values) && numel(values) > rowCount
        rowCount = numel(values);
    end
end
end

function tf = isCompatibleColumn(values, rowCount)
tf = isvector(values) && numel(values) == rowCount;
end

function values = normalizeColumn(values, rowCount)
if isrow(values)
    values = values.';
end
if ischar(values)
    values = string(values);
end
values = reshape(values, rowCount, 1);
end

function tf = columnsEqual(a, b)
try
    if (isnumeric(a) || islogical(a)) && (isnumeric(b) || islogical(b))
        tf = isequaln(double(a), double(b));
        return;
    end
    tf = isequaln(string(a), string(b));
catch
    tf = false;
end
end

function pngPath = plotWorkspaceGridFigure(tableData, workspaceCsvPath, outputDir, noShow, robustDir, workspaceVars)
exportDir = resolveWorkspaceExportDir(outputDir, robustDir);
if ~exist(exportDir, "dir")
    mkdir(exportDir);
end

rowCount = height(tableData);
timeAxis = numericWorkspaceOrTableColumn(tableData, "timestamp", workspaceVars);
if all(~isfinite(timeAxis))
    timeAxis = (0:rowCount - 1).';
else
    timeAxis = timeAxis - timeAxis(1);
end

measMask = stackWorkspaceOrTableColumns(tableData, ["meas_mask_x", "meas_mask_y", "meas_mask_psi", "meas_mask_v"], workspaceVars);
innovation = stackWorkspaceOrTableColumns(tableData, ["innov_x", "innov_y", "innov_psi", "innov_v"], workspaceVars);
maskedInnovation = measMask .* innovation;

rknetGain = stackWorkspaceOrTableColumns(tableData, ["K_eff_x_x", "K_eff_y_y", "K_eff_psi_psi", "K_eff_v_v"], workspaceVars);
rknetGainTitle = "RKNet Effective Gain Diagonal";
rknetGainLabel = "RKNet K_eff";
if all(~isfinite(rknetGain(:)))
    rknetGain = stackWorkspaceOrTableColumns(tableData, ["K_x_x", "K_y_y", "K_psi_psi", "K_v_v"], workspaceVars);
    rknetGainTitle = "RKNet Gain Diagonal";
    rknetGainLabel = "RKNet K";
end
ekfGain = stackWorkspaceOrTableColumns(tableData, ["ekf_K_x_x", "ekf_K_y_y", "ekf_K_psi_psi", "ekf_K_v_v"], workspaceVars);
attackActive = numericWorkspaceOrTableColumn(tableData, "sensor_failure_active", workspaceVars) > 0.5;
attackLabels = attackLabelsFromTable(tableData, workspaceVars);

visibleState = "on";
if noShow
    visibleState = "off";
end

fig = figure( ...
    "Name", "RKNet Workspace 2x2 Diagnostics", ...
    "Color", "w", ...
    "Position", [120 80 1300 850], ...
    "Visible", visibleState);
safeWorkspaceTitle(fig, sprintf("Workspace 2x2 Diagnostics: %s", getFileName(workspaceCsvPath)));

channelNames = ["x", "y", "psi", "v"];
channelColors = [
    0.0000 0.4470 0.7410
    0.8500 0.3250 0.0980
    0.4660 0.6740 0.1880
    0.6350 0.0780 0.1840
];

ax = subplot(2, 2, 1, "Parent", fig);
plotChannels(ax, timeAxis, ekfGain, channelNames, channelColors, "EKF K");
finishWorkspaceAxes(ax, "EKF Gain Diagonal", "time [s]", "gain");
addWorkspaceAttackWindows(ax, timeAxis, attackActive, false, attackLabels);
legend(ax, "show", "FontSize", 8, "Location", "best");

ax = subplot(2, 2, 2, "Parent", fig);
plotChannels(ax, timeAxis, rknetGain, channelNames, channelColors, rknetGainLabel);
finishWorkspaceAxes(ax, rknetGainTitle, "time [s]", "gain");
addWorkspaceAttackWindows(ax, timeAxis, attackActive, false, attackLabels);
legend(ax, "show", "FontSize", 8, "Location", "best");

ax = subplot(2, 2, 3, "Parent", fig);
plotChannels(ax, timeAxis, measMask, channelNames, channelColors, "mask");
ylim(ax, [-0.05 1.05]);
finishWorkspaceAxes(ax, "Measurement Update Mask", "time [s]", "mask");
addWorkspaceAttackWindows(ax, timeAxis, attackActive, true, attackLabels);
legend(ax, "show", "FontSize", 8, "Location", "best");

ax = subplot(2, 2, 4, "Parent", fig);
plotChannels(ax, timeAxis, maskedInnovation, channelNames, channelColors, "masked innov");
yline(ax, 0.0, ":", "Color", [0 0 0], "LineWidth", 0.8, "HandleVisibility", "off");
finishWorkspaceAxes(ax, "Masked Innovation", "time [s]", "masked innovation");
addWorkspaceAttackWindows(ax, timeAxis, attackActive, false, attackLabels);
legend(ax, "show", "FontSize", 8, "Location", "best");

[~, baseName, ~] = fileparts(workspaceCsvPath);
pngPath = char(fullfile(exportDir, string(baseName) + "_workspace_grid.png"));
print(fig, pngPath, "-dpng", "-r180");
fprintf("Saved workspace 2x2 grid figure: %s\n", pngPath);

if noShow
    close(fig);
end
end

function exportDir = resolveWorkspaceExportDir(outputDir, robustDir)
if strlength(string(outputDir)) == 0
    exportDir = fullfile(robustDir, "logs", "plots");
elseif isAbsoluteWorkspacePath(outputDir)
    exportDir = char(string(outputDir));
else
    exportDir = fullfile(robustDir, outputDir);
end
end

function matrix = stackWorkspaceOrTableColumns(tableData, columns, workspaceVars)
rowCount = height(tableData);
matrix = nan(rowCount, numel(columns));
for idx = 1:numel(columns)
    matrix(:, idx) = numericWorkspaceOrTableColumn(tableData, columns(idx), workspaceVars);
end
end

function values = numericWorkspaceOrTableColumn(tableData, name, workspaceVars)
rowCount = height(tableData);
name = char(name);
if hasWorkspaceVar(workspaceVars, name)
    raw = getWorkspaceVar(workspaceVars, name);
    if isvector(raw) && numel(raw) == rowCount
        values = normalizeNumericVector(raw, rowCount);
        return;
    end
end

if ~ismember(name, tableData.Properties.VariableNames)
    values = nan(rowCount, 1);
    return;
end

values = normalizeNumericVector(tableData.(name), rowCount);
end

function values = normalizeNumericVector(raw, rowCount)
if isnumeric(raw) || islogical(raw)
    values = double(raw);
else
    values = str2double(string(raw));
end
values = values(:);
if numel(values) ~= rowCount
    values = nan(rowCount, 1);
end
end

function result = preferFinite(primary, fallback)
result = primary;
if ~isequal(size(primary), size(fallback))
    result = fallback;
    return;
end
missing = ~isfinite(result);
result(missing) = fallback(missing);
end

function plotChannels(ax, timeAxis, values, channelNames, channelColors, labelPrefix)
for idx = 1:numel(channelNames)
    plot(ax, timeAxis, values(:, idx), ...
        "DisplayName", sprintf("%s %s", labelPrefix, channelNames(idx)), ...
        "LineWidth", 1.2, ...
        "Color", channelColors(idx, :));
    hold(ax, "on");
end
end

function addWorkspaceAttackWindows(ax, timeAxis, attackActive, labelFirst, attackLabels)
if nargin < 5
    attackLabels = strings(numel(timeAxis), 1);
end
segments = attackSegmentsFromMask(timeAxis, attackActive);
if isempty(segments)
    return;
end

yl = ylim(ax);
hold(ax, "on");
shownLegendLabels = strings(0, 1);
for idx = 1:size(segments, 1)
    label = dominantAttackLabel(attackLabels, attackActive, timeAxis, segments(idx, :));
    cleanLabel = normalizeAttackLabel(label);
    attackColor = attackColorForLabel(cleanLabel);
    displayName = "";
    handleVisibility = "off";
    if labelFirst
        legendLabel = attackLegendLabel(cleanLabel);
        if ~any(shownLegendLabels == legendLabel)
            displayName = legendLabel;
            handleVisibility = "on";
            shownLegendLabels(end + 1) = legendLabel; %#ok<AGROW>
        end
    end
    patchHandle = patch(ax, ...
        [segments(idx, 1) segments(idx, 2) segments(idx, 2) segments(idx, 1)], ...
        [yl(1) yl(1) yl(2) yl(2)], ...
        attackColor, ...
        "FaceAlpha", 0.10, ...
        "EdgeColor", "none", ...
        "DisplayName", displayName, ...
        "HandleVisibility", handleVisibility);
    try
        uistack(patchHandle, "bottom");
    catch
    end
    if strlength(cleanLabel) > 0
        text(ax, mean(segments(idx, :)), yl(1), char(cleanLabel), ...
            "HorizontalAlignment", "center", ...
            "VerticalAlignment", "bottom", ...
            "FontSize", 7, ...
            "Color", attackColor, ...
            "Interpreter", "none", ...
            "BackgroundColor", [1 1 1], ...
            "Margin", 1, ...
            "HandleVisibility", "off");
    end
end
ylim(ax, yl);
end

function attackLabels = attackLabelsFromTable(tableData, workspaceVars)
rowCount = height(tableData);
branchTypes = stringWorkspaceOrTableColumn(tableData, "sensor_failure_branch_types", workspaceVars);
gpsTypes = stringWorkspaceOrTableColumn(tableData, "sensor_failure_gps_type", workspaceVars);
attackLabels = strings(rowCount, 1);
for idx = 1:rowCount
    attackLabels(idx) = attackTypeLabel(branchTypes(idx), gpsTypes(idx));
end
end

function values = stringWorkspaceOrTableColumn(tableData, name, workspaceVars)
rowCount = height(tableData);
name = char(name);
if hasWorkspaceVar(workspaceVars, name)
    raw = getWorkspaceVar(workspaceVars, name);
    if isvector(raw) && numel(raw) == rowCount
        values = normalizeStringVector(raw, rowCount);
        return;
    end
end

if ~ismember(name, tableData.Properties.VariableNames)
    values = strings(rowCount, 1);
    return;
end

values = normalizeStringVector(tableData.(name), rowCount);
end

function values = normalizeStringVector(raw, rowCount)
values = string(raw);
values = values(:);
if numel(values) ~= rowCount
    values = strings(rowCount, 1);
end
values(ismissing(values)) = "";
end

function label = dominantAttackLabel(attackLabels, attackActive, timeAxis, segmentTimes)
segmentMask = attackActive(:) ...
    & timeAxis(:) >= segmentTimes(1) ...
    & timeAxis(:) <= segmentTimes(2);
labels = strtrim(string(attackLabels(segmentMask)));
labels(ismissing(labels)) = "";
labels(labels == "" | lower(labels) == "none") = [];
if isempty(labels)
    label = "";
    return;
end

uniqueLabels = unique(labels);
counts = zeros(numel(uniqueLabels), 1);
for idx = 1:numel(uniqueLabels)
    counts(idx) = nnz(labels == uniqueLabels(idx));
end
[~, maxIdx] = max(counts);
label = uniqueLabels(maxIdx);
end

function label = normalizeAttackLabel(label)
label = strtrim(string(label));
if ismissing(label) || label == "" || lower(label) == "nan" || lower(label) == "none"
    label = "";
end
end

function legendLabel = attackLegendLabel(label)
label = normalizeAttackLabel(label);
if strlength(label) == 0
    legendLabel = "attack interval";
else
    legendLabel = "attack: " + label;
end
end

function color = attackColorForLabel(label)
label = normalizeAttackLabel(label);
if strlength(label) == 0
    color = [0.5000 0.5000 0.5000];
    return;
end

palette = [
    0.8500 0.3250 0.0980
    0.4940 0.1840 0.5560
    0.9290 0.6940 0.1250
    0.4660 0.6740 0.1880
    0.3010 0.7450 0.9330
    0.6350 0.0780 0.1840
    0.0000 0.4470 0.7410
    0.7500 0.0000 0.7500
    0.2500 0.2500 0.2500
];
chars = double(char(lower(label)));
weights = 1:numel(chars);
paletteIdx = mod(sum(chars .* weights), size(palette, 1)) + 1;
color = palette(paletteIdx, :);
end

function label = attackTypeLabel(branchType, gpsType)
labels = strings(0, 1);
branchLabel = cleanWorkspaceLabel(branchType);
gpsLabel = cleanWorkspaceLabel(gpsType);
if strlength(branchLabel) > 0
    labels(end + 1) = branchLabel; %#ok<AGROW>
end
if strlength(gpsLabel) > 0
    labels(end + 1) = "gps:" + gpsLabel; %#ok<AGROW>
end
if isempty(labels)
    label = "none";
else
    label = strjoin(labels, " + ");
end
end

function label = cleanWorkspaceLabel(value)
label = strtrim(string(value));
if ismissing(label) || label == "" || lower(label) == "nan" || lower(label) == "none"
    label = "";
end
end

function segments = attackSegmentsFromMask(timeAxis, attackActive)
attackActive = attackActive(:);
timeAxis = timeAxis(:);
segments = zeros(0, 2);
startIdx = [];
for idx = 1:numel(attackActive)
    if attackActive(idx) && isempty(startIdx)
        startIdx = idx;
    elseif ~attackActive(idx) && ~isempty(startIdx)
        segments(end + 1, :) = [timeAxis(startIdx), timeAxis(idx - 1)]; %#ok<AGROW>
        startIdx = [];
    end
end
if ~isempty(startIdx)
    segments(end + 1, :) = [timeAxis(startIdx), timeAxis(end)]; %#ok<AGROW>
end
end

function finishWorkspaceAxes(ax, titleText, xText, yText)
title(ax, titleText, "Interpreter", "none");
xlabel(ax, xText);
ylabel(ax, yText);
grid(ax, "on");
box(ax, "on");
end

function safeWorkspaceTitle(fig, textValue)
try
    sgtitle(fig, textValue, "FontSize", 14, "Interpreter", "none");
catch
    annotation(fig, "textbox", [0 0.96 1 0.04], ...
        "String", textValue, ...
        "EdgeColor", "none", ...
        "HorizontalAlignment", "center", ...
        "FontSize", 14, ...
        "Interpreter", "none");
end
end

function fileName = getFileName(pathValue)
[~, name, ext] = fileparts(pathValue);
fileName = string(name) + string(ext);
end

function tf = isAbsoluteWorkspacePath(pathValue)
pathValue = string(pathValue);
tf = startsWith(pathValue, filesep) ...
    || ~isempty(regexp(char(pathValue), "^[A-Za-z]:[\\/]", "once")) ...
    || startsWith(pathValue, "\\");
end
