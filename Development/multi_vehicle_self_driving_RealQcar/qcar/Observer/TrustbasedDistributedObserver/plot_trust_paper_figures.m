function summary = plot_trust_paper_figures(varargin)
%PLOT_TRUST_PAPER_FIGURES Generate paper-oriented comparison figures.
%
% Default usage from this folder starts interactive file selection at results/:
%   plot_trust_paper_figures
%
% Common usage:
%   plot_trust_paper_figures('Focus', 0)
%   plot_trust_paper_figures('Focus', 2)
%   plot_trust_paper_figures('ResultDate', '02-07-26', 'Focus', 0)
%   plot_trust_paper_figures('SelectFiles', false, 'Focus', 0)
%   plot_trust_paper_figures('Focus', 0, 'TimeWindow', [-2 10])
%   plot_trust_paper_figures('Focus', 0, 'TimeWindow', [])  % full run
%   plot_trust_paper_figures('Save', true, 'OutputDir', 'paper_figures')
%
% Explicit file mapping:
%   Put the files in this order: no rollback, rollback, anchoring.
%   files = { ...
%       'results/02-07-26/case2/trust_weight_log_V1_15-32-26_977725.csv', ...
%       'results/02-07-26/case2/trust_weight_log_V1_15-33-59_274476.csv', ...
%       'results/02-07-26/case2/trust_weight_log_V1_15-35-25_102224.csv'};
%   plot_trust_paper_figures('Files', files, 'Focus', 0)
%
% Time-series figures use time relative to attack start (t - t_attack).
%
% Figures exported:
%   1. Trajectory XY
%   2. State estimation
%   3. Position estimation error with reference
%   4. Estimation error and rollback correction
%   5. Trust
%   6. Weights

close all

parser = inputParser;
addParameter(parser, 'Files', strings(0, 1), @(x) isstring(x) || iscell(x) || ischar(x));
addParameter(parser, 'CaseNames', strings(0, 1), @(x) isstring(x) || iscell(x) || ischar(x));
addParameter(parser, 'Focus', 0, @(x) isscalar(x) && isnumeric(x));
addParameter(parser, 'Host', 1, @(x) isscalar(x) && isnumeric(x));
addParameter(parser, 'ResultDate', '', @(x) ischar(x) || isstring(x));
addParameter(parser, 'SelectFiles', true, @(x) islogical(x) || isnumeric(x));
addParameter(parser, 'Save', true, @(x) islogical(x) || isnumeric(x));
addParameter(parser, 'OutputDir', 'paper_figures', @(x) ischar(x) || isstring(x));
addParameter(parser, 'Formats', ["png", "pdf"], @(x) isstring(x) || iscell(x) || ischar(x));
addParameter(parser, 'TimeWindow', [-2 10], @(x) isempty(x) || (isnumeric(x) && numel(x) == 2));
addParameter(parser, 'Dpi', 600, @(x) isscalar(x) && isnumeric(x));
parse(parser, varargin{:});
args = parser.Results;

scriptDir = fileparts(mfilename('fullpath'));
focus = double(args.Focus);
caseNames = normalizeStringArray(args.CaseNames);
if isempty(caseNames)
    caseNames = [
        "No rollback, pure prediction"
        "Rollback, pure prediction"
        "Rollback, prediction + anchoring"];
end

files = normalizeStringArray(args.Files);
if isempty(files) && logical(args.SelectFiles)
    files = selectPaperCaseFiles(scriptDir, string(args.ResultDate), double(args.Host), focus, caseNames);
elseif isempty(files)
    files = defaultCaseFiles(scriptDir, string(args.ResultDate), double(args.Host), focus);
end
if numel(files) ~= numel(caseNames)
    error('Number of files (%d) must match number of case names (%d).', numel(files), numel(caseNames));
end

cases = loadCases(files, caseNames, scriptDir, focus);
outputDir = resolveOutputDir(args.OutputDir, scriptDir);
formats = normalizeStringArray(args.Formats);
timeWindow = double(args.TimeWindow);
overviewTimeWindow = [0 20];

style = paperStyle();
set(groot, 'defaultAxesFontName', style.fontName);
set(groot, 'defaultTextFontName', style.fontName);

summary = buildSummary(cases, focus);
printSummary(summary, focus);
if logical(args.Save)
    ensureFolder(outputDir);
    writetable(summary, fullfile(outputDir, sprintf('rollback_correction_summary_V%d.csv', focus)));
end

figTrajectory = makeTrajectoryFigure(cases, focus, style, overviewTimeWindow);
finishFigure(figTrajectory, 'paper_trajectory_xy', outputDir, formats, logical(args.Save), args.Dpi);

figStates = makeStateFigure(cases, focus, style, overviewTimeWindow);
finishFigure(figStates, 'paper_state_estimation', outputDir, formats, logical(args.Save), args.Dpi);

figPositionError = makePositionErrorFigure(cases, focus, style, timeWindow);
finishFigure(figPositionError, 'paper_position_error_reference', outputDir, formats, logical(args.Save), args.Dpi);

figRollback = makeRollbackFigure(cases, focus, style, timeWindow);
finishFigure(figRollback, 'paper_rollback_correction', outputDir, formats, logical(args.Save), args.Dpi);

figTrust = makeTrustFigure(cases, focus, style, timeWindow);
finishFigure(figTrust, 'paper_trust', outputDir, formats, logical(args.Save), args.Dpi);

figWeights = makeWeightsFigure(cases, focus, style, timeWindow);
finishFigure(figWeights, 'paper_weights', outputDir, formats, logical(args.Save), args.Dpi);

if logical(args.Save)
    fprintf('Saved paper figures to: %s\n', outputDir);
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

function files = defaultCaseFiles(scriptDir, resultDate, ~, ~)
resultDate = strtrim(string(resultDate));
if strlength(resultDate) == 0
    resultDate = "02-07-26";
end

exactFiles = [
    fullfile('results', resultDate, 'case2', 'trust_weight_log_V1_15-38-32_870749.csv')
    fullfile('results', resultDate, 'case2', 'trust_weight_log_V1_15-40-04_038956.csv')
    fullfile('results', resultDate, 'case2', 'trust_weight_log_V1_15-41-09_933192.csv')];

if all(isfile(fullfile(scriptDir, exactFiles)))
    files = exactFiles;
    return;
end

error(['Default paper comparison files are missing. Pass ''Files'' explicitly ', ...
    'in this order: no rollback, rollback, anchoring.']);
end

function files = selectPaperCaseFiles(scriptDir, resultDate, hostId, focus, caseNames)
rootDir = fullfile(scriptDir, 'results');
resultDate = strtrim(string(resultDate));
if strlength(resultDate) == 0
    startDir = rootDir;
else
    startDir = fullfile(rootDir, resultDate);
    if ~isfolder(startDir)
        startDir = rootDir;
    end
end
if ~isfolder(startDir)
    error('Results folder does not exist: %s', rootDir);
end

files = strings(numel(caseNames), 1);
for k = 1:numel(caseNames)
    selected = browseTrustLogFile(startDir, rootDir, scriptDir, hostId, focus, caseNames(k));
    files(k) = erase(string(selected), string(scriptDir) + filesep);
end
fprintf('\nSelected paper comparison files:\n');
for k = 1:numel(files)
    fprintf('  %s -> %s\n', caseNames(k), files(k));
end
fprintf('\n');
end

function selectedFile = browseTrustLogFile(startDir, rootDir, scriptDir, hostId, focus, caseName)
currentDir = startDir;
selectedFile = '';
fprintf('\nSelect file for "%s"\n', caseName);
fprintf('Type a number to enter a folder or select a file. Type u to go up, q to cancel.\n');

while strlength(string(selectedFile)) == 0
    [dirs, csvFiles] = folderChoices(currentDir, hostId);
    relDir = erase(string(currentDir), string(scriptDir) + filesep);
    fprintf('\nCurrent folder: %s\n', relDir);
    fprintf('--------------------------------------------------------------------------------\n');
    for i = 1:numel(dirs)
        fprintf('[%2d] <DIR>  %s\n', i - 1, dirs(i).name);
    end
    for i = 1:numel(csvFiles)
        idx = numel(dirs) + i - 1;
        filepath = fullfile(csvFiles(i).folder, csvFiles(i).name);
        info = candidateSummary(filepath, focus);
        fprintf('[%2d] FILE   rb=%d rows=%2d | RMSE=%.3f m max=%.3f m | %s\n', ...
            idx, info.rollback_enabled, info.rollback_rows, ...
            info.final_attack_rmse_m, info.final_attack_max_m, csvFiles(i).name);
    end
    if isempty(dirs) && isempty(csvFiles)
        fprintf('No subfolders or trust_weight_log_V%d*.csv files here.\n', hostId);
    end
    fprintf('--------------------------------------------------------------------------------\n');

    choice = strtrim(input(sprintf('Choice for "%s": ', caseName), 's'));
    if any(strcmpi(choice, {'q', 'quit', 'cancel'}))
        error('File selection cancelled.');
    elseif any(strcmpi(choice, {'u', '..', 'back'}))
        parentDir = fileparts(currentDir);
        if isInsideFolder(parentDir, rootDir)
            currentDir = parentDir;
        else
            fprintf('Already at top results folder.\n');
        end
        continue;
    end

    idx = str2double(choice);
    if ~isfinite(idx) || idx < 0 || idx >= numel(dirs) + numel(csvFiles)
        fprintf('Invalid choice: %s\n', choice);
        continue;
    end

    idx = idx + 1;
    if idx <= numel(dirs)
        currentDir = fullfile(dirs(idx).folder, dirs(idx).name);
    else
        fileIdx = idx - numel(dirs);
        selectedFile = fullfile(csvFiles(fileIdx).folder, csvFiles(fileIdx).name);
        fprintf('Selected: %s\n', erase(string(selectedFile), string(scriptDir) + filesep));
    end
end
end

function [dirs, csvFiles] = folderChoices(currentDir, hostId)
entries = dir(currentDir);
isUsefulDir = [entries.isdir] & ~ismember({entries.name}, {'.', '..'});
dirs = entries(isUsefulDir);
csvFiles = dir(fullfile(currentDir, sprintf('trust_weight_log_V%d*.csv', hostId)));
csvFiles = csvFiles(~[csvFiles.isdir]);

if ~isempty(dirs)
    [~, idx] = sort(lower(string({dirs.name})));
    dirs = dirs(idx);
end
if ~isempty(csvFiles)
    [~, idx] = sort([csvFiles.datenum], 'ascend');
    csvFiles = csvFiles(idx);
end
end

function tf = isInsideFolder(pathToCheck, rootDir)
pathToCheck = lower(char(pathToCheck));
rootDir = lower(char(rootDir));
tf = startsWith(pathToCheck, rootDir);
end

function info = candidateSummary(filepath, focus)
try
    tbl = readTrustTable(filepath);
    [attackStart_s, attackEnd_s] = attackWindow(tbl, focus);
    metrics = attackMetrics(tbl, focus, attackStart_s, attackEnd_s);
    rb = col(tbl, 'rollback_triggered');
    enabled = maxFinite(col(tbl, 'rollback_enabled'));
    info.rollback_enabled = isfinite(enabled) && enabled > 0;
    info.rollback_rows = nnz(isfinite(rb) & rb > 0);
    info.final_attack_rmse_m = metrics.est_attack_rmse_m;
    info.final_attack_max_m = metrics.est_attack_max_m;
catch
    info.rollback_enabled = false;
    info.rollback_rows = 0;
    info.final_attack_rmse_m = NaN;
    info.final_attack_max_m = NaN;
end
end

function cases = loadCases(files, caseNames, scriptDir, focus)
cases = repmat(struct(), numel(files), 1);
for i = 1:numel(files)
    filepath = resolveInputFile(files(i), scriptDir);
    tbl = readTrustTable(filepath);
    cols = string(tbl.Properties.VariableNames);
    if ~any(cols == "time")
        error('Missing time column in %s.', filepath);
    end
    if ~any(cols == sprintf('est_x_%d', focus))
        error('Focus vehicle V%d is not present in %s.', focus, filepath);
    end

    cases(i).name = caseNames(i);
    cases(i).shortName = sprintf('C%d', i);
    cases(i).file = filepath;
    cases(i).relativeFile = erase(string(filepath), string(scriptDir) + filesep);
    cases(i).table = tbl;
    cases(i).rawTime = col(tbl, 'time');
    [cases(i).attackStart_s, cases(i).attackEnd_s] = attackWindow(tbl, focus);
    cases(i).time = cases(i).rawTime - cases(i).attackStart_s;
    cases(i).event = correctionEvent(tbl, focus, cases(i).attackStart_s);
    cases(i).metrics = attackMetrics(tbl, focus, cases(i).attackStart_s, cases(i).attackEnd_s);
end
end

function filepath = resolveInputFile(fileArg, scriptDir)
filepath = char(fileArg);
if isfile(filepath)
    return;
end

candidate = fullfile(scriptDir, filepath);
if isfile(candidate)
    filepath = candidate;
    return;
end

candidate = fullfile(scriptDir, 'results', filepath);
if isfile(candidate)
    filepath = candidate;
    return;
end

error('File does not exist: %s', filepath);
end

function tbl = readTrustTable(filepath)
opts = detectImportOptions(filepath, 'VariableNamingRule', 'preserve');
try
    opts = setvaropts(opts, opts.VariableNames, 'WhitespaceRule', 'preserve');
catch
end
try
    opts = setvaropts(opts, opts.VariableNames, 'EmptyFieldRule', 'auto');
catch
end
tbl = readtable(filepath, opts);
end

function [attackStart_s, attackEnd_s] = attackWindow(tbl, focus)
startCandidates = [
    firstFinite(col(tbl, 'v2v_attack_start_s'))
    firstFinite(col(tbl, sprintf('inject_attack_start_%d', focus)))];
endCandidates = [
    firstFinite(col(tbl, 'v2v_attack_end_s'))
    firstFinite(col(tbl, sprintf('inject_attack_end_%d', focus)))];

attackStart_s = firstFinite(startCandidates);
attackEnd_s = firstFinite(endCandidates);
if ~isfinite(attackStart_s)
    attackStart_s = 0;
end
if ~isfinite(attackEnd_s) || attackEnd_s <= attackStart_s
    attackEnd_s = NaN;
end
end

function metrics = attackMetrics(tbl, focus, attackStart_s, attackEnd_s)
t = col(tbl, 'time');
post = col(tbl, sprintf('postpred_pos_err_%d', focus));
est = col(tbl, sprintf('est_pos_err_%d', focus));

if isfinite(attackEnd_s)
    mask = t >= attackStart_s & t <= attackEnd_s;
else
    mask = t >= attackStart_s;
end
maskPost = mask & isfinite(post);
maskEst = mask & isfinite(est);

metrics = struct();
metrics.post_attack_rmse_m = rmse(post(maskPost));
metrics.est_attack_rmse_m = rmse(est(maskEst));
metrics.post_attack_max_m = maxOrNan(post(maskPost));
metrics.est_attack_max_m = maxOrNan(est(maskEst));
metrics.attack_duration_s = attackEnd_s - attackStart_s;
end

function outputDir = resolveOutputDir(outputDirArg, scriptDir)
outputDir = char(outputDirArg);
if isempty(outputDir)
    outputDir = fullfile(scriptDir, 'paper_figures');
elseif ~isfolder(outputDir) && ~contains(outputDir, filesep) && ~contains(outputDir, '/')
    outputDir = fullfile(scriptDir, outputDir);
end
end

function ensureFolder(folder)
if ~isfolder(folder)
    mkdir(folder);
end
end

function style = paperStyle()
style.fontName = 'Times New Roman';
style.fontSize = 9;
style.titleSize = 10;
style.lineWidth = 1.25;
style.refWidth = 1.5;
style.markerSize = 5;
style.colors = [
    0.0000 0.4470 0.7410
    0.8500 0.3250 0.0980
    0.4660 0.6740 0.1880
    0.4940 0.1840 0.5560
    0.3010 0.7450 0.9330];
style.refColor = [0.05 0.05 0.05];
style.postColor = [0.78 0.16 0.16];
style.estColor = [0.00 0.30 0.70];
style.eventColor = [0.15 0.15 0.15];
style.attackColor = [0.45 0.45 0.45];
style.gridAlpha = 0.20;
end

function fig = makeTrajectoryFigure(cases, focus, style, timeWindow)
n = numel(cases);
fig = figure('Name', 'Paper Trajectory XY', 'Color', 'w', ...
    'Units', 'centimeters', 'Position', [2 2 18 5.8]);
layout = tiledlayout(fig, 1, n, 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:n
    ax = nexttile(layout);
    tbl = cases(i).table;
    hold(ax, 'on');
    t = cases(i).time;
    plotXY(ax, tbl, focus, t, timeWindow, 'ref', 'Reference', style.refColor, '-', style.refWidth);
    plotXY(ax, tbl, focus, t, timeWindow, 'postpred', 'Post-prediction', style.postColor, '--', style.lineWidth);
    plotXY(ax, tbl, focus, t, timeWindow, 'est', 'Final estimate', style.estColor, '-', style.lineWidth);
    markTrajectoryEvent(ax, cases(i), focus, style, timeWindow);
    axis(ax, 'equal');
    styleAxes(ax, sprintf('(%c) %s', char('a' + i - 1), cases(i).name), 'x [m]', 'y [m]', style);
    if i == 1
        legend(ax, 'Location', 'best', 'FontSize', style.fontSize - 1);
    end
end
title(layout, sprintf('Trajectory comparison for target V%d', focus), ...
    'FontName', style.fontName, 'FontSize', style.titleSize, 'FontWeight', 'bold');
end

function plotXY(ax, tbl, focus, t, timeWindow, prefix, labelText, color, lineStyle, lineWidth)
x = col(tbl, sprintf('%s_x_%d', prefix, focus));
y = col(tbl, sprintf('%s_y_%d', prefix, focus));
mask = isfinite(x) & isfinite(y) & timeWindowMask(t, timeWindow);
if any(mask)
    plot(ax, x(mask), y(mask), 'DisplayName', labelText, ...
        'Color', color, 'LineStyle', lineStyle, 'LineWidth', lineWidth);
end
end

function markTrajectoryEvent(ax, caseData, focus, style, timeWindow)
event = caseData.event;
if ~isfinite(event.index) || event.correction_m <= 0
    return;
end
if ~isTimeInWindow(event.time_from_attack_s, timeWindow)
    return;
end
tbl = caseData.table;
x = col(tbl, sprintf('est_x_%d', focus));
y = col(tbl, sprintf('est_y_%d', focus));
idx = event.index;
if idx >= 1 && idx <= numel(x) && isfinite(x(idx)) && isfinite(y(idx))
    plot(ax, x(idx), y(idx), 'o', 'MarkerSize', style.markerSize, ...
        'MarkerEdgeColor', style.eventColor, 'MarkerFaceColor', style.eventColor, ...
        'DisplayName', 'Max correction');
end
end

function fig = makeStateFigure(cases, focus, style, timeWindow)
states = ["x", "y", "theta", "v", "a"];
ylabels = ["x [m]", "y [m]", "theta [rad]", "v [m/s]", "a [m/s^2]"];
n = numel(cases);
fig = figure('Name', 'Paper State Estimation', 'Color', 'w', ...
    'Units', 'centimeters', 'Position', [2 2 20 16]);
layout = tiledlayout(fig, numel(states), n, 'TileSpacing', 'compact', 'Padding', 'compact');

for s = 1:numel(states)
    for i = 1:numel(cases)
        ax = nexttile(layout, (s - 1) * n + i);
        tbl = cases(i).table;
        t = cases(i).time;
        hold(ax, 'on');
        ref = col(tbl, sprintf('ref_%s_%d', states(s), focus));
        post = col(tbl, sprintf('postpred_%s_%d', states(s), focus));
        est = col(tbl, sprintf('est_%s_%d', states(s), focus));
        if any(isfinite(ref))
            plot(ax, t, ref, 'Color', style.refColor, 'LineWidth', style.refWidth, ...
                'DisplayName', 'Reference');
        end
        if any(isfinite(post))
            plot(ax, t, post, '--', 'Color', style.postColor, 'LineWidth', style.lineWidth, ...
                'DisplayName', 'Post-prediction');
        end
        if any(isfinite(est))
            plot(ax, t, est, '-', 'Color', style.estColor, 'LineWidth', style.lineWidth, ...
                'DisplayName', 'Final estimate');
        end
        markAttackAndRollback(ax, cases(i), style);
        applyTimeWindow(ax, timeWindow);
        if s == 1
            titleText = sprintf('(%c) %s', char('a' + i - 1), cases(i).name);
        else
            titleText = sprintf('%s state', states(s));
        end
        styleAxes(ax, titleText, 'time from attack start [s]', ylabels(s), style);
        if s == 1 && i == 1
            legend(ax, 'Location', 'best', 'FontSize', style.fontSize - 1);
        end
    end
end
title(layout, sprintf('State estimates for target V%d', focus), ...
    'FontName', style.fontName, 'FontSize', style.titleSize, 'FontWeight', 'bold');
end

function fig = makePositionErrorFigure(cases, focus, style, timeWindow)
n = numel(cases);
fig = figure('Name', 'Paper Position Error Reference', 'Color', 'w', ...
    'Units', 'centimeters', 'Position', [2 2 20 11]);
layout = tiledlayout(fig, 3, n, 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:n
    tbl = cases(i).table;
    t = cases(i).time;
    refX = col(tbl, sprintf('ref_x_%d', focus));
    refY = col(tbl, sprintf('ref_y_%d', focus));
    postX = col(tbl, sprintf('postpred_x_%d', focus));
    postY = col(tbl, sprintf('postpred_y_%d', focus));
    estX = col(tbl, sprintf('est_x_%d', focus));
    estY = col(tbl, sprintf('est_y_%d', focus));

    ax = nexttile(layout, i);
    hold(ax, 'on');
    yline(ax, 0, ':', 'Color', style.attackColor, 'LineWidth', 0.8, ...
        'HandleVisibility', 'off');
    plotSignedError(ax, t, postX, refX, 'Post-prediction - reference', ...
        style.postColor, '--', style.lineWidth);
    plotSignedError(ax, t, estX, refX, 'Final estimate - reference', ...
        style.estColor, '-', style.lineWidth);
    markAttackAndRollback(ax, cases(i), style);
    applyTimeWindow(ax, timeWindow);
    styleAxes(ax, sprintf('(%c) %s', char('a' + i - 1), cases(i).name), ...
        'time from attack start [s]', 'x error [m]', style);
    if i == 1
        legend(ax, 'Location', 'best', 'FontSize', style.fontSize - 1);
    end

    ax = nexttile(layout, n + i);
    hold(ax, 'on');
    yline(ax, 0, ':', 'Color', style.attackColor, 'LineWidth', 0.8, ...
        'HandleVisibility', 'off');
    plotSignedError(ax, t, postY, refY, 'Post-prediction - reference', ...
        style.postColor, '--', style.lineWidth);
    plotSignedError(ax, t, estY, refY, 'Final estimate - reference', ...
        style.estColor, '-', style.lineWidth);
    markAttackAndRollback(ax, cases(i), style);
    applyTimeWindow(ax, timeWindow);
    styleAxes(ax, 'y-position signed error', ...
        'time from attack start [s]', 'y error [m]', style);

    ax = nexttile(layout, 2 * n + i);
    hold(ax, 'on');
    postPosErr = positionErrorWithReference(tbl, focus, 'postpred');
    estPosErr = positionErrorWithReference(tbl, focus, 'est');
    if any(isfinite(postPosErr))
        plot(ax, t, postPosErr, '--', 'Color', style.postColor, ...
            'LineWidth', style.lineWidth, 'DisplayName', 'Post-prediction error');
    end
    if any(isfinite(estPosErr))
        plot(ax, t, estPosErr, '-', 'Color', style.estColor, ...
            'LineWidth', style.lineWidth, 'DisplayName', 'Final estimate error');
    end
    markAttackAndRollback(ax, cases(i), style);
    applyTimeWindow(ax, timeWindow);
    styleAxes(ax, 'position error magnitude', ...
        'time from attack start [s]', 'position error [m]', style);
end

title(layout, sprintf('Position estimation error relative to reference for target V%d', focus), ...
    'FontName', style.fontName, 'FontSize', style.titleSize, 'FontWeight', 'bold');
end

function plotSignedError(ax, t, signal, reference, labelText, color, lineStyle, lineWidth)
err = signal - reference;
mask = isfinite(t) & isfinite(err);
if any(mask)
    plot(ax, t(mask), err(mask), 'Color', color, 'LineStyle', lineStyle, ...
        'LineWidth', lineWidth, 'DisplayName', labelText);
end
end

function err = positionErrorWithReference(tbl, focus, prefix)
err = col(tbl, sprintf('%s_pos_err_%d', prefix, focus));
if any(isfinite(err))
    return;
end

x = col(tbl, sprintf('%s_x_%d', prefix, focus));
y = col(tbl, sprintf('%s_y_%d', prefix, focus));
refX = col(tbl, sprintf('ref_x_%d', focus));
refY = col(tbl, sprintf('ref_y_%d', focus));
mask = isfinite(x) & isfinite(y) & isfinite(refX) & isfinite(refY);
err = nan(size(x));
err(mask) = hypot(x(mask) - refX(mask), y(mask) - refY(mask));
end

function fig = makeRollbackFigure(cases, focus, style, timeWindow)
n = numel(cases);
fig = figure('Name', 'Paper Rollback Correction', 'Color', 'w', ...
    'Units', 'centimeters', 'Position', [2 2 18 12]);
layout = tiledlayout(fig, n, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:n
    ax = nexttile(layout);
    tbl = cases(i).table;
    t = cases(i).time;
    post = col(tbl, sprintf('postpred_pos_err_%d', focus));
    est = col(tbl, sprintf('est_pos_err_%d', focus));
    hold(ax, 'on');
    if any(isfinite(post))
        plot(ax, t, post, '--', 'Color', style.postColor, 'LineWidth', style.lineWidth, ...
            'DisplayName', 'Before rollback correction');
    end
    if any(isfinite(est))
        plot(ax, t, est, '-', 'Color', style.estColor, 'LineWidth', style.lineWidth, ...
            'DisplayName', 'After rollback correction');
    end
    markAttackAndRollback(ax, cases(i), style);
    annotateCorrection(ax, cases(i), style);
    applyTimeWindow(ax, timeWindow);
    styleAxes(ax, sprintf('(%c) %s', char('a' + i - 1), cases(i).name), ...
        'time from attack start [s]', 'position error [m]', style);
    if i == 1
        legend(ax, 'Location', 'best', 'FontSize', style.fontSize - 1);
    end
end
title(layout, sprintf('Rollback correction effect for target V%d', focus), ...
    'FontName', style.fontName, 'FontSize', style.titleSize, 'FontWeight', 'bold');
end

function annotateCorrection(ax, caseData, style)
event = caseData.event;
metrics = caseData.metrics;
if ~event.rollback_triggered || ~isfinite(event.time_from_attack_s)
    msg = sprintf('No rollback event\nattack RMSE = %.3f m\nattack max = %.3f m', ...
        metrics.est_attack_rmse_m, metrics.est_attack_max_m);
else
    if event.postpred_pos_err_m > 0
        pct = 100 * event.correction_m / event.postpred_pos_err_m;
    else
        pct = NaN;
    end
    msg = sprintf(['t - t_attack = %.3f s\npost = %.3f m, final = %.3f m\n', ...
        'corrected = %.3f m (%.1f%%)\nattack RMSE = %.3f m'], ...
        event.time_from_attack_s, event.postpred_pos_err_m, event.est_pos_err_m, ...
        event.correction_m, pct, metrics.est_attack_rmse_m);
end
text(ax, 0.02, 0.82, msg, 'Units', 'normalized', ...
    'FontName', style.fontName, 'FontSize', style.fontSize, ...
    'Color', style.eventColor, 'BackgroundColor', [1 1 1 0.85], ...
    'Margin', 3, 'Interpreter', 'none');
end

function fig = makeTrustFigure(cases, focus, style, timeWindow)
panels = {
    sprintf('trust_%d', focus), 'direct trust'
    sprintf('gtrust_%d', focus), 'generalized trust'
    sprintf('local_trust_%d', focus), 'local trust'
    sprintf('global_trust_%d', focus), 'global trust'};

n = numel(cases);
fig = figure('Name', 'Paper Trust', 'Color', 'w', ...
    'Units', 'centimeters', 'Position', [2 2 20 14]);
layout = tiledlayout(fig, size(panels, 1), n, 'TileSpacing', 'compact', 'Padding', 'compact');

for p = 1:size(panels, 1)
    for i = 1:numel(cases)
        ax = nexttile(layout, (p - 1) * n + i);
        hold(ax, 'on');
        y = col(cases(i).table, panels{p, 1});
        if any(isfinite(y))
            plot(ax, cases(i).time, y, 'Color', style.estColor, ...
                'LineWidth', style.lineWidth, 'DisplayName', panels{p, 2});
        end
        markAttackAndRollback(ax, cases(i), style);
        applyTimeWindow(ax, timeWindow);
        ylim(ax, paddedLimits(ax, [0 1]));
        if p == 1
            titleText = sprintf('(%c) %s', char('a' + i - 1), cases(i).name);
        else
            titleText = panels{p, 2};
        end
        styleAxes(ax, titleText, 'time from attack start [s]', 'trust [-]', style);
    end
end
title(layout, sprintf('Trust evolution for target V%d', focus), ...
    'FontName', style.fontName, 'FontSize', style.titleSize, 'FontWeight', 'bold');
end

function fig = makeWeightsFigure(cases, focus, style, timeWindow)
panels = {
    sprintf('w0_final_%d', focus), 'base/nominal weight'
    sprintf('w_self_final_%d', focus), 'self weight'
    sprintf('w_neighbor_sum_final_%d', focus), 'neighbor total weight'
    sprintf('w_neighbor_%d', focus), 'target neighbor weight'};

n = numel(cases);
fig = figure('Name', 'Paper Weights', 'Color', 'w', ...
    'Units', 'centimeters', 'Position', [2 2 20 14]);
layout = tiledlayout(fig, size(panels, 1), n, 'TileSpacing', 'compact', 'Padding', 'compact');

for p = 1:size(panels, 1)
    for i = 1:numel(cases)
        ax = nexttile(layout, (p - 1) * n + i);
        hold(ax, 'on');
        y = col(cases(i).table, panels{p, 1});
        if any(isfinite(y))
            plot(ax, cases(i).time, y, 'Color', style.estColor, ...
                'LineWidth', style.lineWidth, 'DisplayName', panels{p, 2});
        end
        markAttackAndRollback(ax, cases(i), style);
        applyTimeWindow(ax, timeWindow);
        ylim(ax, paddedLimits(ax, [0 1]));
        if p == 1
            titleText = sprintf('(%c) %s', char('a' + i - 1), cases(i).name);
        else
            titleText = panels{p, 2};
        end
        styleAxes(ax, titleText, 'time from attack start [s]', 'weight [-]', style);
    end
end
title(layout, sprintf('Weight allocation for target V%d', focus), ...
    'FontName', style.fontName, 'FontSize', style.titleSize, 'FontWeight', 'bold');
end

function markAttackAndRollback(ax, caseData, style)
xline(ax, 0, '-', 'Color', style.attackColor, 'LineWidth', 0.9, ...
    'HandleVisibility', 'off');
attackDuration = caseData.attackEnd_s - caseData.attackStart_s;
if isfinite(attackDuration) && attackDuration > 0
    xline(ax, attackDuration, '-', 'Color', style.attackColor, 'LineWidth', 0.9, ...
        'HandleVisibility', 'off');
end

rb = col(caseData.table, 'rollback_triggered');
t = caseData.time;
idx = find(isfinite(t) & isfinite(rb) & rb > 0);
for k = 1:numel(idx)
    xline(ax, caseData.time(idx(k)), ':', 'Color', style.eventColor, ...
        'LineWidth', 0.9, 'HandleVisibility', 'off');
end
end

function event = correctionEvent(tbl, focus, attackStart_s)
t = col(tbl, 'time');
post = col(tbl, sprintf('postpred_pos_err_%d', focus));
est = col(tbl, sprintf('est_pos_err_%d', focus));
rb = col(tbl, 'rollback_triggered');
enabled = maxFinite(col(tbl, 'rollback_enabled'));
correction = post - est;

mask = isfinite(t) & isfinite(post) & isfinite(est);
rbMask = mask & isfinite(rb) & rb > 0;
if any(rbMask)
    idxs = find(rbMask);
    [~, relIdx] = max(correction(idxs));
    idx = idxs(relIdx);
elseif any(mask)
    idxs = find(mask);
    [~, relIdx] = max(correction(idxs));
    idx = idxs(relIdx);
else
    idx = NaN;
end

event = struct();
event.index = idx;
event.rollback_enabled = isfinite(enabled) && enabled > 0;
event.rollback_triggered = false;
event.rollback_count = nnz(isfinite(rb) & rb > 0);
event.time_s = NaN;
event.time_from_attack_s = NaN;
event.postpred_pos_err_m = NaN;
event.est_pos_err_m = NaN;
event.correction_m = 0;
event.correction_percent = 0;

if isfinite(idx)
    event.rollback_triggered = isfinite(rb(idx)) && rb(idx) > 0;
    event.time_s = t(idx);
    event.time_from_attack_s = t(idx) - attackStart_s;
    event.postpred_pos_err_m = post(idx);
    event.est_pos_err_m = est(idx);
    event.correction_m = correction(idx);
    if post(idx) > 0
        event.correction_percent = 100 * correction(idx) / post(idx);
    end
end
end

function summary = buildSummary(cases, focus)
n = numel(cases);
caseName = strings(n, 1);
file = strings(n, 1);
rollbackEnabled = false(n, 1);
rollbackTriggeredRows = zeros(n, 1);
attackStart_s = nan(n, 1);
attackEnd_s = nan(n, 1);
rollbackEventTime_s = nan(n, 1);
rollbackEventTimeFromAttack_s = nan(n, 1);
postpredPosErr_m = nan(n, 1);
estPosErr_m = nan(n, 1);
correction_m = nan(n, 1);
correction_percent = nan(n, 1);
postAttackRmse_m = nan(n, 1);
finalAttackRmse_m = nan(n, 1);
postAttackMax_m = nan(n, 1);
finalAttackMax_m = nan(n, 1);

for i = 1:n
    event = cases(i).event;
    metrics = cases(i).metrics;
    caseName(i) = cases(i).name;
    file(i) = cases(i).relativeFile;
    rollbackEnabled(i) = event.rollback_enabled;
    rollbackTriggeredRows(i) = event.rollback_count;
    attackStart_s(i) = cases(i).attackStart_s;
    attackEnd_s(i) = cases(i).attackEnd_s;
    if event.rollback_triggered
        rollbackEventTime_s(i) = event.time_s;
        rollbackEventTimeFromAttack_s(i) = event.time_from_attack_s;
        postpredPosErr_m(i) = event.postpred_pos_err_m;
        estPosErr_m(i) = event.est_pos_err_m;
        correction_m(i) = event.correction_m;
        correction_percent(i) = event.correction_percent;
    else
        correction_m(i) = 0;
        correction_percent(i) = 0;
    end
    postAttackRmse_m(i) = metrics.post_attack_rmse_m;
    finalAttackRmse_m(i) = metrics.est_attack_rmse_m;
    postAttackMax_m(i) = metrics.post_attack_max_m;
    finalAttackMax_m(i) = metrics.est_attack_max_m;
end

focusVehicle = repmat(focus, n, 1);
summary = table(caseName, focusVehicle, rollbackEnabled, rollbackTriggeredRows, ...
    attackStart_s, attackEnd_s, rollbackEventTime_s, rollbackEventTimeFromAttack_s, ...
    postpredPosErr_m, estPosErr_m, correction_m, correction_percent, ...
    postAttackRmse_m, finalAttackRmse_m, postAttackMax_m, finalAttackMax_m, file);
end

function printSummary(summary, focus)
fprintf('\nPaper rollback correction summary for target V%d\n', focus);
fprintf('--------------------------------------------------------------------------------\n');
for i = 1:height(summary)
    fprintf('%d) %s\n', i, summary.caseName(i));
    fprintf('   file: %s\n', summary.file(i));
    fprintf('   rollback enabled: %d | rollback rows: %d\n', ...
        summary.rollbackEnabled(i), summary.rollbackTriggeredRows(i));
    fprintf('   attack window: %.3f s to %.3f s | final RMSE %.3f m | final max %.3f m\n', ...
        summary.attackStart_s(i), summary.attackEnd_s(i), ...
        summary.finalAttackRmse_m(i), summary.finalAttackMax_m(i));
    if isfinite(summary.rollbackEventTime_s(i))
        fprintf('   rollback t = %.4f s (t-attack = %.4f s) | post = %.3f m | final = %.3f m | corrected = %.3f m (%.1f%%)\n', ...
            summary.rollbackEventTime_s(i), summary.rollbackEventTimeFromAttack_s(i), ...
            summary.postpredPosErr_m(i), summary.estPosErr_m(i), ...
            summary.correction_m(i), summary.correction_percent(i));
    else
        fprintf('   no rollback correction event in this run\n');
    end
end
fprintf('--------------------------------------------------------------------------------\n\n');
end

function y = col(tbl, name)
name = char(name);
if ismember(name, tbl.Properties.VariableNames)
    raw = tbl.(name);
    if isnumeric(raw)
        y = double(raw);
    elseif islogical(raw)
        y = double(raw);
    elseif iscell(raw)
        y = str2double(string(raw));
    else
        y = str2double(string(raw));
    end
else
    y = nan(height(tbl), 1);
end
y = y(:);
end

function value = maxFinite(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = max(values);
end
end

function value = firstFinite(values)
values = values(:);
idx = find(isfinite(values), 1, 'first');
if isempty(idx)
    value = NaN;
else
    value = values(idx);
end
end

function value = maxOrNan(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = max(values);
end
end

function value = rmse(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = sqrt(mean(values .^ 2));
end
end

function styleAxes(ax, titleText, xlabelText, ylabelText, style)
title(ax, titleText, 'FontSize', style.titleSize, 'FontWeight', 'bold');
xlabel(ax, xlabelText, 'FontSize', style.fontSize);
ylabel(ax, ylabelText, 'FontSize', style.fontSize);
grid(ax, 'on');
ax.GridAlpha = style.gridAlpha;
ax.FontName = style.fontName;
ax.FontSize = style.fontSize;
ax.LineWidth = 0.8;
box(ax, 'on');
end

function lim = paddedLimits(ax, fallback)
children = findobj(ax, 'Type', 'line');
y = [];
for i = 1:numel(children)
    y = [y; children(i).YData(:)]; %#ok<AGROW>
end
y = y(isfinite(y));
if isempty(y)
    lim = fallback;
    return;
end
yMin = min(y);
yMax = max(y);
pad = 0.05 * max(1e-6, yMax - yMin);
lim = [min(fallback(1), yMin - pad), max(fallback(2), yMax + pad)];
end

function applyTimeWindow(ax, timeWindow)
if isempty(timeWindow)
    return;
end
if numel(timeWindow) == 2 && all(isfinite(timeWindow)) && timeWindow(2) > timeWindow(1)
    xlim(ax, timeWindow);
end
end

function mask = timeWindowMask(t, timeWindow)
mask = true(size(t));
if isempty(timeWindow)
    return;
end
if numel(timeWindow) == 2 && all(isfinite(timeWindow)) && timeWindow(2) > timeWindow(1)
    mask = isfinite(t) & t >= timeWindow(1) & t <= timeWindow(2);
end
end

function tf = isTimeInWindow(t, timeWindow)
if isempty(timeWindow)
    tf = true;
    return;
end
if numel(timeWindow) == 2 && all(isfinite(timeWindow)) && timeWindow(2) > timeWindow(1)
    tf = isfinite(t) && t >= timeWindow(1) && t <= timeWindow(2);
else
    tf = true;
end
end

function finishFigure(fig, name, outputDir, formats, shouldSave, dpi)
set(fig, 'Renderer', 'painters');
if ~shouldSave
    return;
end
ensureFolder(outputDir);
for i = 1:numel(formats)
    fmt = lower(strtrim(formats(i)));
    out = fullfile(outputDir, name + "." + fmt);
    switch fmt
        case "pdf"
            exportgraphics(fig, out, 'ContentType', 'vector');
        case "png"
            exportgraphics(fig, out, 'Resolution', dpi);
        otherwise
            warning('Unsupported export format "%s"; skipping.', fmt);
    end
end
end
