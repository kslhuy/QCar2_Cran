function summary = plot_trust_data_matlab(varargin)
%PLOT_TRUST_DATA_MATLAB MATLAB port of plot_trust_data.py for static analysis.
%
% Usage examples:
%   plot_trust_data_matlab
%   plot_trust_data_matlab('File', 'trust_weight_log_V1.csv')
%   plot_trust_data_matlab('File', 'trust_weight_log_V0.csv', 'Focus', 1)
%   plot_trust_data_matlab('File', 'trust_weight_log_V0.csv', 'All', true)
%   plot_trust_data_matlab('Case', 'case1_local', 'Host', 1, 'Focus', 0)
%   plot_trust_data_matlab('Case', 'case1_local', 'Host', 1, 'Latest', true)
%   plot_trust_data_matlab('ResultDate', '02-07-26', 'Host', 1)
%   plot_trust_data_matlab('SelectFile', false)
%   plot_trust_data_matlab('PaperOnly', true, 'SavePaperFigures', true)
%
% This port focuses on the main static diagnostics workflow:
%   1. trust plots
%   2. weight plots
%   3. state-estimation plots
%   4. impact histograms
%   5. attack timeline
%
% It does not attempt to replicate the Python playback dashboard,
% motion-test export, rollback timeline, or relative-UIO plotting.
close all

parser = inputParser;
addParameter(parser, 'File', '', @(x) ischar(x) || isstring(x));
addParameter(parser, 'Case', '', @(x) ischar(x) || isstring(x));
addParameter(parser, 'Host', [], @(x) isempty(x) || (isscalar(x) && isnumeric(x)));
addParameter(parser, 'ResultDate', '', @(x) ischar(x) || isstring(x));
addParameter(parser, 'SelectFile', true, @(x) islogical(x) || isnumeric(x));
addParameter(parser, 'SelectFiles', [], @(x) isempty(x) || islogical(x) || isnumeric(x));
addParameter(parser, 'Latest', false, @(x) islogical(x) || isnumeric(x));
addParameter(parser, 'Focus', [], @(x) isempty(x) || (isscalar(x) && isnumeric(x)));
addParameter(parser, 'All', false, @(x) islogical(x) || isnumeric(x));
addParameter(parser, 'PlotAttackTimeline', true, @(x) islogical(x) || isnumeric(x));
addParameter(parser, 'PlotImpactHistograms', true, @(x) islogical(x) || isnumeric(x));
addParameter(parser, 'PlotPaperFigures', true, @(x) islogical(x) || isnumeric(x));
addParameter(parser, 'PaperOnly', false, @(x) islogical(x) || isnumeric(x));
addParameter(parser, 'SavePaperFigures', false, @(x) islogical(x) || isnumeric(x));
addParameter(parser, 'PaperOutputDir', 'paper_figures', @(x) ischar(x) || isstring(x));
addParameter(parser, 'PaperFormats', ["png", "pdf"], @(x) isstring(x) || iscell(x) || ischar(x));
addParameter(parser, 'Dpi', 600, @(x) isscalar(x) && isnumeric(x));
parse(parser, varargin{:});
args = parser.Results;
if ~isempty(args.SelectFiles)
    args.SelectFile = args.SelectFiles;
end

scriptDir = fileparts(mfilename('fullpath'));
csvFiles = discoverCsvFiles(scriptDir, args);
if isempty(csvFiles)
    error('No trust log files found under %s or %s', scriptDir, fullfile(scriptDir, 'results'));
end

if strlength(string(args.File)) > 0
    fileToPlot = resolveCsvFile(args.File, scriptDir);
elseif logical(args.Latest)
    fileToPlot = fullfile(csvFiles(1).folder, csvFiles(1).name);
elseif logical(args.SelectFile)
    fileToPlot = selectCsvFileWithDialog(csvFiles, scriptDir, args);
else
    fileToPlot = selectCsvFile(csvFiles, scriptDir);
end

tbl = readTrustTable(fileToPlot);
columns = string(tbl.Properties.VariableNames);
if ~any(columns == "time")
    error('Invalid format: missing required column "time".');
end

times = colToArray(tbl, 'time');
vehicleIds = extractVehicleIds(columns);
active = activeVehicles(tbl, columns, vehicleIds);
if isempty(active)
    error('No active vehicle data found in %s', fileToPlot);
end

hostId = parseHostId(fileToPlot);
neighbors = active(active ~= hostId);
if isempty(neighbors)
    focusCandidates = active;
else
    focusCandidates = neighbors;
end

fprintf('Host vehicle: V%d\n', hostId);
fprintf('Active vehicles: %s\n', mat2str(active));
fprintf('Candidate focus vehicles: %s\n', mat2str(focusCandidates));
fprintf('Total samples: %d\n\n', height(tbl));

printStaticMetrics(tbl, active);

if logical(args.All)
    focuses = focusCandidates;
elseif ~isempty(args.Focus)
    focus = double(args.Focus);
    if any(focusCandidates == focus) || any(active == focus)
        focuses = focus;
    else
        warning('Requested focus V%d not found. Using V%d instead.', focus, focusCandidates(1));
        focuses = focusCandidates(1);
    end
else
    focuses = focusCandidates(1);
end

paperOutputDir = resolvePaperOutputDir(args.PaperOutputDir, scriptDir);
paperFormats = normalizeStringArray(args.PaperFormats);
plotPaperFigures = logical(args.PlotPaperFigures) || logical(args.PaperOnly);
for focus = focuses
    if plotPaperFigures
        figTrustPaper = makePaperTrustComponentsFigure(tbl, times, focus, hostId);
        figWeightPaper = makePaperWeightsFigure(tbl, times, active, focus, hostId);
        finishPaperFigure(figTrustPaper, sprintf('paper_trust_components_hostV%d_focusV%d', hostId, focus), ...
            paperOutputDir, paperFormats, logical(args.SavePaperFigures), args.Dpi);
        finishPaperFigure(figWeightPaper, sprintf('paper_weights_hostV%d_focusV%d', hostId, focus), ...
            paperOutputDir, paperFormats, logical(args.SavePaperFigures), args.Dpi);
    end

    if ~logical(args.PaperOnly)
        makeTrustFigure(tbl, times, active, focus, hostId);
        makeWeightsFigure(tbl, times, active, focus, hostId);
        makeEstimationFigure(tbl, times, active, focus, hostId);
        if logical(args.PlotImpactHistograms)
            makeImpactHistogramFigure(tbl, focus, hostId);
        end
    end
end

if logical(args.PlotAttackTimeline) && ~logical(args.PaperOnly)
    makeAttackTimelineFigure(tbl, times, columns, active, hostId);
end
if logical(args.SavePaperFigures) && plotPaperFigures
    fprintf('Saved paper figures to: %s\n', paperOutputDir);
end

summary = struct( ...
    'file', fileToPlot, ...
    'host_id', hostId, ...
    'active', active, ...
    'focus_candidates', focusCandidates, ...
    'focuses', focuses);
end

function csvFiles = discoverCsvFiles(scriptDir, args)
csvFiles = [
    dir(fullfile(scriptDir, 'trust_weight_log_V*.csv'));
    dir(fullfile(scriptDir, 'results', '**', 'trust_weight_log_V*.csv'))];

csvFiles = csvFiles(~[csvFiles.isdir]);
csvFiles = filterCsvFiles(csvFiles, scriptDir, args);
csvFiles = sortCsvFilesNewestFirst(csvFiles);
end

function csvFiles = filterCsvFiles(csvFiles, scriptDir, args)
dateFilter = lower(strtrim(string(args.ResultDate)));
if strlength(dateFilter) > 0
    keep = false(size(csvFiles));
    for i = 1:numel(csvFiles)
        filepath = fullfile(csvFiles(i).folder, csvFiles(i).name);
        relpath = lower(string(erase(filepath, [scriptDir filesep])));
        keep(i) = contains(relpath, dateFilter);
    end
    csvFiles = csvFiles(keep);
end

caseFilter = lower(strtrim(string(args.Case)));
if strlength(caseFilter) > 0
    keep = false(size(csvFiles));
    for i = 1:numel(csvFiles)
        filepath = fullfile(csvFiles(i).folder, csvFiles(i).name);
        relpath = lower(string(erase(filepath, [scriptDir filesep])));
        keep(i) = contains(relpath, caseFilter);
    end
    csvFiles = csvFiles(keep);
end

if ~isempty(args.Host)
    host = double(args.Host);
    keep = false(size(csvFiles));
    for i = 1:numel(csvFiles)
        keep(i) = parseHostId(fullfile(csvFiles(i).folder, csvFiles(i).name)) == host;
    end
    csvFiles = csvFiles(keep);
end
end

function csvFiles = sortCsvFilesNewestFirst(csvFiles)
if isempty(csvFiles)
    return;
end
[~, idx] = sort([csvFiles.datenum], 'descend');
csvFiles = csvFiles(idx);
end

function fileToPlot = resolveCsvFile(fileArg, scriptDir)
fileToPlot = char(fileArg);
if isfile(fileToPlot)
    return;
end

candidate = fullfile(scriptDir, fileToPlot);
if isfile(candidate)
    fileToPlot = candidate;
    return;
end

candidate = fullfile(scriptDir, 'results', fileToPlot);
if isfile(candidate)
    fileToPlot = candidate;
    return;
end

error('Specified file does not exist: %s', fileToPlot);
end

function fileToPlot = selectCsvFileWithDialog(csvFiles, scriptDir, args)
try
    startDir = selectDialogStartDir(csvFiles, scriptDir, args);
    previousDir = pwd;
    cleanupObj = onCleanup(@() cd(previousDir));
    cd(startDir);
    filterSpec = {
        trustLogPattern(args), 'Trust log CSV files'
        '*.csv', 'CSV files (*.csv)'
        '*.*', 'All files (*.*)'};
    [selected, selectedPath] = uigetfile(filterSpec, 'Select trust CSV to plot');
catch dialogError
    warning('plot_trust_data_matlab:FileDialogUnavailable', ...
        'File dialog unavailable (%s). Falling back to command-window selection.', dialogError.message);
    fileToPlot = selectCsvFile(csvFiles, scriptDir);
    return;
end

if isequal(selected, 0)
    error('File selection cancelled.');
end

fileToPlot = fullfile(selectedPath, selected);
fprintf('\nSelected trust plot file:\n  %s\n\n', char(erase(string(fileToPlot), string(scriptDir) + filesep)));
end

function startDir = selectDialogStartDir(csvFiles, scriptDir, args)
rootDir = fullfile(scriptDir, 'results');
resultDate = strtrim(string(args.ResultDate));
if strlength(resultDate) > 0
    datedDir = fullfile(rootDir, char(resultDate));
    if isfolder(datedDir)
        startDir = datedDir;
    else
        startDir = rootDir;
    end
elseif isfolder(rootDir)
    startDir = rootDir;
else
    startDir = scriptDir;
end

if isempty(dir(fullfile(startDir, trustLogPattern(args)))) && ~isempty(csvFiles)
    startDir = csvFiles(1).folder;
end
end

function pattern = trustLogPattern(args)
if ~isempty(args.Host)
    pattern = sprintf('trust_weight_log_V%d*.csv', double(args.Host));
else
    pattern = 'trust_weight_log_V*.csv';
end
end

function fileToPlot = selectCsvFile(csvFiles, scriptDir)
if numel(csvFiles) == 1
    fileToPlot = fullfile(csvFiles(1).folder, csvFiles(1).name);
    return;
end

fprintf('Found files (newest first):\n');
for i = 1:numel(csvFiles)
    filepath = fullfile(csvFiles(i).folder, csvFiles(i).name);
    fprintf('  [%d] %s\n', i - 1, char(erase(filepath, [scriptDir filesep])));
end

choice = input(sprintf('Select file to plot [0-%d] (default 0): ', numel(csvFiles) - 1), 's');
if isempty(strtrim(choice))
    idx = 1;
else
    idx = str2double(choice) + 1;
end
if ~isfinite(idx) || idx < 1 || idx > numel(csvFiles)
    error('Invalid file selection.');
end
fileToPlot = fullfile(csvFiles(idx).folder, csvFiles(idx).name);
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

function hostId = parseHostId(filepath)
[~, name, ext] = fileparts(filepath);
token = regexp([name ext], 'V(\d+)(_[^.]*)?\.csv$', 'tokens', 'once');
if isempty(token)
    hostId = -1;
else
    hostId = str2double(token{1});
end
end

function vehicleIds = extractVehicleIds(columns)
prefixes = string({ ...
    'vehicle_present', 'trust', 'gtrust', 'w_neighbor', ...
    'w0_final', 'w_self_final', 'w_neighbor_sum_final', ...
    'est_conf', 'pred_mode', 'est_x', 'est_y', 'est_v', 'est_a', 'est_theta', ...
    'v_score', 'd_score', 'a_score', 'h_score', ...
    'b_score', 'q_factor', 'local_trust', 'global_trust', ...
    'gamma_host', 'gamma_local_peer', 'gamma_self', ...
    'd_host_mean', 'd_local_mean', 'd_self', ...
    'mi_dist', 'mi_veh_id', 'mi_elem_idx', 'mi_elem_val'});
ids = [];
for i = 1:numel(columns)
    token = regexp(columns(i), '^(.*)_(\d+)$', 'tokens', 'once');
    if isempty(token)
        continue;
    end
    base = string(token{1});
    vid = str2double(token{2});
    if any(prefixes == base)
        ids(end + 1) = vid; %#ok<AGROW>
    end
end
vehicleIds = unique(ids);
end

function active = activeVehicles(tbl, columns, vehicleIds)
active = [];
probePrefixes = {'vehicle_present', 'trust', 'gtrust', ...
    'w_neighbor', 'w0_final', 'w_self_final', ...
    'w_neighbor_sum_final', 'est_conf', 'est_x'};

for vid = vehicleIds
    for i = 1:numel(probePrefixes)
        col = sprintf('%s_%d', probePrefixes{i}, vid);
        if ~isColumnPresent(columns, col)
            continue;
        end
        arr = colToArray(tbl, col);
        if strcmp(probePrefixes{i}, 'vehicle_present')
            if any(arr > 0 & isfinite(arr))
                active(end + 1) = vid; %#ok<AGROW>
                break;
            end
        elseif any(isfinite(arr))
            active(end + 1) = vid; %#ok<AGROW>
            break;
        end
    end
end
active = unique(active);
end

function tf = isColumnPresent(columns, col)
tf = any(columns == string(col));
end

function arr = colToArray(tbl, col)
n = height(tbl);
if ~ismember(col, tbl.Properties.VariableNames)
    arr = nan(n, 1);
    return;
end

raw = tbl.(col);
if isnumeric(raw)
    arr = double(raw);
elseif islogical(raw)
    arr = double(raw);
elseif isstring(raw)
    arr = str2double(raw);
elseif iscell(raw)
    arr = nan(n, 1);
    for i = 1:n
        value = raw{i};
        if isnumeric(value) && isscalar(value)
            arr(i) = double(value);
        elseif islogical(value) && isscalar(value)
            arr(i) = double(value);
        else
            arr(i) = str2double(string(value));
        end
    end
elseif ischar(raw)
    arr = str2double(cellstr(raw));
elseif iscategorical(raw)
    arr = str2double(string(raw));
else
    try
        arr = str2double(string(raw));
    catch
        arr = nan(n, 1);
    end
end
arr = arr(:);
end

function txt = colToText(tbl, col)
n = height(tbl);
txt = strings(n, 1);
if ~ismember(col, tbl.Properties.VariableNames)
    return;
end
raw = tbl.(col);
if isstring(raw)
    txt = raw(:);
elseif iscell(raw)
    for i = 1:n
        txt(i) = string(raw{i});
    end
elseif isnumeric(raw) || islogical(raw)
    txt = string(raw(:));
else
    txt = string(raw);
    txt = txt(:);
end
txt = strtrim(txt);
end

function style = ieeeStyle()
style.fontName = 'Times New Roman';
style.fontSize = 7.5;
style.titleSize = 8.2;
style.layoutTitleSize = 9.0;
style.legendSize = 6.5;
style.gridAlpha = 0.22;
style.minorGridAlpha = 0.12;
style.axesLineWidth = 0.65;
end

function fig = paperFigure(name, sizeCm)
style = ieeeStyle();
fig = figure('Name', name, 'Color', 'w', ...
    'Units', 'centimeters', 'Position', [2 2 sizeCm(1) sizeCm(2)], ...
    'PaperUnits', 'centimeters', 'PaperPosition', [0 0 sizeCm(1) sizeCm(2)], ...
    'PaperSize', sizeCm, 'PaperPositionMode', 'manual', ...
    'Renderer', 'painters', 'InvertHardcopy', 'off', ...
    'DefaultAxesFontName', style.fontName, ...
    'DefaultTextFontName', style.fontName, ...
    'DefaultAxesFontSize', style.fontSize, ...
    'DefaultTextFontSize', style.fontSize);
end

function layoutTitle(layout, titleText)
style = ieeeStyle();
title(layout, titleText, 'FontName', style.fontName, ...
    'FontSize', style.layoutTitleSize, 'FontWeight', 'bold');
end

function applyPaperAxes(ax)
style = ieeeStyle();
ax.FontName = style.fontName;
ax.FontSize = style.fontSize;
ax.LineWidth = style.axesLineWidth;
ax.GridAlpha = style.gridAlpha;
ax.MinorGridAlpha = style.minorGridAlpha;
box(ax, 'on');
end

function plotted = plotSeries(ax, times, tbl, prefix, vids, labelFmt)
if nargin < 6
    labelFmt = 'Vehicle %d';
end
plotted = 0;
for vid = vids
    col = sprintf('%s_%d', prefix, vid);
    arr = colToArray(tbl, col);
    if any(isfinite(arr))
        plot(ax, times, arr, 'DisplayName', sprintf(labelFmt, vid), 'LineWidth', 1.2);
        plotted = plotted + 1;
        hold(ax, 'on');
    end
end
end

function noData(ax, titleText)
style = ieeeStyle();
title(ax, titleText, 'FontName', style.fontName, ...
    'FontSize', style.titleSize, 'FontWeight', 'bold');
text(ax, 0.5, 0.5, 'No data', 'Units', 'normalized', ...
    'HorizontalAlignment', 'center', 'Color', [0.5 0.5 0.5], ...
    'FontName', style.fontName, 'FontSize', style.fontSize);
applyPaperAxes(ax);
end

function styleAxes(ax, titleText, ylabelText, xlabelText, showLegend)
style = ieeeStyle();
if nargin < 5
    showLegend = true;
end
if ~isempty(titleText)
    title(ax, titleText, 'FontName', style.fontName, ...
        'FontSize', style.titleSize, 'FontWeight', 'bold');
end
if ~isempty(ylabelText)
    ylabel(ax, ylabelText, 'FontName', style.fontName, 'FontSize', style.fontSize);
end
if ~isempty(xlabelText)
    xlabel(ax, xlabelText, 'FontName', style.fontName, 'FontSize', style.fontSize);
end
grid(ax, 'on');
applyPaperAxes(ax);
if showLegend
    handles = findobj(ax, '-property', 'DisplayName');
    labels = string(get(handles, 'DisplayName'));
    labels = labels(labels ~= "");
    if ~isempty(labels)
        legend(ax, 'show', 'Location', 'best', 'FontName', style.fontName, ...
            'FontSize', style.legendSize, 'Box', 'off');
    end
end
end

function addTurnSections(ax, times, tbl)
isTurning = colToArray(tbl, 'is_turning');
mask = isfinite(isTurning) & isTurning >= 1;
spans = maskToTimeSpans(times, mask);
if isempty(spans)
    return;
end
yl = ylim(ax);
for i = 1:size(spans, 1)
    x0 = spans(i, 1);
    x1 = spans(i, 2);
    patchSpan(ax, x0, x1, yl(1), yl(2), [1.0 0.84 0.0], 0.12);
end
uistack(findobj(ax, 'Type', 'line'), 'top');
end

function fig = makePaperTrustComponentsFigure(tbl, times, focus, hostId)
fig = paperFigure(sprintf('Paper Trust Components Host V%d Focus V%d', hostId, focus), [18.0 12.2]);
t = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
layoutTitle(t, sprintf('Trust, opinion, and component scores for target V%d', focus));

ax = nexttile(t, 1);
hold(ax, 'on');
trustCols = { ...
    sprintf('trust_%d', focus), 'final trust', [0.0000 0.4470 0.7410], '-'; ...
    sprintf('gtrust_%d', focus), 'opinion score O_i(j)', [0.8500 0.3250 0.0980], '-'; ...
    sprintf('local_trust_%d', focus), 'local trust', [0.4660 0.6740 0.1880], '--'; ...
    sprintf('global_trust_%d', focus), 'global trust', [0.4940 0.1840 0.5560], '--'};
count = plotPaperColumns(ax, times, tbl, trustCols, 1.35);
if count == 0
    noData(ax, sprintf('(a) Trust signals for V%d', focus));
else
    yline(ax, 0.5, ':', 'Color', [0.55 0.55 0.55], ...
        'LineWidth', 0.85, 'DisplayName', 'threshold');
    ylim(ax, [0 1.05]);
    addAttackSpans(ax, times, tbl, focus);
end
styleAxes(ax, '(a) Final trust and opinion signals', 'trust [-]', '', true);

ax = nexttile(t, 2);
hold(ax, 'on');
componentCols = { ...
    sprintf('v_score_%d', focus), 'velocity', [0.0000 0.4470 0.7410], '-'; ...
    sprintf('d_score_%d', focus), 'distance', [0.8500 0.3250 0.0980], '-'; ...
    sprintf('a_score_%d', focus), 'acceleration', [0.9290 0.6940 0.1250], '-'; ...
    sprintf('h_score_%d', focus), 'heading', [0.4940 0.1840 0.5560], '-'; ...
    sprintf('b_score_%d', focus), 'beacon', [0.4660 0.6740 0.1880], '--'; ...
    sprintf('q_factor_%d', focus), 'quality factor', [0.3010 0.7450 0.9330], '--'};
count = plotPaperColumns(ax, times, tbl, componentCols, 1.15);
if count == 0
    noData(ax, sprintf('(b) Component scores for V%d', focus));
else
    ylim(ax, [0 1.05]);
    addAttackSpans(ax, times, tbl, focus);
end
styleAxes(ax, '(b) Local/direct component scores', 'score [-]', '', true);

ax = nexttile(t, 3);
hold(ax, 'on');
globalFactorCols = { ...
    sprintf('gamma_host_%d', focus), '\gamma_{host}', [0.0000 0.4470 0.7410], '-'; ...
    sprintf('gamma_local_peer_%d', focus), '\gamma_{local peer}', [0.8500 0.3250 0.0980], '-'; ...
    sprintf('gamma_self_%d', focus), '\gamma_{self}', [0.4660 0.6740 0.1880], '-'; ...
    sprintf('global_trust_%d', focus), 'global trust', [0.4940 0.1840 0.5560], '--'};
count = plotPaperColumns(ax, times, tbl, globalFactorCols, 1.25);
if count == 0
    noData(ax, sprintf('(c) Global opinion factors for V%d', focus));
else
    ylim(ax, [0 1.05]);
    addAttackSpans(ax, times, tbl, focus);
end
styleAxes(ax, '(c) Global/opinion fusion factors', 'score [-]', 'time [s]', true);

set(findall(fig, '-property', 'Interpreter'), 'Interpreter', 'tex');
set(findall(fig, '-property', 'TickLabelInterpreter'), 'TickLabelInterpreter', 'tex');
end

function fig = makePaperWeightsFigure(tbl, times, active, focus, hostId)
fig = paperFigure(sprintf('Paper Weights Host V%d Focus V%d', hostId, focus), [18.0 6.6]);
t = tiledlayout(fig, 1, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
layoutTitle(t, sprintf('Fusion weights for target V%d', focus));

ax = nexttile(t, 1);
hold(ax, 'on');
weightCols = paperWeightColumns(tbl, active, focus);
count = plotPaperColumns(ax, times, tbl, weightCols, 1.35);
if count == 0
    noData(ax, sprintf('Fusion weights for V%d', focus));
else
    ylim(ax, [0 1.05]);
    addAttackSpans(ax, times, tbl, focus);
end
styleAxes(ax, 'Direct, neighbor, and self weights', 'weight [-]', 'time [s]', true);

set(findall(fig, '-property', 'Interpreter'), 'Interpreter', 'tex');
set(findall(fig, '-property', 'TickLabelInterpreter'), 'TickLabelInterpreter', 'tex');
end

function cols = paperWeightColumns(tbl, active, focus)
if hasFiniteColumn(tbl, sprintf('w0_final_%d', focus)) || ...
        hasFiniteColumn(tbl, sprintf('w_self_final_%d', focus)) || ...
        hasFiniteColumn(tbl, sprintf('w_neighbor_sum_final_%d', focus))
    cols = { ...
        sprintf('w0_final_%d', focus), 'w_0', [0.0000 0.4470 0.7410], '-'; ...
        sprintf('w_neighbor_sum_final_%d', focus), 'w_{neighbor}', [0.8500 0.3250 0.0980], '-'; ...
        sprintf('w_self_final_%d', focus), 'w_{self}', [0.4660 0.6740 0.1880], '-'};
    return;
end

neighborCol = 'total_neighbor_weight';
if ~hasFiniteColumn(tbl, neighborCol)
    for src = active
        candidate = sprintf('w_neighbor_from_v%d_to_%d', src, focus);
        if hasFiniteColumn(tbl, candidate)
            neighborCol = candidate;
            break;
        end
    end
end

cols = { ...
    'w0', 'w_0', [0.0000 0.4470 0.7410], '-'; ...
    neighborCol, 'w_{neighbor}', [0.8500 0.3250 0.0980], '-'; ...
    'w_self', 'w_{self}', [0.4660 0.6740 0.1880], '-'};
end

function count = plotPaperColumns(ax, times, tbl, colRows, lineWidth)
count = 0;
for i = 1:size(colRows, 1)
    arr = colToArray(tbl, colRows{i, 1});
    if any(isfinite(arr))
        plot(ax, times, arr, ...
            'Color', colRows{i, 3}, ...
            'LineStyle', colRows{i, 4}, ...
            'LineWidth', lineWidth, ...
            'DisplayName', colRows{i, 2});
        count = count + 1;
    end
end
end

function addAttackSpans(ax, times, tbl, focus)
attackFlag = colToArray(tbl, sprintf('flag_attack_%d', focus));
mask = isfinite(attackFlag) & attackFlag >= 0.5;
spans = maskToTimeSpans(times, mask);
if isempty(spans)
    return;
end
yl = ylim(ax);
for i = 1:size(spans, 1)
    patchSpan(ax, spans(i, 1), spans(i, 2), yl(1), yl(2), [0.78 0.78 0.78], 0.18);
end
uistack(findobj(ax, 'Type', 'patch'), 'bottom');
uistack(findobj(ax, 'Type', 'line'), 'top');
end

function outputDir = resolvePaperOutputDir(outputDirArg, scriptDir)
outputDir = char(outputDirArg);
if isempty(outputDir)
    outputDir = fullfile(scriptDir, 'paper_figures');
elseif ~isfolder(outputDir) && ~contains(outputDir, filesep) && ~contains(outputDir, '/')
    outputDir = fullfile(scriptDir, outputDir);
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

function finishPaperFigure(fig, name, outputDir, formats, shouldSave, dpi)
if ~shouldSave
    return;
end
if ~isfolder(outputDir)
    mkdir(outputDir);
end
for i = 1:numel(formats)
    fmt = lower(strtrim(formats(i)));
    out = fullfile(outputDir, string(name) + "." + fmt);
    switch fmt
        case "pdf"
            exportgraphics(fig, out, 'ContentType', 'vector');
        case "png"
            exportgraphics(fig, out, 'Resolution', dpi);
        otherwise
            warning('Unsupported paper figure format "%s"; skipping.', fmt);
    end
end
end

function makeTrustFigure(tbl, times, active, focus, hostId)
fig = paperFigure(sprintf('Trust Calculation Host V%d Focus V%d', hostId, focus), [18.0 13.8]);
t = tiledlayout(fig, 3, 4, 'TileSpacing', 'compact', 'Padding', 'compact');
layoutTitle(t, sprintf('Trust Calculation (Host V%d, Focus V%d)', hostId, focus));

ax = nexttile(t, 1);
if plotSeries(ax, times, tbl, 'trust', active) == 0
    noData(ax, 'Direct Trust Score');
else
    yline(ax, 0.5, 'r:', 'threshold', 'Alpha', 0.6);
    addTurnSections(ax, times, tbl);
end
styleAxes(ax, 'Direct Trust Score', 'Trust [0,1]', '', true);

ax = nexttile(t, 2);
if plotSeries(ax, times, tbl, 'gtrust', active) == 0
    noData(ax, 'Generalized Trust O_i(j)');
else
    yline(ax, 0.5, 'r:', 'Alpha', 0.6, 'HandleVisibility', 'off');
    addTurnSections(ax, times, tbl);
end
styleAxes(ax, 'Generalized Trust O_i(j)', 'Trust [0,1]', '', true);

ax = nexttile(t, 3);
if plotSeries(ax, times, tbl, 'local_trust', active, 'LT V%d') == 0
    noData(ax, sprintf('Local Trust (V%d)', focus));
else
    addTurnSections(ax, times, tbl);
end
styleAxes(ax, 'Local Trust (gamma_local)', 'Trust [0,1]', '', true);

ax = nexttile(t, 4);
if plotSeries(ax, times, tbl, 'global_trust', active, 'GT V%d') == 0
    noData(ax, sprintf('Global Trust (V%d)', focus));
else
    addTurnSections(ax, times, tbl);
end
styleAxes(ax, 'Global Trust (gamma_cross)', 'Trust [0,1]', '', true);

ax = nexttile(t, 5);
componentCols = { ...
    sprintf('v_score_%d', focus), 'Velocity'; ...
    sprintf('d_score_%d', focus), 'Distance'; ...
    sprintf('a_score_%d', focus), 'Acceleration'; ...
    sprintf('h_score_%d', focus), 'Heading'; ...
    sprintf('b_score_%d', focus), 'Beacon'; ...
    sprintf('q_factor_%d', focus), 'Quality'};
plotNamedColumns(ax, times, tbl, componentCols);
styleAxes(ax, sprintf('Component Scores (V%d)', focus), 'Score [0,1]', '', true);

ax = nexttile(t, 6);
gammaCols = { ...
    sprintf('gamma_host_%d', focus), 'gamma_host'; ...
    sprintf('gamma_local_peer_%d', focus), 'gamma_local_peer'; ...
    sprintf('gamma_self_%d', focus), 'gamma_self'; ...
    sprintf('global_trust_%d', focus), 'global_trust'};
plotNamedColumns(ax, times, tbl, gammaCols);
styleAxes(ax, sprintf('Global Trust Factors (V%d)', focus), 'Value [0,1]', '', true);

ax = nexttile(t, 7);
distCols = { ...
    sprintf('d_host_mean_%d', focus), 'd_host_mean'; ...
    sprintf('d_local_mean_%d', focus), 'd_local_mean'; ...
    sprintf('d_self_%d', focus), 'd_self'};
plotNamedColumns(ax, times, tbl, distCols);
styleAxes(ax, sprintf('Mahalanobis Distances (V%d)', focus), 'Distance', 'Time [s]', true);

ax = nexttile(t, 8);
plotOffsetFlags(ax, times, tbl, { ...
    sprintf('flag_attack_%d', focus), 'Target Attack', 0.0; ...
    sprintf('flag_local_%d', focus), 'Local Est Check', 0.10; ...
    sprintf('flag_global_%d', focus), 'Global Est Check', 0.20});
styleAxes(ax, sprintf('Attack Flags (V%d)', focus), 'Flag', 'Time [s]', true);

ax = nexttile(t, 9);
relCols = { ...
    sprintf('y_local_distance_%d', focus), 'Local rel dist'; ...
    sprintf('y_true_distance_%d', focus), 'True rel dist'; ...
    sprintf('yolo_rel_distance_%d', focus), 'YOLO rel dist'};
plotNamedColumns(ax, times, tbl, relCols);
styleAxes(ax, sprintf('Relative Distance (V%d)', focus), 'Distance [m]', 'Time [s]', true);

ax = nexttile(t, 10);
arrIdx = colToArray(tbl, sprintf('mi_elem_idx_%d', focus));
arrVal = colToArray(tbl, sprintf('mi_elem_val_%d', focus));
if any(isfinite(arrIdx)) && any(isfinite(arrVal))
    style = ieeeStyle();
    yyaxis(ax, 'left');
    plot(ax, times, arrVal, 'LineWidth', 1.4, 'Color', [0.85 0.45 0.10], ...
        'DisplayName', 'Max Element Value');
    ylabel(ax, 'Contribution / Score', 'FontName', style.fontName, 'FontSize', style.fontSize);
    yyaxis(ax, 'right');
    valid = isfinite(arrIdx);
    scatter(ax, times(valid), arrIdx(valid), 14, [0.10 0.55 0.20], 'filled', ...
        'DisplayName', 'Element Index');
    ylabel(ax, 'Impact Element', 'FontName', style.fontName, 'FontSize', style.fontSize);
    yticks(ax, 0:4);
    yticklabels(ax, {'x', 'y', 'theta', 'v', 'a'});
    title(ax, sprintf('Max Impact Element Index and Score (V%d)', focus), ...
        'FontName', style.fontName, 'FontSize', style.titleSize, 'FontWeight', 'bold');
    xlabel(ax, 'Time [s]', 'FontName', style.fontName, 'FontSize', style.fontSize);
    grid(ax, 'on');
    applyPaperAxes(ax);
    legend(ax, 'show', 'Location', 'best', 'FontName', style.fontName, ...
        'FontSize', style.legendSize, 'Box', 'off');
else
    noData(ax, sprintf('Max Impact Element (V%d)', focus));
    styleAxes(ax, sprintf('Max Impact Element Index and Score (V%d)', focus), '', 'Time [s]', false);
end

ax = nexttile(t, 11);
errCols = { ...
    sprintf('yolo_true_rel_dist_error_%d', focus), 'YOLO-True dist error'; ...
    sprintf('yolo_true_rel_vel_error_%d', focus), 'YOLO-True rel vel error'};
count = plotNamedColumns(ax, times, tbl, errCols);
if count > 0
    yline(ax, 0.0, 'k:', 'Alpha', 0.6, 'HandleVisibility', 'off');
end
styleAxes(ax, sprintf('YOLO vs True Error (V%d)', focus), 'Error', 'Time [s]', true);

ax = nexttile(t, 12);
plotOffsetFlags(ax, times, tbl, { ...
    sprintf('rel_meas_used_global_%d', focus), 'Any external rel used', 0.00; ...
    sprintf('yolo_rel_meas_used_global_%d', focus), 'YOLO rel used', 0.12; ...
    sprintf('rel_dist_meas_used_%d', focus), 'External dist used', 0.24; ...
    sprintf('rel_vel_meas_used_%d', focus), 'External rel vel used', 0.36});
styleAxes(ax, sprintf('Relative Measurement Usage (V%d)', focus), 'Flag', 'Time [s]', true);
end

function makeWeightsFigure(tbl, times, active, focus, hostId)
hasFinal = hasFiniteColumn(tbl, sprintf('w0_final_%d', focus)) || ...
    hasFiniteColumn(tbl, sprintf('w_self_final_%d', focus)) || ...
    hasFiniteColumn(tbl, sprintf('w_neighbor_sum_final_%d', focus));
if ~hasFinal
    for src = active
        if hasFiniteColumn(tbl, sprintf('w_neighbor_from_v%d_to_%d', src, focus))
            hasFinal = true;
            break;
        end
    end
end

fig = paperFigure(sprintf('Weight Calculation Host V%d Focus V%d', hostId, focus), [18.0 10.5]);
t = tiledlayout(fig, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
note = 'legacy summary';
if hasFinal
    note = 'final per-target';
end
layoutTitle(t, sprintf('Weight Calculation (%s) (Host V%d, Focus V%d)', note, hostId, focus));

ax = nexttile(t, 1);
if hasFinal
    cols = { ...
        sprintf('w0_final_%d', focus), sprintf('w0 final -> V%d', focus); ...
        sprintf('w_self_final_%d', focus), sprintf('w_self final -> V%d', focus); ...
        sprintf('w_neighbor_sum_final_%d', focus), sprintf('neighbor sum final -> V%d', focus)};
else
    cols = { ...
        'w0', 'w0 summary'; ...
        'w_self', 'w_self summary'; ...
        'total_neighbor_weight', 'neighbor total summary'};
end
plotNamedColumns(ax, times, tbl, cols);
styleAxes(ax, sprintf('Final Target Weights V%d', focus), 'Weight', '', true);

ax = nexttile(t, 2);
count = 0;
if hasFinal
    for src = active
        col = sprintf('w_neighbor_from_v%d_to_%d', src, focus);
        arr = colToArray(tbl, col);
        if any(isfinite(arr))
            hold(ax, 'on');
            plot(ax, times, arr, 'LineWidth', 1.2, 'DisplayName', sprintf('V%d -> V%d', src, focus));
            count = count + 1;
        end
    end
else
    count = plotSeries(ax, times, tbl, 'w_neighbor', active, 'summary w\\_neighbor V%d');
end
if count == 0
    noData(ax, sprintf('Final Neighbor Source Weights to V%d', focus));
end
styleAxes(ax, sprintf('Final Neighbor Source Weights to V%d', focus), 'Weight', '', true);

ax = nexttile(t, 3);
plotNamedColumns(ax, times, tbl, { ...
    'w0', 'w0 summary'; ...
    'w_self', 'w_self summary'; ...
    'total_neighbor_weight', 'neighbor total summary'});
styleAxes(ax, 'Legacy Summary Weights', 'Weight', '', true);

ax = nexttile(t, 4);
if plotSeries(ax, times, tbl, 'w_neighbor', active, 'summary w\\_neighbor V%d') == 0
    noData(ax, 'Legacy Per-Neighbor Weights');
end
styleAxes(ax, 'Legacy Per-Neighbor Weights', 'Weight', 'Time [s]', true);

ax = nexttile(t, 5);
plotNamedColumns(ax, times, tbl, { ...
    'mean_direct_trust', 'Mean Direct Trust'; ...
    'mean_generalized_trust', 'Mean Generalized Trust'; ...
    'mean_gamma_host', 'Mean gamma\_host'; ...
    'mean_gamma_local_peer', 'Mean gamma\_local\_peer'; ...
    'mean_gamma_self', 'Mean gamma\_self'; ...
    'weighted_neighbor_trust', 'Weighted Neighbor Trust'});
styleAxes(ax, 'Trust Summary Metrics', 'Value', 'Time [s]', true);

ax = nexttile(t, 6);
count = 0;
for pair = { ...
        {'trusted_neighbor_count', 'Trusted Neighbors', true}, ...
        {'active_vehicle_count', 'Active Vehicles', true}, ...
        {'total_neighbor_weight', 'Legacy Neighbor Total', false}}
    item = pair{1};
    arr = colToArray(tbl, item{1});
    if any(isfinite(arr))
        hold(ax, 'on');
        if item{3}
            stairs(ax, times, arr, 'LineWidth', 1.2, 'DisplayName', item{2});
        else
            plot(ax, times, arr, 'LineWidth', 1.1, 'DisplayName', item{2});
        end
        count = count + 1;
    end
end
if count == 0
    noData(ax, 'Counts / Legacy Summary');
end
styleAxes(ax, 'Counts / Legacy Summary', 'Count or Weight', 'Time [s]', true);
end

function makeEstimationFigure(tbl, times, active, focus, hostId)
fig = paperFigure(sprintf('State Estimation Host V%d Focus V%d', hostId, focus), [18.0 18.5]);
t = tiledlayout(fig, 5, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
layoutTitle(t, sprintf('State Estimation (Host V%d, Focus V%d)', hostId, focus));

ax = nexttile(t, 1);
if plotSeries(ax, times, tbl, 'est_x', active, 'V%d') == 0
    noData(ax, 'Estimated X');
end
styleAxes(ax, 'Estimated X', 'x [m]', '', true);

ax = nexttile(t, 2);
if plotSeries(ax, times, tbl, 'est_y', active, 'V%d') == 0
    noData(ax, 'Estimated Y');
end
styleAxes(ax, 'Estimated Y', 'y [m]', '', true);

ax = nexttile(t, 3);
if plotSeries(ax, times, tbl, 'est_v', active, 'V%d') == 0
    noData(ax, 'Estimated Velocity');
end
styleAxes(ax, 'Estimated Velocity', 'v [m/s]', '', true);

ax = nexttile(t, 4);
if plotSeries(ax, times, tbl, 'est_a', active, 'V%d') == 0
    noData(ax, 'Estimated Acceleration');
end
styleAxes(ax, 'Estimated Acceleration', 'a [m/s^2]', '', true);

ax = nexttile(t, 5);
if plotSeries(ax, times, tbl, 'est_theta', active, 'V%d') == 0
    noData(ax, 'Estimated Heading');
end
styleAxes(ax, 'Estimated Heading', 'theta [rad]', '', true);

ax = nexttile(t, 6);
count = 0;
for vid = active
    x = colToArray(tbl, sprintf('est_x_%d', vid));
    y = colToArray(tbl, sprintf('est_y_%d', vid));
    valid = isfinite(x) & isfinite(y);
    if any(valid)
        hold(ax, 'on');
        plot(ax, x, y, 'LineWidth', 1.2, 'DisplayName', sprintf('V%d', vid));
        firstIdx = find(valid, 1, 'first');
        plot(ax, x(firstIdx), y(firstIdx), 'o', 'HandleVisibility', 'off');
        count = count + 1;
    end
end
if count == 0
    noData(ax, 'Trajectory XY');
else
    axis(ax, 'equal');
end
styleAxes(ax, 'Trajectory XY', 'y [m]', 'x [m]', true);

ax = nexttile(t, 7);
if plotSeries(ax, times, tbl, 'consensus_x', active, 'V%d') == 0
    noData(ax, 'Consensus X');
end
styleAxes(ax, 'Consensus X', 'x [m]', '', true);

ax = nexttile(t, 8);
if plotSeries(ax, times, tbl, 'postpred_x', active, 'V%d') == 0
    noData(ax, 'Post-prediction X');
end
styleAxes(ax, 'Post-prediction X', 'x [m]', '', true);

ax = nexttile(t, 9);
count = plotSeries(ax, times, tbl, 'est_conf', active, 'Conf V%d');
count = count + plotNamedColumns(ax, times, tbl, { ...
    'self_belief', 'self\_belief'; ...
    'platoon_conf_mean', 'platoon\_conf\_mean'; ...
    'platoon_conf_min', 'platoon\_conf\_min'; ...
    'platoon_conf_max', 'platoon\_conf\_max'});
if count == 0
    noData(ax, 'Estimation Confidence');
end
styleAxes(ax, 'Estimation Confidence', 'Confidence', 'Time [s]', true);

ax = nexttile(t, 10);
count = 0;
for vid = active
    col = sprintf('pred_mode_%d', vid);
    arr = colToArray(tbl, col);
    if any(isfinite(arr))
        hold(ax, 'on');
        stairs(ax, times, arr, 'LineWidth', 1.2, 'DisplayName', sprintf('V%d', vid));
        count = count + 1;
    end
end
predCount = colToArray(tbl, 'prediction_mode_count');
if any(isfinite(predCount))
    hold(ax, 'on');
    plot(ax, times, predCount, 'k--', 'LineWidth', 1.2, 'DisplayName', 'total\_pred\_mode');
    count = count + 1;
end
if count == 0
    noData(ax, 'Prediction Mode');
end
styleAxes(ax, 'Prediction Mode Flags', 'Mode (0/1)', 'Time [s]', true);
end

function makeImpactHistogramFigure(tbl, focus, hostId)
fig = paperFigure(sprintf('Impact Histograms Host V%d Focus V%d', hostId, focus), [18.0 7.0]);
t = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
layoutTitle(t, sprintf('Max Impact Overview (Host V%d, Focus V%d)', hostId, focus));

ax = nexttile(t, 1);
vehCol = colToArray(tbl, sprintf('mi_veh_id_%d', focus));
validVeh = vehCol(isfinite(vehCol));
if isempty(validVeh)
    noData(ax, sprintf('Max Impact Vehicle (V%d)', focus));
else
    [u, ~, idx] = unique(int32(validVeh));
    counts = accumarray(idx, 1);
    labels = compose('V%d', double(u));
    bar(ax, categorical(cellstr(labels)), counts, 'FaceColor', [0.0 0.45 0.74], 'FaceAlpha', 0.75);
    styleAxes(ax, sprintf('Max Impact Vehicle Frequency (Focus V%d)', focus), ...
        'Frequency', '', false);
end

ax = nexttile(t, 2);
idxCol = colToArray(tbl, sprintf('mi_elem_idx_%d', focus));
validIdx = idxCol(isfinite(idxCol));
if isempty(validIdx)
    noData(ax, sprintf('Max Impact Element (V%d)', focus));
else
    [u, ~, idx] = unique(int32(validIdx));
    counts = accumarray(idx, 1);
    elemNames = ["x", "y", "theta", "v", "a"];
    labels = strings(size(u));
    for i = 1:numel(u)
        if u(i) >= 0 && u(i) < numel(elemNames)
            labels(i) = elemNames(u(i) + 1);
        else
            labels(i) = string(u(i));
        end
    end
    bar(ax, categorical(cellstr(labels)), counts, 'FaceColor', [0.85 0.33 0.10], 'FaceAlpha', 0.75);
    styleAxes(ax, sprintf('Max Impact Element Frequency (Focus V%d)', focus), ...
        'Frequency', '', false);
end
end

function makeAttackTimelineFigure(tbl, times, columns, active, hostId)
attackFields = { ...
    'x', 'Injected X', 'x [m]'; ...
    'y', 'Injected Y', 'y [m]'; ...
    'theta', 'Injected Heading', 'theta [rad]'; ...
    'velocity', 'Injected Velocity', 'v [m/s]'; ...
    'acceleration', 'Injected Acceleration', 'a [m/s^2]'; ...
    'confidence', 'Injected Confidence', 'confidence'};

intervals = extractAttackIntervals(tbl, columns, active, times);
events = extractAttackEvents(tbl, columns, times);
[tMin, tMax] = finiteTimeBounds(times);

lanes = ["module enabled", "attack active"];
for i = 1:numel(intervals)
    label = string(intervals(i).target_label);
    if ~any(lanes == label)
        lanes(end + 1) = label; %#ok<AGROW>
    end
end

fieldMask = false(size(attackFields, 1), 1);
for i = 1:size(attackFields, 1)
    fieldMask(i) = attackFieldHasData(tbl, columns, active, attackFields{i, 1});
end
fieldRows = attackFields(fieldMask, :);
valueRows = ceil(size(fieldRows, 1) / 2);
if valueRows < 1
    valueRows = 1;
end

figHeight = min(22.0, 7.2 + 3.0 * valueRows);
fig = paperFigure(sprintf('Attack Timeline Host V%d', hostId), [18.0 figHeight]);
t = tiledlayout(fig, 2 + valueRows, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
layoutTitle(t, sprintf('V2V Attack Timeline (Host V%d)', hostId));

ax = nexttile(t, [1 2]);
hold(ax, 'on');
laneY = containers.Map(cellstr(lanes), num2cell(1:numel(lanes)));
barH = 0.72;

enabled = colToArray(tbl, 'v2v_attack_enabled');
spans = maskToTimeSpans(times, isfinite(enabled) & enabled >= 0.5);
for i = 1:size(spans, 1)
    y = laneY('module enabled');
    patchSpan(ax, spans(i, 1), spans(i, 2), y - barH / 2, y + barH / 2, [0.2 0.7 0.2], 0.20);
end

activeArr = colToArray(tbl, 'v2v_attack_active');
spans = maskToTimeSpans(times, isfinite(activeArr) & activeArr >= 0.5);
for i = 1:size(spans, 1)
    y = laneY('attack active');
    patchSpan(ax, spans(i, 1), spans(i, 2), y - barH / 2, y + barH / 2, [0.85 0.20 0.20], 0.20);
end

colors = lines(10);
typeMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
style = ieeeStyle();
for i = 1:numel(intervals)
    y = laneY(char(intervals(i).target_label));
    startS = intervals(i).start_s;
    endS = intervals(i).end_s;
    attackType = char(string(intervals(i).type));
    if ~isKey(typeMap, attackType)
        typeMap(attackType) = colors(mod(typeMap.Count, size(colors, 1)) + 1, :);
    end
    color = typeMap(attackType);
    patchSpan(ax, startS, startS + max(endS - startS, 1e-3), y - barH / 2, y + barH / 2, color, 0.45);
    text(ax, startS + 0.5 * max(endS - startS, 1e-3), y, char(intervals(i).display_label), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
        'FontName', style.fontName, 'FontSize', style.fontSize, 'Clipping', 'on');
end

for i = 1:numel(events)
    eventTime = events(i).time_s;
    eventName = lower(string(events(i).event));
    if ~isfinite(eventTime)
        continue;
    end
    color = [0.2 0.7 0.2];
    if eventName ~= "enable"
        color = [0.85 0.20 0.20];
    end
    xline(ax, eventTime, '--', 'Color', color, 'LineWidth', 1.1, ...
        'Label', char(eventName), 'LabelOrientation', 'aligned');
end

set(ax, 'YTick', 1:numel(lanes), 'YTickLabel', cellstr(lanes));
xlim(ax, [tMin, tMax]);
ylim(ax, [0.4, numel(lanes) + 0.6]);
styleAxes(ax, 'Attack Intervals and Enable/Disable Events', '', 'Time [s]', false);

ax = nexttile(t, [1 2]);
count = 0;
for item = { ...
        {'v2v_attack_enabled', 'module enabled'}, ...
        {'v2v_attack_active', 'attack active'}, ...
        {'v2v_attack_active_count', 'active scenario count'}}
    pair = item{1};
    arr = colToArray(tbl, pair{1});
    if any(isfinite(arr))
        hold(ax, 'on');
        stairs(ax, times, arr, 'LineWidth', 1.2, 'DisplayName', pair{2});
        count = count + 1;
    end
end
if count == 0
    noData(ax, 'Attack Status Signals');
end
styleAxes(ax, 'Attack Status Signals', 'Value', 'Time [s]', true);

fieldIndex = 0;
for i = 1:size(fieldRows, 1)
    fieldIndex = fieldIndex + 1;
    ax = nexttile(t);
    plotAttackValuePanel(ax, times, tbl, columns, active, ...
        fieldRows{i, 1}, fieldRows{i, 2}, fieldRows{i, 3});
end
while fieldIndex < valueRows * 2
    fieldIndex = fieldIndex + 1;
    ax = nexttile(t);
    axis(ax, 'off');
end
end

function count = plotNamedColumns(ax, times, tbl, colPairs)
count = 0;
for i = 1:size(colPairs, 1)
    arr = colToArray(tbl, colPairs{i, 1});
    if any(isfinite(arr))
        hold(ax, 'on');
        plot(ax, times, arr, 'LineWidth', 1.2, 'DisplayName', colPairs{i, 2});
        count = count + 1;
    end
end
if count == 0
    noData(ax, 'No data');
end
end

function plotOffsetFlags(ax, times, tbl, rows)
count = 0;
for i = 1:size(rows, 1)
    arr = colToArray(tbl, rows{i, 1});
    if any(isfinite(arr))
        hold(ax, 'on');
        stairs(ax, times, 0.85 * arr + rows{i, 3}, 'LineWidth', 1.4, ...
            'DisplayName', rows{i, 2});
        count = count + 1;
    end
end
if count == 0
    noData(ax, 'No data');
end
end

function tf = hasFiniteColumn(tbl, col)
arr = colToArray(tbl, col);
tf = any(isfinite(arr));
end

function printStaticMetrics(tbl, active)
fprintf('%s\n', repmat('=', 1, 80));
fprintf('%s\n', centerText('STATIC METRICS DASHBOARD', 80));
fprintf('%s\n', repmat('=', 1, 80));
fprintf('%-10s | %-12s | %-15s | %-16s | %s\n', ...
    'Vehicle', 'Target Atk', 'Local Est Bad', 'Global Est Bad', 'Max Impact Elements (Count)');
fprintf('%s\n', repmat('-', 1, 80));

elemNames = {'x', 'y', 'theta', 'v', 'a'};
for vid = active
    targetAtk = safeCount(colToArray(tbl, sprintf('flag_attack_%d', vid)));
    localBad = safeCount(colToArray(tbl, sprintf('flag_local_%d', vid)));
    globalBad = safeCount(colToArray(tbl, sprintf('flag_global_%d', vid)));
    elemIdx = colToArray(tbl, sprintf('mi_elem_idx_%d', vid));
    elemDist = colToArray(tbl, sprintf('mi_dist_%d', vid));
    elemStr = 'None';

    valid = isfinite(elemIdx);
    if any(valid)
        counts = zeros(1, 5);
        distSums = zeros(1, 5);
        idxVals = int32(elemIdx(valid));
        if ~any(isfinite(elemDist))
            elemDist = zeros(size(elemIdx));
        end
        distVals = elemDist(valid);
        for i = 1:numel(idxVals)
            idx = double(idxVals(i)) + 1;
            if idx >= 1 && idx <= 5
                counts(idx) = counts(idx) + 1;
                if isfinite(distVals(i))
                    distSums(idx) = distSums(idx) + distVals(i);
                end
            end
        end
        [sortedCounts, order] = sort(counts, 'descend');
        parts = strings(0, 1);
        for i = 1:numel(order)
            if sortedCounts(i) <= 0
                continue;
            end
            idx = order(i);
            avgDist = distSums(idx) / sortedCounts(i);
            parts(end + 1) = sprintf('%s (%dx, avg dist: %.2f)', elemNames{idx}, sortedCounts(i), avgDist); %#ok<AGROW>
        end
        if ~isempty(parts)
            elemStr = strjoin(cellstr(parts(1:min(2, numel(parts)))), ', ');
        end
    end

    fprintf('V%-9d | %-12d | %-15d | %-16d | %s\n', vid, targetAtk, localBad, globalBad, elemStr);
end
fprintf('%s\n\n', repmat('=', 1, 80));
end

function n = safeCount(arr)
arr = arr(isfinite(arr));
if isempty(arr)
    n = 0;
else
    n = int32(sum(arr));
end
end

function txt = centerText(str, width)
str = char(str);
if numel(str) >= width
    txt = str;
    return;
end
left = floor((width - numel(str)) / 2);
right = width - numel(str) - left;
txt = [repmat(' ', 1, left), str, repmat(' ', 1, right)];
end

function spans = maskToTimeSpans(times, mask)
times = times(:);
mask = logical(mask(:));
valid = isfinite(times);
spans = zeros(0, 2);
if isempty(times) || numel(mask) ~= numel(times) || ~any(valid & mask)
    return;
end

validTimes = times(valid);
dt = diff(validTimes);
dt = dt(dt > 0);
if isempty(dt)
    fallbackDt = 1e-3;
else
    fallbackDt = median(dt);
end

startIdx = [];
for i = 1:numel(mask)
    if mask(i) && isempty(startIdx)
        startIdx = i;
    end
    if ~isempty(startIdx) && (~mask(i) || i == numel(mask))
        if mask(i) && i == numel(mask)
            endIdx = i;
        else
            endIdx = i - 1;
        end
        startT = times(startIdx);
        endT = times(endIdx);
        if isfinite(startT) && isfinite(endT)
            if endT <= startT
                endT = startT + fallbackDt;
            end
            spans(end + 1, :) = [startT, endT]; %#ok<AGROW>
        end
        startIdx = [];
    end
end
end

function [tMin, tMax] = finiteTimeBounds(times)
finiteTimes = times(isfinite(times));
if isempty(finiteTimes)
    tMin = 0.0;
    tMax = 1.0;
    return;
end
tMin = min(finiteTimes);
tMax = max(finiteTimes);
if ~isfinite(tMax) || tMax <= tMin
    tMax = tMin + 1.0;
end
end

function intervals = extractAttackIntervals(tbl, columns, active, times)
intervals = struct('start_s', {}, 'end_s', {}, 'target_label', {}, ...
    'display_label', {}, 'type', {});
[tMin, tMax] = finiteTimeBounds(times);

raw = lastNonemptyValue(tbl, 'v2v_attack_intervals');
data = jsonListOrEmpty(raw);
for i = 1:numel(data)
    item = data(i);
    [startS, endS] = clampedInterval(getFieldOr(item, 'start_s', NaN), ...
        getFieldOr(item, 'end_s', NaN), tMin, tMax);
    targetLabel = attackTargetLabel(item);
    displayLabel = attackDisplayLabel(item);
    intervals(end + 1) = struct( ... %#ok<AGROW>
        'start_s', startS, ...
        'end_s', endS, ...
        'target_label', targetLabel, ...
        'display_label', displayLabel, ...
        'type', string(getFieldOr(item, 'type', 'attack')));
end
if ~isempty(intervals)
    return;
end

if isColumnPresent(columns, 'v2v_attack_start_s')
    startArr = colToArray(tbl, 'v2v_attack_start_s');
    endArr = colToArray(tbl, 'v2v_attack_end_s');
    typeTxt = colToText(tbl, 'v2v_attack_types');
    dataTypeTxt = colToText(tbl, 'v2v_attack_data_types');
    seen = strings(0, 1);
    for i = 1:numel(startArr)
        if ~isfinite(startArr(i))
            continue;
        end
        [startS, endS] = clampedInterval(startArr(i), endArr(i), tMin, tMax);
        key = sprintf('%s|%s|%.6f|%.6f', typeTxt(i), dataTypeTxt(i), startS, endS);
        if any(seen == key)
            continue;
        end
        seen(end + 1) = key; %#ok<AGROW>
        item = struct('type', char(typeTxt(i)), 'data_type', char(dataTypeTxt(i)));
        intervals(end + 1) = struct( ... %#ok<AGROW>
            'start_s', startS, ...
            'end_s', endS, ...
            'target_label', "attack", ...
            'display_label', attackDisplayLabel(item), ...
            'type', string(typeTxt(i)));
    end
end

if isempty(intervals)
    seen = strings(0, 1);
    for vid = active
        startCol = sprintf('inject_attack_start_%d', vid);
        endCol = sprintf('inject_attack_end_%d', vid);
        if ~isColumnPresent(columns, startCol)
            continue;
        end
        startArr = colToArray(tbl, startCol);
        endArr = colToArray(tbl, endCol);
        typeTxt = colToText(tbl, sprintf('inject_attack_type_%d', vid));
        modTxt = colToText(tbl, sprintf('inject_attack_modification_%d', vid));
        dataTxt = colToText(tbl, sprintf('inject_attack_data_type_%d', vid));
        fieldTxt = colToText(tbl, sprintf('inject_attack_fields_%d', vid));
        for i = 1:numel(startArr)
            if ~isfinite(startArr(i))
                continue;
            end
            [startS, endS] = clampedInterval(startArr(i), endArr(i), tMin, tMax);
            key = sprintf('V%d|%s|%s|%s|%.6f|%.6f', vid, typeTxt(i), modTxt(i), dataTxt(i), startS, endS);
            if any(seen == key)
                continue;
            end
            seen(end + 1) = key; %#ok<AGROW>
            item = struct( ...
                'type', char(typeTxt(i)), ...
                'modification', char(modTxt(i)), ...
                'data_type', char(dataTxt(i)), ...
                'target_fields', splitPipeList(fieldTxt(i)), ...
                'attacker_id', vid, ...
                'victim_ids', vid);
            intervals(end + 1) = struct( ... %#ok<AGROW>
                'start_s', startS, ...
                'end_s', endS, ...
                'target_label', "V" + string(vid), ...
                'display_label', attackDisplayLabel(item), ...
                'type', string(typeTxt(i)));
        end
    end
end
end

function events = extractAttackEvents(tbl, columns, times)
events = struct('event', {}, 'time_s', {});
raw = lastNonemptyValue(tbl, 'v2v_attack_events');
data = jsonListOrEmpty(raw);
for i = 1:numel(data)
    eventTime = safeFloat(getFieldOr(data(i), 'time_s', NaN), NaN);
    if isfinite(eventTime)
        events(end + 1) = struct('event', string(getFieldOr(data(i), 'event', '')), 'time_s', eventTime); %#ok<AGROW>
    end
end
if ~isempty(events)
    return;
end

if ~isColumnPresent(columns, 'v2v_attack_enabled')
    return;
end
enabled = colToArray(tbl, 'v2v_attack_enabled');
mask = isfinite(enabled) & enabled >= 0.5;
prev = false;
for i = 1:numel(mask)
    if ~isfinite(times(i))
        continue;
    end
    if mask(i) && ~prev
        events(end + 1) = struct('event', "enable", 'time_s', times(i)); %#ok<AGROW>
    elseif prev && ~mask(i)
        events(end + 1) = struct('event', "disable", 'time_s', times(i)); %#ok<AGROW>
    end
    prev = mask(i);
end
end

function plotAttackValuePanel(ax, times, tbl, columns, active, field, titleText, ylabelText)
count = 0;
colors = lines(max(1, numel(active)));
for i = 1:numel(active)
    vid = active(i);
    modifiedCol = sprintf('inject_attack_modified_%s_%d', field, vid);
    originalCol = sprintf('inject_attack_original_%s_%d', field, vid);
    deltaCol = sprintf('inject_attack_delta_%s_%d', field, vid);
    if ~isColumnPresent(columns, modifiedCol) && ~isColumnPresent(columns, originalCol)
        continue;
    end
    modified = colToArray(tbl, modifiedCol);
    original = colToArray(tbl, originalCol);
    delta = colToArray(tbl, deltaCol);
    if ~any(isfinite(modified) | isfinite(original) | isfinite(delta))
        continue;
    end
    hold(ax, 'on');
    color = colors(mod(i - 1, size(colors, 1)) + 1, :);
    if any(isfinite(original))
        plot(ax, times, original, '--', 'Color', color, 'LineWidth', 1.1, ...
            'DisplayName', sprintf('V%d original', vid));
    end
    if any(isfinite(modified))
        plot(ax, times, modified, '-', 'Color', color, 'LineWidth', 1.7, ...
            'DisplayName', sprintf('V%d injected', vid));
    elseif any(isfinite(delta))
        plot(ax, times, delta, '-', 'Color', color, 'LineWidth', 1.5, ...
            'DisplayName', sprintf('V%d delta', vid));
    end
    count = count + 1;
end
if count == 0
    noData(ax, titleText);
end
styleAxes(ax, titleText, ylabelText, 'Time [s]', true);
end

function tf = attackFieldHasData(tbl, columns, active, field)
tf = false;
for vid = active
    for kind = ["original", "modified", "delta"]
        col = sprintf('inject_attack_%s_%s_%d', kind, field, vid);
        if isColumnPresent(columns, col) && hasFiniteColumn(tbl, col)
            tf = true;
            return;
        end
    end
end
end

function txt = lastNonemptyValue(tbl, col)
txt = "";
if ~ismember(col, tbl.Properties.VariableNames)
    return;
end
values = colToText(tbl, col);
for i = numel(values):-1:1
    value = strtrim(values(i));
    if value ~= "" && value ~= "<missing>" && value ~= "missing"
        txt = value;
        return;
    end
end
end

function data = jsonListOrEmpty(raw)
data = struct([]);
raw = strtrim(string(raw));
if raw == "" || raw == "[]" || raw == "<missing>"
    return;
end
try
    decoded = jsondecode(char(raw));
catch
    return;
end
if isstruct(decoded)
    data = decoded;
elseif iscell(decoded)
    keep = cellfun(@isstruct, decoded);
    data = [decoded{keep}];
end
end

function [startS, endS] = clampedInterval(startVal, endVal, tMin, tMax)
startS = safeFloat(startVal, tMin);
endS = safeFloat(endVal, tMax);
if ~isfinite(startS)
    startS = tMin;
end
if ~isfinite(endS)
    endS = tMax;
end
startS = min(max(startS, tMin), tMax);
endS = min(max(endS, startS), tMax);
if endS <= startS
    endS = min(max(startS + 1e-3, tMin), max(tMax, startS + 1e-3));
end
end

function label = attackTargetLabel(item)
dataType = lower(string(getFieldOr(item, 'data_type', 'attack')));
attacker = string(getFieldOr(item, 'attacker_id', ''));
victims = getFieldOr(item, 'victim_ids', []);
victimIds = numericList(victims);
if dataType == "local"
    label = "local V" + attacker;
elseif dataType == "fleet"
    if any(victimIds == -1)
        victimText = "all";
    else
        victimText = strjoin(compose("V%d", victimIds), ",");
    end
    label = "fleet " + victimText;
elseif dataType == "both"
    if any(victimIds == -1)
        victimText = "all";
    else
        victimText = strjoin(compose("V%d", victimIds), ",");
    end
    label = "both V" + attacker + "->" + victimText;
else
    label = dataType;
end
end

function label = attackDisplayLabel(item)
parts = strings(0, 1);
baseParts = {getFieldOr(item, 'type', 'attack'), getFieldOr(item, 'modification', ''), getFieldOr(item, 'data_type', '')};
for i = 1:numel(baseParts)
    value = strtrim(string(baseParts{i}));
    if value ~= ""
        parts(end + 1) = value; %#ok<AGROW>
    end
end
fields = getFieldOr(item, 'target_fields', []);
fieldNames = string(splitPipeList(fields));
fieldNames = fieldNames(fieldNames ~= "");
if ~isempty(fieldNames)
    parts(end + 1) = strjoin(cellstr(fieldNames), '/'); %#ok<AGROW>
end
if isempty(parts)
    label = "attack";
else
    label = strjoin(cellstr(parts), ' ');
end
end

function value = getFieldOr(s, fieldName, defaultValue)
value = defaultValue;
try
    if isstruct(s) && isfield(s, fieldName)
        value = s.(fieldName);
    end
catch
end
end

function value = safeFloat(raw, defaultValue)
if nargin < 2
    defaultValue = NaN;
end
if isnumeric(raw) && isscalar(raw)
    value = double(raw);
    return;
end
value = str2double(string(raw));
if ~isfinite(value)
    value = defaultValue;
end
end

function ids = numericList(raw)
if isnumeric(raw)
    ids = double(raw(:)');
    return;
end
if iscell(raw)
    pieces = string(raw);
elseif isstring(raw)
    pieces = raw;
else
    pieces = split(replace(string(raw), ",", "|"), "|");
end
ids = [];
for i = 1:numel(pieces)
    v = str2double(strtrim(pieces(i)));
    if isfinite(v)
        ids(end + 1) = v; %#ok<AGROW>
    end
end
ids = unique(ids);
end

function parts = splitPipeList(raw)
if isstring(raw) || ischar(raw)
    txt = string(raw);
    if txt == ""
        parts = {};
    else
        pieces = split(replace(txt, ",", "|"), "|");
        pieces = strtrim(pieces);
        pieces = pieces(pieces ~= "");
        parts = cellstr(pieces);
    end
elseif iscell(raw)
    parts = raw;
elseif isnumeric(raw)
    parts = cellstr(string(raw));
else
    parts = {};
end
end

function patchSpan(ax, x0, x1, y0, y1, color, alphaValue)
patch(ax, [x0 x1 x1 x0], [y0 y0 y1 y1], color, ...
    'FaceAlpha', alphaValue, 'EdgeColor', color, 'LineWidth', 1.0, ...
    'HandleVisibility', 'off');
end
