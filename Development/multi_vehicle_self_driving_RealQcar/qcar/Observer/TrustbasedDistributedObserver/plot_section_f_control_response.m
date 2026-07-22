function summary = plot_section_f_control_response(varargin)
%PLOT_SECTION_F_CONTROL_RESPONSE Plot trust-fused platoon-control evidence.
%
% This is the MATLAB version of the Section F figure:
%   (a) trust and direct W0 weight for the attacked vehicle
%   (b) controller fusion alpha and ACC/CACC commands
%   (c) full-platoon inter-vehicle spacing
%   (d) V2V attack timeline
%
% Common usage:
%   plot_section_f_control_response
%   plot_section_f_control_response('File', 'results/06-07-26/trust_weight_log_V1_16-10-42_211014.csv')
%   plot_section_f_control_response('Attacker', 0, 'Save', true)
%   plot_section_f_control_response('PlotWindow', [5 20], 'OutputDir', 'paper_figures')

parser = inputParser;
addParameter(parser, 'File', '', @(x) ischar(x) || isstring(x));
addParameter(parser, 'Attacker', NaN, @(x) isscalar(x) && isnumeric(x));
addParameter(parser, 'Host', NaN, @(x) isscalar(x) && isnumeric(x));
addParameter(parser, 'Peer', NaN, @(x) isscalar(x) && isnumeric(x));
addParameter(parser, 'FleetGapFiles', strings(0, 1), @(x) isstring(x) || iscell(x) || ischar(x));
addParameter(parser, 'ResultDate', '', @(x) ischar(x) || isstring(x));
addParameter(parser, 'SelectFile', true, @(x) islogical(x) || isnumeric(x));
addParameter(parser, 'Save', true, @(x) islogical(x) || isnumeric(x));
addParameter(parser, 'OutputDir', '', @(x) ischar(x) || isstring(x));
addParameter(parser, 'Formats', ["png", "pdf"], @(x) isstring(x) || iscell(x) || ischar(x));
addParameter(parser, 'Dpi', 600, @(x) isscalar(x) && isnumeric(x));
addParameter(parser, 'AttackWindow', [NaN NaN], @isWindowSpec);
addParameter(parser, 'PlotWindow', [NaN NaN], @(x) isnumeric(x) && numel(x) == 2);
addParameter(parser, 'TurnWindow', [NaN NaN], @isWindowSpec);
addParameter(parser, 'ShowTurnSections', false, @(x) islogical(x) || isnumeric(x));
addParameter(parser, 'ShowControllerGaps', false, @(x) islogical(x) || isnumeric(x));
addParameter(parser, 'ShowAccDesiredGap', false, @(x) islogical(x) || isnumeric(x));
addParameter(parser, 'ShowAttackStatusRows', false, @(x) islogical(x) || isnumeric(x));
addParameter(parser, 'ShowAttackEvents', false, @(x) islogical(x) || isnumeric(x));
addParameter(parser, 'ShowAttackIntervalMarkers', true, @(x) islogical(x) || isnumeric(x));
addParameter(parser, 'ShowTrustThreshold', false, @(x) islogical(x) || isnumeric(x));
addParameter(parser, 'TrustThreshold', 0.70, @(x) isscalar(x) && isnumeric(x));
addParameter(parser, 'SpacingThreshold', 0.30, @(x) isscalar(x) && isnumeric(x));
addParameter(parser, 'SpacingTolerance', 0.15, @(x) isscalar(x) && isnumeric(x));
addParameter(parser, 'GapStabilityStdMax', 0.25, @(x) isscalar(x) && isnumeric(x));
addParameter(parser, 'GoodCoveragePercent', 80.0, @(x) isscalar(x) && isnumeric(x));
addParameter(parser, 'FusionAlphaAttackMax', 0.50, @(x) isscalar(x) && isnumeric(x));
addParameter(parser, 'FusionAlphaDropMinPercent', 30.0, @(x) isscalar(x) && isnumeric(x));
addParameter(parser, 'CaccS0', 0.30, @(x) isscalar(x) && isnumeric(x));
addParameter(parser, 'CaccHeadway', 0.45, @(x) isscalar(x) && isnumeric(x));
addParameter(parser, 'AccDesiredDistance', 0.45, @(x) isscalar(x) && isnumeric(x));
addParameter(parser, 'AccHeadway', 0.45, @(x) isscalar(x) && isnumeric(x));
addParameter(parser, 'WeightScale', 1.0, @(x) isscalar(x) && isnumeric(x));
addParameter(parser, 'CommandScale', 10.0, @(x) isscalar(x) && isnumeric(x));
parse(parser, varargin{:});
args = parser.Results;

scriptDir = fileparts(mfilename('fullpath'));
filepath = resolveSelectedFile(string(args.File), scriptDir, string(args.ResultDate), logical(args.SelectFile));
tbl = readTrustTable(filepath);
if ~ismember('time', tbl.Properties.VariableNames)
    error('Missing time column in %s.', filepath);
end

t = col(tbl, 'time');
host = resolveHost(filepath, tbl, double(args.Host));
attacker = resolveAttacker(tbl, double(args.Attacker));
if ~isfinite(attacker)
    error('Could not infer attacker vehicle. Pass ''Attacker'', for example: plot_section_f_control_response(''Attacker'', 0).');
end
attacker = round(attacker);
peer = resolvePeer(tbl, host, attacker, double(args.Peer));

attackWindows = resolveAttackWindows(tbl, t, attacker, double(args.AttackWindow));
plotWindow = resolvePlotWindow(t, attackWindows, double(args.PlotWindow));
turnMask = resolveTurnMask(tbl, t, args.TurnWindow);

weightSpec = chooseSourceWeightColumn(tbl, attacker, host);
gapSpec = buildFleetGapSeries(tbl, host);
fleetGapFiles = normalizeStringArray(args.FleetGapFiles);
if ~isempty(fleetGapFiles)
    fleetGapFiles = resolveInputFiles(fleetGapFiles, scriptDir);
    batchGapSpec = buildBatchFleetGapSeries(fleetGapFiles, t, host);
    if ~isempty(batchGapSpec)
        gapSpec = batchGapSpec;
    else
        warning('Could not build batch full-platoon gaps; using gaps available in the selected host log.');
    end
end
gapSpec = orderGapSpecForHost(gapSpec, host);
style = paperStyle();
set(groot, 'defaultAxesFontName', style.fontName);
set(groot, 'defaultTextFontName', style.fontName);

summary = buildControlSummary(tbl, t, filepath, host, attacker, peer, ...
    weightSpec, gapSpec, attackWindows, turnMask, args.TrustThreshold, args);
printSummary(summary, weightSpec);

fig = makeControlFigure(tbl, t, filepath, host, attacker, peer, weightSpec, ...
    gapSpec, attackWindows, plotWindow, turnMask, summary, args, style);

outputDir = resolveOutputDir(string(args.OutputDir), filepath, scriptDir);
baseName = sprintf('section_f_control_response_%s', erase(fileBaseName(filepath), "trust_weight_log_"));
formats = normalizeStringArray(args.Formats);
if logical(args.Save)
    ensureFolder(outputDir);
    writetable(summary, fullfile(outputDir, baseName + "_metrics.csv"));
end
finishFigure(fig, baseName, outputDir, formats, logical(args.Save), args.Dpi);
if logical(args.Save)
    fprintf('Saved Section F control figure and metrics to: %s\n', outputDir);
end
end

function fig = makeControlFigure(tbl, t, ~, host, attacker, ~, weightSpec, ...
    gapSpec, ~, plotWindow, turnMask, summary, args, style)

trust = col(tbl, sprintf('trust_%d', attacker));
w0Weight = col(tbl, weightSpec.column);
alpha = col(tbl, 'ctrl_alpha');
uFinal = col(tbl, 'ctrl_u_final') * args.CommandScale;
uCacc = col(tbl, 'ctrl_u_cacc') * args.CommandScale;
uSensor = col(tbl, 'ctrl_u_sensor') * args.CommandScale;
sensorGap = col(tbl, 'ctrl_sensor_gap');
projectedGap = col(tbl, 'ctrl_along_track_gap');
leaderDist = col(tbl, 'ctrl_distance_to_leader');
hostSpeed = hostVelocity(tbl, host);
caccDesiredGap = args.CaccS0 + args.CaccHeadway * max(hostSpeed, 0);
accDesiredGap = args.AccDesiredDistance + args.AccHeadway * max(hostSpeed, 0);
visibleMask = t >= plotWindow(1) & t <= plotWindow(2);
turnMask = normalizeMask(turnMask, t);

fig = figure('Name', 'Section F Control Response', 'Color', 'w', ...
    'Units', 'centimeters', 'Position', [2 2 19 21]);
layout = tiledlayout(fig, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

hostLeader = hostLeaderFromGapSpec(gapSpec, host);
if isfinite(host) && isfinite(hostLeader)
    titleText = sprintf('Trust-fused control: host V%d follows V%d, attacker V%d', ...
        host, hostLeader, attacker);
elseif isfinite(host)
    titleText = sprintf('Trust-fused control: host V%d, attacker V%d', host, attacker);
else
    titleText = sprintf('Trust-fused control under attacker V%d', attacker);
end
hostGapLabel = hostPairLabelFromGapSpec(gapSpec, host);
subtitleMinGap = firstFinite([
    summary.minHostPairGapAttack_m
    summary.minFleetDirectGapAttack_m
    summary.minControlGapAttack_m]);
subtitleText = sprintf('%s | alpha %.3f | W0 %.3f | min %s %.3f m | RMSE %.3f m', ...
    char(summary.overallAttackVerdict), ...
    summary.alphaAttackMean, summary.sourceWeightAttackMean, ...
    hostGapLabel, subtitleMinGap, summary.rmseHostGapErrorAttack_m);
title(layout, titleText, 'FontName', style.fontName, 'FontSize', style.titleSize + 1, ...
    'FontWeight', 'bold', 'Interpreter', 'none');
subtitle(layout, subtitleText, 'FontName', style.fontName, 'FontSize', style.fontSize, ...
    'Interpreter', 'none');

ax1 = nexttile(layout);
hold(ax1, 'on');
plotFinite(ax1, t, trust, '-', style.blue, style.lineWidth, sprintf('trust assigned to V%d', attacker));
if abs(args.WeightScale - 1.0) > 1e-12
    weightLabel = sprintf('%g x %s', args.WeightScale, weightSpec.label);
else
    weightLabel = char(weightSpec.label);
end
plotFinite(ax1, t, w0Weight * args.WeightScale, '-', style.orange, style.lineWidth, weightLabel);
if logical(args.ShowTrustThreshold)
    yline(ax1, args.TrustThreshold, ':', sprintf('threshold %.2f', args.TrustThreshold), ...
        'Color', style.attackColor, 'LineWidth', 0.8, 'HandleVisibility', 'off');
end
ylim(ax1, [0 max(1.05, maxFinite([trust(visibleMask); w0Weight(visibleMask) * args.WeightScale]) * 1.05)]);
markTurnSections(ax1, t, turnMask, style, logical(args.ShowTurnSections));
applyTimeWindow(ax1, plotWindow);
styleAxes(ax1, '(a) Attacker reliability: trust and W0', ...
    '', 'trust / W0 [-]', style);
legend(ax1, 'Location', 'northwest', 'FontSize', style.fontSize - 1, 'Interpreter', 'none');

ax2 = nexttile(layout);
hold(ax2, 'on');
plotFinite(ax2, t, alpha, '-', style.blue, style.lineWidth, 'fusion alpha');
plotFinite(ax2, t, uFinal, '-', style.green, style.lineWidth, sprintf('final command x%g', args.CommandScale));
plotFinite(ax2, t, uCacc, '-', style.purple, style.lineWidth, sprintf('CACC command x%g', args.CommandScale));
plotFinite(ax2, t, uSensor, '-', style.orange, style.lineWidth, sprintf('sensor ACC x%g', args.CommandScale));
commandMax = maxFinite([alpha(visibleMask); uFinal(visibleMask); uCacc(visibleMask); uSensor(visibleMask)]);
ylim(ax2, [0 max(1.10, commandMax * 1.08)]);
markTurnSections(ax2, t, turnMask, style, logical(args.ShowTurnSections));
applyTimeWindow(ax2, plotWindow);
styleAxes(ax2, '(b) Controller response: trust gate and longitudinal commands', ...
    '', 'alpha / scaled command [-]', style);
legend(ax2, 'Location', 'northwest', 'FontSize', style.fontSize - 1, 'Interpreter', 'none');

ax3 = nexttile(layout);
hold(ax3, 'on');
plotFleetGaps(ax3, t, gapSpec, style);
if isfinite(host)
    caccDesiredGapLabel = sprintf('host CACC desired gap V%d', host);
    accDesiredGapLabel = sprintf('host ACC desired gap V%d', host);
else
    caccDesiredGapLabel = 'host CACC desired gap';
    accDesiredGapLabel = 'host ACC desired gap';
end
plotFinite(ax3, t, caccDesiredGap, ':', style.eventColor, style.lineWidth, caccDesiredGapLabel);
if logical(args.ShowAccDesiredGap)
    plotFinite(ax3, t, accDesiredGap, '--', style.attackColor, 0.95, accDesiredGapLabel);
end
if logical(args.ShowControllerGaps)
    plotFinite(ax3, t, leaderDist, '-.', style.purple, 0.9, 'controller leader distance');
    plotFinite(ax3, t, projectedGap, ':', style.green, 0.9, 'controller along-track gap');
    plotFinite(ax3, t, sensorGap, '--', style.orange, 0.9, 'sensor/ACC gap');
end
yline(ax3, args.SpacingThreshold, ':', sprintf('min gap = %.2f m', args.SpacingThreshold), ...
    'Color', style.attackColor, 'LineWidth', 0.8, 'HandleVisibility', 'off');
gapMaxInputs = [gapSpecValues(gapSpec, visibleMask); caccDesiredGap(visibleMask); ...
    leaderDist(visibleMask); projectedGap(visibleMask); sensorGap(visibleMask)];
if logical(args.ShowAccDesiredGap)
    gapMaxInputs = [gapMaxInputs; accDesiredGap(visibleMask)];
end
gapMax = maxFinite(gapMaxInputs);
if ~isfinite(gapMax)
    gapMax = 1.0;
end
ylim(ax3, [0 max(1.25, gapMax * 1.08)]);
markTurnSections(ax3, t, turnMask, style, logical(args.ShowTurnSections));
applyTimeWindow(ax3, plotWindow);
styleAxes(ax3, '(c) Safety check: full-platoon spacing', ...
    '', 'gap [m]', style);
legend(ax3, 'Location', 'northwest', 'FontSize', style.fontSize - 1, 'Interpreter', 'none');

ax4 = nexttile(layout);
plotAttackTimeline(ax4, tbl, t, plotWindow, style, ...
    logical(args.ShowAttackStatusRows), logical(args.ShowAttackEvents), ...
    logical(args.ShowAttackIntervalMarkers));
applyTimeWindow(ax4, plotWindow);
end

function ok = isWindowSpec(x)
ok = isnumeric(x) && (isempty(x) || mod(numel(x), 2) == 0);
end

function filepath = resolveSelectedFile(fileArg, scriptDir, resultDate, selectFile)
fileArg = strtrim(fileArg);
if strlength(fileArg) > 0
    filepath = resolveInputFile(fileArg, scriptDir);
    return;
end

if ~selectFile
    error('No file was provided. Set ''SelectFile'' to true or pass ''File''.');
end

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
    'Select trust_weight_log CSV for Section F control response');
if isequal(selected, 0)
    error('File selection cancelled.');
end
filepath = char(fullfile(selectedPath, selected));
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

function filepaths = resolveInputFiles(fileArgs, scriptDir)
fileArgs = normalizeStringArray(fileArgs);
filepaths = strings(numel(fileArgs), 1);
for k = 1:numel(fileArgs)
    filepaths(k) = string(resolveInputFile(fileArgs(k), scriptDir));
end
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

function host = resolveHost(filepath, ~, requestedHost)
if isfinite(requestedHost)
    host = round(requestedHost);
    return;
end

[~, name] = fileparts(filepath);
tokens = regexp(name, '_V(\d+)_', 'tokens', 'once');
if ~isempty(tokens)
    host = str2double(tokens{1});
    return;
end

host = NaN;
end

function attacker = resolveAttacker(tbl, requestedAttacker)
if isfinite(requestedAttacker)
    attacker = round(requestedAttacker);
    return;
end

names = string(tbl.Properties.VariableNames);
attackCols = names(startsWith(names, "inject_attack_attacker_"));
for k = 1:numel(attackCols)
    value = firstFinite(col(tbl, attackCols(k)));
    if isfinite(value)
        attacker = round(value);
        return;
    end
end

activeCols = names(startsWith(names, "trust_"));
for k = 1:numel(activeCols)
    token = regexp(activeCols(k), '^trust_(\d+)$', 'tokens', 'once');
    if ~isempty(token)
        candidate = str2double(token{1});
        if any(isfinite(col(tbl, activeCols(k))))
            attacker = candidate;
            return;
        end
    end
end
attacker = NaN;
end

function peer = resolvePeer(tbl, host, attacker, requestedPeer)
if isfinite(requestedPeer)
    peer = round(requestedPeer);
    return;
end

ids = extractVehicleIds(tbl);
ids = ids(ids ~= attacker);
if isfinite(host)
    ids = ids(ids ~= host);
end
for k = 1:numel(ids)
    if any(isfinite(positionErrorWithReference(tbl, ids(k), 'est')))
        peer = ids(k);
        return;
    end
end
peer = NaN;
end

function ids = extractVehicleIds(tbl)
names = string(tbl.Properties.VariableNames);
ids = [];
for k = 1:numel(names)
    token = regexp(names(k), '_(\d+)$', 'tokens', 'once');
    if ~isempty(token)
        ids(end + 1) = str2double(token{1}); %#ok<AGROW>
    end
end
ids = unique(ids(isfinite(ids)));
end

function windows = resolveAttackWindows(tbl, t, attacker, requestedWindows)
windows = normalizeWindowRows(requestedWindows);
if ~isempty(windows)
    windows = clipWindowsToTime(mergeTimeWindows(windows), t);
    return;
end

windows = attackWindowsFromIntervalJson(tbl, attacker);
if isempty(windows)
    windows = attackWindowsFromActiveColumns(tbl, t, attacker);
end
if isempty(windows)
    windows = attackWindowsFromStartEndColumns(tbl, attacker);
end
if isempty(windows)
    windows = fallbackAttackWindow(t);
end

windows = clipWindowsToTime(mergeTimeWindows(windows), t);
if isempty(windows)
    windows = fallbackAttackWindow(t);
end
end

function windows = attackWindowsFromIntervalJson(tbl, attacker)
windows = zeros(0, 2);
texts = textColumn(tbl, 'v2v_attack_intervals');
texts = texts(~ismissing(texts));
texts = unique(texts, 'stable');
for k = 1:numel(texts)
    text = strtrim(texts(k));
    if strlength(text) == 0 || text == "[]" || lower(text) == "nan"
        continue;
    end

    try
        intervals = jsondecode(char(text));
    catch
        continue;
    end
    if isempty(intervals) || ~isstruct(intervals)
        continue;
    end

    for j = 1:numel(intervals)
        item = intervals(j);
        itemAttacker = numericStructField(item, 'attacker_id');
        if isfinite(attacker) && isfinite(itemAttacker) && round(itemAttacker) ~= attacker
            continue;
        end

        startTime = numericStructField(item, 'start_s');
        endTime = numericStructField(item, 'end_s');
        if isfinite(startTime) && isfinite(endTime) && endTime > startTime
            windows(end + 1, :) = [startTime endTime]; %#ok<AGROW>
        end
    end
end
windows = mergeTimeWindows(windows);
end

function windows = attackWindowsFromActiveColumns(tbl, t, attacker)
windows = zeros(0, 2);
specificMask = false(size(t));
if isfinite(attacker)
    y = col(tbl, sprintf('inject_attack_active_%d', attacker));
    if any(isfinite(y))
        specificMask = y >= 0.5;
    end
end
if any(specificMask)
    windows = maskToTimeSpans(t, specificMask);
    return;
end

globalMask = false(size(t));
y = col(tbl, 'v2v_attack_active');
if any(isfinite(y))
    globalMask = globalMask | y >= 0.5;
end
y = col(tbl, 'v2v_attack_active_count');
if any(isfinite(y))
    globalMask = globalMask | y > 0;
end
if any(globalMask)
    windows = maskToTimeSpans(t, globalMask);
end
end

function windows = attackWindowsFromStartEndColumns(tbl, attacker)
windows = zeros(0, 2);
windows = appendStartEndWindow(windows, ...
    firstFinite(col(tbl, 'v2v_attack_start_s')), ...
    firstFinite(col(tbl, 'v2v_attack_end_s')));

if isfinite(attacker)
    windows = appendStartEndWindow(windows, ...
        firstFinite(col(tbl, sprintf('inject_attack_start_%d', attacker))), ...
        firstFinite(col(tbl, sprintf('inject_attack_end_%d', attacker))));
end

windows = mergeTimeWindows(windows);
end

function windows = appendStartEndWindow(windows, startTime, endTime)
if isfinite(startTime) && isfinite(endTime) && endTime > startTime
    windows(end + 1, :) = [startTime endTime];
end
end

function windows = fallbackAttackWindow(t)
finiteTime = t(isfinite(t));
if isempty(finiteTime)
    windows = [10.0 15.0];
    return;
end

startTime = max(min(finiteTime), 10.0);
endTime = min(max(finiteTime), startTime + 5.0);
if endTime <= startTime
    endTime = startTime + 1e-3;
end
windows = [startTime endTime];
end

function [attackStart, attackEnd] = attackWindowBounds(windows)
if isempty(windows)
    attackStart = NaN;
    attackEnd = NaN;
else
    attackStart = min(windows(:, 1));
    attackEnd = max(windows(:, 2));
end
end

function plotWindow = resolvePlotWindow(t, attackWindows, requestedWindow)
if all(isfinite(requestedWindow)) && requestedWindow(2) > requestedWindow(1)
    plotWindow = requestedWindow;
    return;
end
[attackStart, attackEnd] = attackWindowBounds(attackWindows);
plotWindow = [attackStart - 5.0, attackEnd + 5.0];
finiteTime = t(isfinite(t));
if ~isempty(finiteTime)
    plotWindow(1) = max(plotWindow(1), min(finiteTime));
    plotWindow(2) = min(plotWindow(2), max(finiteTime));
end
end

function turnMask = resolveTurnMask(tbl, t, requestedWindows)
turnMask = false(size(t));

loggedTurn = col(tbl, 'is_turning');
if any(isfinite(loggedTurn))
    turnMask = turnMask | loggedTurn >= 0.5;
end

windows = normalizeWindowRows(requestedWindows);
for k = 1:size(windows, 1)
    turnMask = turnMask | (t >= windows(k, 1) & t <= windows(k, 2));
end

turnMask = normalizeMask(turnMask, t);
end

function windows = normalizeWindowRows(requestedWindows)
if isempty(requestedWindows)
    windows = zeros(0, 2);
    return;
end

values = double(requestedWindows);
if isempty(values) || mod(numel(values), 2) ~= 0
    windows = zeros(0, 2);
    return;
end

windows = reshape(values(:), 2, []).';
valid = all(isfinite(windows), 2) & windows(:, 2) > windows(:, 1);
windows = windows(valid, :);
end

function windows = mergeTimeWindows(windows)
if isempty(windows)
    windows = zeros(0, 2);
    return;
end

windows = double(windows);
valid = all(isfinite(windows), 2) & windows(:, 2) > windows(:, 1);
windows = sortrows(windows(valid, :), 1);
if isempty(windows)
    return;
end

merged = windows(1, :);
tol = 1e-6;
for k = 2:size(windows, 1)
    if windows(k, 1) <= merged(end, 2) + tol
        merged(end, 2) = max(merged(end, 2), windows(k, 2));
    else
        merged(end + 1, :) = windows(k, :); %#ok<AGROW>
    end
end
windows = merged;
end

function windows = clipWindowsToTime(windows, t)
windows = mergeTimeWindows(windows);
finiteTime = t(isfinite(t));
if isempty(windows) || isempty(finiteTime)
    return;
end

lo = min(finiteTime);
hi = max(finiteTime);
windows(:, 1) = max(windows(:, 1), lo);
windows(:, 2) = min(windows(:, 2), hi);
windows = mergeTimeWindows(windows);
end

function mask = timeInWindows(t, windows)
mask = false(size(t));
windows = mergeTimeWindows(windows);
for k = 1:size(windows, 1)
    mask = mask | (t >= windows(k, 1) & t <= windows(k, 2));
end
mask = normalizeMask(mask, t);
end

function duration = totalWindowDuration(windows)
windows = mergeTimeWindows(windows);
if isempty(windows)
    duration = NaN;
else
    duration = sum(windows(:, 2) - windows(:, 1));
end
end

function mask = normalizeMask(mask, t)
mask = logical(mask(:));
if numel(mask) ~= numel(t)
    mask = false(numel(t), 1);
end
mask = mask & isfinite(t(:));
end

function spec = chooseSourceWeightColumn(tbl, attacker, host)
spec = struct('column', "", 'label', "W0 weight");

candidates = strings(0, 1);
labels = strings(0, 1);
candidates(end + 1, 1) = sprintf('w0_final_%d', attacker);
labels(end + 1, 1) = sprintf('W0 final V%d', attacker);
candidates(end + 1, 1) = "w0";
labels(end + 1, 1) = "W0 host";
candidates(end + 1, 1) = sprintf('w_neighbor_%d', attacker);
labels(end + 1, 1) = sprintf('neighbor weight V%d fallback', attacker);
if isfinite(host)
    candidates(end + 1, 1) = sprintf('w_neighbor_from_v%d_to_%d', attacker, host);
    labels(end + 1, 1) = sprintf('neighbor weight V%d to V%d fallback', attacker, host);
end

for k = 1:numel(candidates)
    y = col(tbl, candidates(k));
    if any(isfinite(y))
        spec.column = candidates(k);
        spec.label = labels(k);
        return;
    end
end

names = string(tbl.Properties.VariableNames);
for k = 1:numel(candidates)
    if any(names == candidates(k))
        spec.column = candidates(k);
        spec.label = labels(k) + " (not finite in this log)";
        warning('Source-weight column %s exists for V%d but has no finite values.', ...
            candidates(k), attacker);
        return;
    end
end

fallbackTrust = sprintf('trust_%d', attacker);
if any(names == fallbackTrust)
    spec.column = fallbackTrust;
    spec.label = sprintf('trust V%d fallback', attacker);
    warning('No W0/weight column found for attacked vehicle V%d; using %s for plotting continuity.', ...
        attacker, fallbackTrust);
    return;
end

error('Could not find a W0, weight, or trust column for attacked vehicle V%d.', attacker);
end

function summary = buildControlSummary(tbl, t, filepath, host, attacker, peer, ...
    weightSpec, gapSpec, attackWindows, turnMask, trustThreshold, args)

turnMask = normalizeMask(turnMask, t);
finiteTimeMask = isfinite(t);
[attackStart, attackEnd] = attackWindowBounds(attackWindows);
preMask = t >= attackStart - 5.0 & t < attackStart;
attackMask = timeInWindows(t, attackWindows);
postMask = t > attackEnd & t <= attackEnd + 5.0;
attackTurnMask = attackMask & turnMask;
attackStraightMask = attackMask & ~turnMask;
attackActiveDuration = totalWindowDuration(attackWindows);
attackSpanDuration = attackEnd - attackStart;
if isfinite(attackSpanDuration) && attackSpanDuration > 0 && isfinite(attackActiveDuration)
    attackDutyPercent = 100.0 * attackActiveDuration / attackSpanDuration;
else
    attackDutyPercent = NaN;
end

trust = col(tbl, sprintf('trust_%d', attacker));
weight = col(tbl, weightSpec.column);
alpha = col(tbl, 'ctrl_alpha');
sensorGap = col(tbl, 'ctrl_sensor_gap');
projectedGap = col(tbl, 'ctrl_along_track_gap');
leaderDist = col(tbl, 'ctrl_distance_to_leader');
controlGap = firstFiniteSeries(projectedGap, leaderDist, sensorGap);
hostPairGap = hostDirectGapSeries(gapSpec, host, height(tbl));
fleetDirectGap = minGapSeries(gapSpec, height(tbl), "direct");
hostSpeed = hostVelocity(tbl, host);
caccDesiredGap = args.CaccS0 + args.CaccHeadway * max(hostSpeed, 0);
accDesiredGap = args.AccDesiredDistance + args.AccHeadway * max(hostSpeed, 0);
caccGapError = controlGap - caccDesiredGap;
accGapError = sensorGap - accDesiredGap;
hostGapError = hostPairGap - caccDesiredGap;
hostGapMargin = hostPairGap - args.SpacingThreshold;
hostGapUniverse = attackMask & isfinite(hostPairGap);
hostTrackingUniverse = hostGapUniverse & isfinite(caccDesiredGap);

err = [];
ids = [attacker peer];
ids = ids(isfinite(ids));
for k = 1:numel(ids)
    e = positionErrorWithReference(tbl, ids(k), 'est');
    err = [err; e(attackMask)]; %#ok<AGROW>
end

detectionDelay = firstCrossingDelay(t, trust, attackStart, trustThreshold, false, attackMask);
recoveryDelay = firstCrossingDelay(t, trust, attackEnd, trustThreshold, true);

sourcePre = finiteMean(weight(preMask));
sourceAttack = finiteMean(weight(attackMask));
alphaPre = finiteMean(alpha(preMask));
alphaAttack = finiteMean(alpha(attackMask));
uFinal = col(tbl, 'ctrl_u_final');
uCacc = col(tbl, 'ctrl_u_cacc');
uSensor = col(tbl, 'ctrl_u_sensor');
fusionDelta = abs(uFinal - uCacc);

minHostPairGapAttack = finiteMin(hostPairGap(attackMask));
meanHostPairGapAttack = finiteMean(hostPairGap(attackMask));
minFleetDirectGapAttack = finiteMin(fleetDirectGap(attackMask));
meanFleetDirectGapAttack = finiteMean(fleetDirectGap(attackMask));
minHostGapMarginAttack = finiteMin(hostGapMargin(attackMask));
meanHostGapErrorAttack = finiteMean(hostGapError(attackMask));
rmseHostGapErrorAttack = rmse(hostGapError(attackMask));
meanAbsHostGapErrorAttack = finiteMean(abs(hostGapError(attackMask)));
stdHostPairGapAttack = finiteStd(hostPairGap(attackMask));
hostGapWithinDesiredAttack = percentTrue( ...
    abs(hostGapError) <= args.SpacingTolerance, hostTrackingUniverse);
hostGapBelowS0Attack = percentTrue( ...
    hostPairGap < args.SpacingThreshold, hostGapUniverse);
hostGapBelowDesiredAttack = percentTrue( ...
    hostGapError < -args.SpacingTolerance, hostTrackingUniverse);
hostGapAboveDesiredAttack = percentTrue( ...
    hostGapError > args.SpacingTolerance, hostTrackingUniverse);
alphaDrop = percentDrop(alphaPre, alphaAttack);
[safetyVerdict, trackingVerdict, stabilityVerdict, fusionVerdict, overallVerdict] = ...
    attackVerdicts(minHostPairGapAttack, rmseHostGapErrorAttack, ...
    stdHostPairGapAttack, hostGapWithinDesiredAttack, alphaPre, ...
    alphaAttack, alphaDrop, args);

summary = table( ...
    string(filepath), host, attacker, peer, attackStart, attackEnd, ...
    size(mergeTimeWindows(attackWindows), 1), attackActiveDuration, ...
    attackSpanDuration, attackDutyPercent, ...
    minHostPairGapAttack, meanHostPairGapAttack, ...
    minFleetDirectGapAttack, meanFleetDirectGapAttack, ...
    minHostGapMarginAttack, meanHostGapErrorAttack, rmseHostGapErrorAttack, ...
    meanAbsHostGapErrorAttack, stdHostPairGapAttack, ...
    hostGapWithinDesiredAttack, hostGapBelowS0Attack, ...
    hostGapBelowDesiredAttack, hostGapAboveDesiredAttack, ...
    safetyVerdict, trackingVerdict, stabilityVerdict, fusionVerdict, overallVerdict, ...
    finiteMean(trust(preMask)), finiteMean(trust(attackMask)), finiteMin(trust(attackMask)), ...
    detectionDelay, recoveryDelay, sourcePre, sourceAttack, percentDrop(sourcePre, sourceAttack), ...
    alphaPre, alphaAttack, finiteMean(alpha(postMask)), alphaDrop, ...
    finiteMin(sensorGap(attackMask)), finiteMin(controlGap(attackMask)), ...
    finiteMin(projectedGap(attackMask)), finiteMean(sensorGap(attackMask)), ...
    finiteMean(controlGap(attackMask)), finiteMean(caccDesiredGap(attackMask)), ...
    finiteMean(accDesiredGap(attackMask)), finiteMean(caccGapError(attackMask)), ...
    finiteMean(accGapError(attackMask)), rmse(caccGapError(attackMask)), ...
    rmse(accGapError(attackMask)), ...
    maxOrNan(err), rmse(err), finiteMean(uFinal(attackMask)), ...
    finiteMean(uCacc(attackMask)), finiteMean(uSensor(attackMask)), ...
    finiteMean(fusionDelta(attackMask)), ...
    percentTrue(isfinite(uSensor), attackMask), ...
    percentTrue(isfinite(alpha) & alpha <= 0.5, attackMask & isfinite(alpha)), ...
    percentTrue(turnMask, finiteTimeMask), percentTrue(turnMask, attackMask), ...
    finiteMean(controlGap(preMask)), finiteMean(controlGap(attackStraightMask)), ...
    finiteMean(controlGap(attackTurnMask)), finiteMean(controlGap(postMask)), ...
    finiteMean(sensorGap(preMask)), finiteMean(sensorGap(attackStraightMask)), ...
    finiteMean(sensorGap(attackTurnMask)), finiteMean(sensorGap(postMask)), ...
    finiteMean(caccGapError(attackStraightMask)), finiteMean(caccGapError(attackTurnMask)), ...
    'VariableNames', {'file', 'host', 'attacker', 'peer', 'attackStart_s', 'attackEnd_s', ...
    'attackIntervalCount', 'attackActiveDuration_s', 'attackSpanDuration_s', ...
    'attackDuty_percent', ...
    'minHostPairGapAttack_m', 'meanHostPairGapAttack_m', ...
    'minFleetDirectGapAttack_m', 'meanFleetDirectGapAttack_m', ...
    'minHostGapMarginAttack_m', 'meanHostGapErrorAttack_m', ...
    'rmseHostGapErrorAttack_m', 'meanAbsHostGapErrorAttack_m', ...
    'stdHostPairGapAttack_m', 'hostGapWithinDesiredAttack_percent', ...
    'hostGapBelowS0Attack_percent', 'hostGapBelowDesiredAttack_percent', ...
    'hostGapAboveDesiredAttack_percent', 'safetyVerdict', ...
    'spacingTrackingVerdict', 'gapStabilityVerdict', 'fusionVerdict', ...
    'overallAttackVerdict', ...
    'trustPreMean', 'trustAttackMean', 'trustAttackMin', 'trustDetectionDelay_s', ...
    'trustRecoveryDelay_s', 'sourceWeightPreMean', 'sourceWeightAttackMean', ...
    'sourceWeightSuppression_percent', 'alphaPreMean', 'alphaAttackMean', ...
    'alphaPostMean', 'alphaDrop_percent', 'minSensorGapAttack_m', ...
    'minControlGapAttack_m', 'minProjectedGapAttack_m', ...
    'meanSensorGapAttack_m', 'meanControlGapAttack_m', ...
    'meanCaccDesiredGapAttack_m', 'meanAccDesiredGapAttack_m', ...
    'meanCaccGapErrorAttack_m', 'meanAccGapErrorAttack_m', ...
    'rmseCaccGapErrorAttack_m', 'rmseAccGapErrorAttack_m', ...
    'maxEstPosErrAttack_m', 'rmseEstPosErrAttack_m', ...
    'meanUFinalAttack', 'meanUCaccAttack', 'meanUSensorAttack', ...
    'meanAbsFusionDeltaAttack', 'sensorCommandAvailableAttack_percent', ...
    'sensorDominantAlphaAttack_percent', 'turnCoverage_percent', ...
    'turnCoverageAttack_percent', 'meanControlGapPre_m', ...
    'meanControlGapAttackStraight_m', 'meanControlGapAttackTurn_m', ...
    'meanControlGapPost_m', 'meanSensorGapPre_m', ...
    'meanSensorGapAttackStraight_m', 'meanSensorGapAttackTurn_m', ...
    'meanSensorGapPost_m', 'meanCaccGapErrorAttackStraight_m', ...
    'meanCaccGapErrorAttackTurn_m'});
end

function printSummary(summary, weightSpec)
fprintf('\nSection F control-response summary\n');
fprintf('File: %s\n', summary.file);
fprintf('Host V%d, attacker V%d, peer V%d, attack span [%.3f, %.3f] s\n', ...
    summary.host, summary.attacker, summary.peer, summary.attackStart_s, summary.attackEnd_s);
fprintf('Attack intervals: %d, active duration %.3f s over %.3f s span (%.1f%% duty)\n', ...
    summary.attackIntervalCount, summary.attackActiveDuration_s, ...
    summary.attackSpanDuration_s, summary.attackDuty_percent);
fprintf('W0/weight column: %s\n', weightSpec.column);
fprintf('Trust: %.3f pre -> %.3f attack, detection delay %.3f s, recovery delay %.3f s\n', ...
    summary.trustPreMean, summary.trustAttackMean, summary.trustDetectionDelay_s, ...
    summary.trustRecoveryDelay_s);
fprintf('W0/weight: %.4f pre -> %.4f attack (%.1f%% suppression)\n', ...
    summary.sourceWeightPreMean, summary.sourceWeightAttackMean, ...
    summary.sourceWeightSuppression_percent);
fprintf('Controller alpha: %.3f pre -> %.3f attack (%.1f%% drop)\n', ...
    summary.alphaPreMean, summary.alphaAttackMean, summary.alphaDrop_percent);
fprintf('Fusion branch: sensor command available %.1f%% of attack; alpha <= 0.5 for %.1f%%; mean |u_final-u_cacc| %.4f\n', ...
    summary.sensorCommandAvailableAttack_percent, ...
    summary.sensorDominantAlphaAttack_percent, summary.meanAbsFusionDeltaAttack);
fprintf('Turn coverage: %.1f%% overall, %.1f%% during attack\n', ...
    summary.turnCoverage_percent, summary.turnCoverageAttack_percent);
fprintf('Verdict: %s | safety %s, spacing %s, stability %s, fusion %s\n', ...
    summary.overallAttackVerdict, summary.safetyVerdict, ...
    summary.spacingTrackingVerdict, summary.gapStabilityVerdict, ...
    summary.fusionVerdict);
fprintf('Pairwise gaps: host mean %.3f m, min %.3f m; fleet direct min %.3f m during attack\n', ...
    summary.meanHostPairGapAttack_m, summary.minHostPairGapAttack_m, ...
    summary.minFleetDirectGapAttack_m);
fprintf('Expected spacing: error mean %.3f m, RMSE %.3f m; within tolerance %.1f%%, below s0 %.1f%%\n', ...
    summary.meanHostGapErrorAttack_m, summary.rmseHostGapErrorAttack_m, ...
    summary.hostGapWithinDesiredAttack_percent, summary.hostGapBelowS0Attack_percent);
fprintf('Attack CACC gap: control mean %.3f m; desired %.3f m (error %.3f m)\n', ...
    summary.meanControlGapAttack_m, summary.meanCaccDesiredGapAttack_m, ...
    summary.meanCaccGapErrorAttack_m);
fprintf('Attack ACC gap: sensor mean %.3f m; desired %.3f m (error %.3f m)\n', ...
    summary.meanSensorGapAttack_m, summary.meanAccDesiredGapAttack_m, ...
    summary.meanAccGapErrorAttack_m);
fprintf('Control gap by section: pre %.3f m, attack straight %.3f m, attack turn %.3f m, post %.3f m\n', ...
    summary.meanControlGapPre_m, summary.meanControlGapAttackStraight_m, ...
    summary.meanControlGapAttackTurn_m, summary.meanControlGapPost_m);
fprintf('Min sensor gap %.3f m, min control gap %.3f m; max attack-window position error %.3f m\n\n', ...
    summary.minSensorGapAttack_m, summary.minControlGapAttack_m, ...
    summary.maxEstPosErrAttack_m);
end

function delay = firstCrossingDelay(t, y, startTime, threshold, above, allowedMask)
if nargin < 6
    allowedMask = true(size(t));
end
allowedMask = normalizeMask(allowedMask, t);
delay = NaN;
mask = isfinite(t) & t >= startTime & isfinite(y) & allowedMask;
idxs = find(mask);
for k = 1:numel(idxs)
    idx = idxs(k);
    if above
        crossed = y(idx) >= threshold;
    else
        crossed = y(idx) < threshold;
    end
    if crossed
        delay = t(idx) - startTime;
        return;
    end
end
end

function drop = percentDrop(preValue, attackValue)
if isfinite(preValue) && abs(preValue) > 1e-12 && isfinite(attackValue)
    drop = 100.0 * (preValue - attackValue) / preValue;
else
    drop = NaN;
end
end

function [safetyVerdict, trackingVerdict, stabilityVerdict, fusionVerdict, overallVerdict] = ...
    attackVerdicts(minHostGap, rmseGapError, stdHostGap, withinDesiredPercent, ...
    alphaPre, alphaAttack, alphaDropPercent, args)

if isfinite(minHostGap)
    safetyPass = minHostGap >= args.SpacingThreshold;
    safetyVerdict = verdictLabel(safetyPass, "PASS", "FAIL");
else
    safetyPass = false;
    safetyVerdict = "UNKNOWN";
end

trackingKnown = isfinite(rmseGapError) || isfinite(withinDesiredPercent);
trackingPass = false;
if trackingKnown
    trackingPass = (isfinite(rmseGapError) && rmseGapError <= args.SpacingTolerance) ...
        || (isfinite(withinDesiredPercent) && withinDesiredPercent >= args.GoodCoveragePercent);
    trackingVerdict = verdictLabel(trackingPass, "PASS", "OFF_TARGET");
else
    trackingVerdict = "UNKNOWN";
end

if isfinite(stdHostGap)
    stabilityPass = stdHostGap <= args.GapStabilityStdMax;
    stabilityVerdict = verdictLabel(stabilityPass, "PASS", "UNSTABLE");
else
    stabilityPass = false;
    stabilityVerdict = "UNKNOWN";
end

fusionKnown = isfinite(alphaPre) && isfinite(alphaAttack) && isfinite(alphaDropPercent);
if fusionKnown
    fusionPass = alphaAttack <= args.FusionAlphaAttackMax ...
        && alphaDropPercent >= args.FusionAlphaDropMinPercent;
    fusionVerdict = verdictLabel(fusionPass, "PASS", "WEAK");
else
    fusionPass = false;
    fusionVerdict = "UNKNOWN";
end

if safetyVerdict == "UNKNOWN" || trackingVerdict == "UNKNOWN" ...
        || stabilityVerdict == "UNKNOWN" || fusionVerdict == "UNKNOWN"
    overallVerdict = "UNKNOWN_UNDER_ATTACK";
elseif ~safetyPass
    overallVerdict = "UNSAFE_UNDER_ATTACK";
elseif ~fusionPass
    overallVerdict = "FUSION_WEAK_UNDER_ATTACK";
elseif ~trackingPass
    overallVerdict = "SPACING_OFF_TARGET";
elseif ~stabilityPass
    overallVerdict = "GAP_UNSTABLE";
else
    overallVerdict = "PASS_UNDER_ATTACK";
end
end

function label = verdictLabel(condition, passLabel, failLabel)
if condition
    label = string(passLabel);
else
    label = string(failLabel);
end
end

function plotAttackTimeline(ax, tbl, t, plotWindow, style, showStatusRows, ...
    showEvents, showIntervalMarkers)
hold(ax, 'on');
intervals = attackTimelineIntervals(tbl, t);
if showEvents
    events = attackTimelineEvents(tbl, t);
else
    events = struct('event', {}, 'time_s', {});
end

lanes = strings(0, 1);
if showStatusRows
    lanes = ["module enabled"; "attack active"];
end
for k = 1:numel(intervals)
    for j = 1:numel(intervals(k).lanes)
        lane = intervals(k).lanes(j);
        if strlength(lane) > 0 && ~any(lanes == lane)
            lanes(end + 1) = lane; %#ok<AGROW>
        end
    end
end
if isempty(lanes)
    lanes = "attack";
end

barH = 0.68;
enabledSpans = zeros(0, 2);
activeSpans = zeros(0, 2);
if showStatusRows
    enabled = col(tbl, 'v2v_attack_enabled');
    enabledSpans = maskToTimeSpans(t, isfinite(enabled) & enabled >= 0.5);
    plotTimelineSpans(ax, enabledSpans, laneIndex(lanes, "module enabled"), ...
        barH, [0.20 0.70 0.20], 0.18);

    activeMask = false(size(t));
    active = col(tbl, 'v2v_attack_active');
    if any(isfinite(active))
        activeMask = activeMask | active >= 0.5;
    end
    activeCount = col(tbl, 'v2v_attack_active_count');
    if any(isfinite(activeCount))
        activeMask = activeMask | activeCount > 0;
    end
    activeSpans = maskToTimeSpans(t, activeMask);
    plotTimelineSpans(ax, activeSpans, laneIndex(lanes, "attack active"), ...
        barH, [0.85 0.20 0.20], 0.18);
end

colors = [
    style.blue
    style.orange
    style.green
    style.purple
    style.darkGreen
    [0.20 0.60 0.80]
    [0.80 0.35 0.65]];
colorKeys = strings(0, 1);
for k = 1:numel(intervals)
    colorKey = intervals(k).display_label;
    if strlength(colorKey) == 0
        colorKey = intervals(k).type;
    end
    idx = find(colorKeys == colorKey, 1, 'first');
    if isempty(idx)
        colorKeys(end + 1, 1) = colorKey; %#ok<AGROW>
        idx = numel(colorKeys);
    end
    color = colors(mod(idx - 1, size(colors, 1)) + 1, :);
    for j = 1:numel(intervals(k).lanes)
        y = laneIndex(lanes, intervals(k).lanes(j));
        if ~isfinite(y)
            continue;
        end
        patchTimelineSpan(ax, intervals(k).start_s, intervals(k).end_s, ...
            y - barH / 2, y + barH / 2, color, 0.45);
        if showIntervalMarkers
            plotTimelineBoundaryMarkers(ax, intervals(k).start_s, intervals(k).end_s, ...
                y - barH / 2, y + barH / 2, color);
        end
        text(ax, mean([intervals(k).start_s intervals(k).end_s]), y, ...
            char(intervals(k).display_label), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
            'FontName', style.fontName, 'FontSize', max(style.fontSize - 1, 6), ...
            'Interpreter', 'none', 'Clipping', 'on', 'HandleVisibility', 'off');
    end
end

for k = 1:numel(events)
    eventTime = events(k).time_s;
    if ~isfinite(eventTime)
        continue;
    end
    eventName = lower(events(k).event);
    color = [0.20 0.70 0.20];
    if eventName ~= "enable"
        color = [0.85 0.10 0.10];
    end
    xline(ax, eventTime, '--', 'Color', color, 'LineWidth', 0.9, ...
        'HandleVisibility', 'off');
    text(ax, eventTime, numel(lanes) + 0.45, char(eventName), ...
        'Rotation', 90, 'Color', color, 'FontName', style.fontName, ...
        'FontSize', max(style.fontSize - 1, 6), ...
        'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle', ...
        'Interpreter', 'none', 'Clipping', 'on', 'HandleVisibility', 'off');
end

if isempty(intervals) && isempty(enabledSpans) && isempty(activeSpans)
    text(ax, mean(plotWindow), 1.5, 'No V2V attack timeline data', ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
        'FontName', style.fontName, 'FontSize', style.fontSize, ...
        'HandleVisibility', 'off');
end

set(ax, 'YTick', 1:numel(lanes), 'YTickLabel', cellstr(lanes));
ylim(ax, [0.35, numel(lanes) + 0.65]);
styleAxes(ax, '(d) V2V attack timeline', 'time [s]', '', style);
if all(isfinite(plotWindow)) && plotWindow(2) > plotWindow(1)
    xlim(ax, plotWindow);
else
    finiteT = t(isfinite(t));
    if ~isempty(finiteT)
        xlim(ax, [min(finiteT) max(finiteT)]);
    end
end
end

function intervals = attackTimelineIntervals(tbl, t)
intervals = struct('start_s', {}, 'end_s', {}, 'lanes', {}, ...
    'display_label', {}, 'type', {});
finiteT = t(isfinite(t));
if isempty(finiteT)
    tMin = 0.0;
    tMax = 1.0;
else
    tMin = min(finiteT);
    tMax = max(finiteT);
end

data = jsonArrayOrEmpty(lastNonemptyText(tbl, 'v2v_attack_intervals'));
if ~isempty(data)
    intervals(numel(data)) = struct( ...
        'start_s', NaN, ...
        'end_s', NaN, ...
        'lanes', "", ...
        'display_label', "", ...
        'type', "");
end
count = 0;
for k = 1:numel(data)
    item = data(k);
    startTime = numericStructField(item, 'start_s');
    endTime = numericStructField(item, 'end_s');
    if ~isfinite(startTime) || ~isfinite(endTime)
        continue;
    end
    startTime = max(startTime, tMin);
    endTime = min(endTime, tMax);
    if endTime <= startTime
        endTime = startTime + 1e-3;
    end
    count = count + 1;
    intervals(count) = struct( ...
        'start_s', startTime, ...
        'end_s', endTime, ...
        'lanes', attackTimelineLanes(item), ...
        'display_label', attackTimelineLabel(item), ...
        'type', attackTimelineType(item));
end
if count > 0
    intervals = intervals(1:count);
else
    intervals = struct('start_s', {}, 'end_s', {}, 'lanes', {}, ...
        'display_label', {}, 'type', {});
end

if ~isempty(intervals)
    return;
end

startTime = firstFinite(col(tbl, 'v2v_attack_start_s'));
endTime = firstFinite(col(tbl, 'v2v_attack_end_s'));
if isfinite(startTime) && isfinite(endTime) && endTime > startTime
    intervals(end + 1) = struct( ...
        'start_s', max(startTime, tMin), ...
        'end_s', min(endTime, tMax), ...
        'lanes', "fleet all", ...
        'display_label', "attack", ...
        'type', "attack");
end
end

function events = attackTimelineEvents(tbl, t)
events = struct('event', {}, 'time_s', {});
data = jsonArrayOrEmpty(lastNonemptyText(tbl, 'v2v_attack_events'));
for k = 1:numel(data)
    eventTime = numericStructField(data(k), 'time_s');
    eventName = stringStructField(data(k), 'event');
    if isfinite(eventTime) && strlength(eventName) > 0
        events(end + 1) = struct('event', eventName, 'time_s', eventTime); %#ok<AGROW>
    end
end
if ~isempty(events)
    return;
end

enabled = col(tbl, 'v2v_attack_enabled');
if ~any(isfinite(enabled))
    return;
end

mask = isfinite(enabled) & enabled >= 0.5;
prev = false;
for k = 1:numel(mask)
    if ~isfinite(t(k))
        continue;
    end
    if mask(k) && ~prev
        events(end + 1) = struct('event', "enable", 'time_s', t(k)); %#ok<AGROW>
    elseif prev && ~mask(k)
        events(end + 1) = struct('event', "disable", 'time_s', t(k)); %#ok<AGROW>
    end
    prev = mask(k);
end
end

function lanes = attackTimelineLanes(item)
dataType = lower(stringStructField(item, 'data_type'));
attackerId = numericStructField(item, 'attacker_id');
if isfinite(attackerId)
    localLane = "local V" + string(round(attackerId));
else
    localLane = "local";
end

lanes = strings(0, 1);
if contains(dataType, "fleet") || contains(dataType, "global") || contains(dataType, "both")
    lanes(end + 1, 1) = "fleet all";
end
if contains(dataType, "local") || contains(dataType, "both")
    lanes(end + 1, 1) = localLane;
end
if isempty(lanes)
    lanes = "attack";
end
end

function label = attackTimelineLabel(item)
label = attackCaseLabel(item);
if strlength(label) > 0
    return;
end

label = stringStructField(item, 'name');
if strlength(label) == 0
    attackType = stringStructField(item, 'type');
    modification = stringStructField(item, 'modification');
    fields = stringStructField(item, 'target_fields');
    label = strjoin([attackType modification fields], " ");
end
label = erase(label, "Mix_test_");
label = erase(label, "Case");
label = strrep(label, "_", " ");
label = strtrim(label);
if strlength(label) > 24
    label = extractBefore(label, 22) + "...";
end
if strlength(label) == 0
    label = "attack";
end
end

function label = attackCaseLabel(item)
label = "";
caseFields = ["case_id", "case_number", "case"];
for k = 1:numel(caseFields)
    caseNumber = numericStructField(item, caseFields(k));
    if isfinite(caseNumber)
        label = "Case " + string(round(caseNumber));
        return;
    end
end

texts = [
    stringStructField(item, 'name')
    stringStructField(item, 'description')];
for k = 1:numel(texts)
    txt = char(texts(k));
    if isempty(txt)
        continue;
    end
    token = regexp(txt, '(?i)case[_\s-]*(\d+)', 'tokens', 'once');
    if ~isempty(token)
        label = "Case " + string(token{1});
        return;
    end
end
end

function typeName = attackTimelineType(item)
typeName = stringStructField(item, 'type');
if strlength(typeName) == 0
    typeName = "attack";
end
end

function plotTimelineSpans(ax, spans, y, barH, color, alpha)
if ~isfinite(y)
    return;
end
for k = 1:size(spans, 1)
    patchTimelineSpan(ax, spans(k, 1), spans(k, 2), ...
        y - barH / 2, y + barH / 2, color, alpha);
end
end

function plotTimelineBoundaryMarkers(ax, x1, x2, y1, y2, color)
markerColor = max(0.0, color * 0.65);
line(ax, [x1 x1], [y1 y2], 'Color', markerColor, ...
    'LineStyle', ':', 'LineWidth', 1.2, 'HandleVisibility', 'off');
line(ax, [x2 x2], [y1 y2], 'Color', markerColor, ...
    'LineStyle', ':', 'LineWidth', 1.2, 'HandleVisibility', 'off');
end

function patchTimelineSpan(ax, x1, x2, y1, y2, color, alpha)
if ~isfinite(x1) || ~isfinite(x2) || x2 <= x1
    return;
end
patch(ax, [x1 x2 x2 x1], [y1 y1 y2 y2], color, ...
    'FaceAlpha', alpha, 'EdgeColor', color, 'EdgeAlpha', min(alpha + 0.20, 1.0), ...
    'LineWidth', 0.5, 'HandleVisibility', 'off');
end

function idx = laneIndex(lanes, lane)
found = find(lanes == string(lane), 1, 'first');
if isempty(found)
    idx = NaN;
else
    idx = found;
end
end

function markTurnSections(ax, t, turnMask, style, enabled)
if ~enabled
    return;
end

turnMask = normalizeMask(turnMask, t);
spans = maskToTimeSpans(t, turnMask);
if isempty(spans)
    return;
end

yl = ylim(ax);
handles = gobjects(size(spans, 1), 1);
for k = 1:size(spans, 1)
    handles(k) = patch(ax, ...
        [spans(k, 1) spans(k, 2) spans(k, 2) spans(k, 1)], ...
        [yl(1) yl(1) yl(2) yl(2)], ...
        style.turnShade, 'FaceAlpha', 0.22, 'EdgeColor', style.turnEdge, ...
        'EdgeAlpha', 0.25, 'LineWidth', 0.4, 'HandleVisibility', 'off');
end

labelSpan = spans(1, :);
text(ax, mean(labelSpan), yl(2) - 0.04 * diff(yl), 'turn', ...
    'Color', style.turnEdge, 'FontSize', style.fontSize - 1, ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
    'Clipping', 'on', 'HandleVisibility', 'off');

try
    uistack(handles, 'bottom');
catch
end
ylim(ax, yl);
end

function spans = maskToTimeSpans(t, mask)
t = t(:);
mask = normalizeMask(mask, t);
spans = zeros(0, 2);
if isempty(t) || ~any(mask)
    return;
end

finiteTime = t(isfinite(t));
dt = 1e-3;
if numel(finiteTime) > 1
    diffs = diff(unique(finiteTime));
    diffs = diffs(diffs > 0);
    if ~isempty(diffs)
        dt = median(diffs);
    end
end

inSpan = false;
startTime = NaN;
lastTime = NaN;
for k = 1:numel(t)
    isOn = mask(k) && isfinite(t(k));
    if isOn
        if ~inSpan
            startTime = t(k);
            inSpan = true;
        end
        lastTime = t(k);
    end

    if inSpan && (~isOn || k == numel(t))
        endTime = lastTime;
        if ~isfinite(endTime) || endTime <= startTime
            endTime = startTime + dt;
        end
        spans(end + 1, :) = [startTime endTime]; %#ok<AGROW>
        inSpan = false;
        startTime = NaN;
        lastTime = NaN;
    end
end
end

function gapSpec = buildFleetGapSeries(tbl, host)
gapSpec = struct('follower', {}, 'leader', {}, 'role', {}, ...
    'isHostDirect', {}, 'gap', {}, 'label', {});

ids = extractVehicleIdsWithPosition(tbl);
if isempty(ids)
    return;
end

ids = sort(ids(:).');
for k = 2:numel(ids)
    follower = ids(k);
    leader = ids(k - 1);
    isHostDirect = isfinite(host) && follower == round(host);
    gapSpec = appendPairGapSpec(gapSpec, tbl, follower, leader, ...
        "direct", isHostDirect);
end

if numel(ids) >= 3
    follower = ids(end);
    leader = ids(1);
    if ~pairExists(gapSpec, follower, leader)
        gapSpec = appendPairGapSpec(gapSpec, tbl, follower, leader, ...
            "end_to_end", false);
    end
end

ctrlLeader = round(firstFinite(col(tbl, 'ctrl_leader_id')));
if isfinite(host) && isfinite(ctrlLeader) && ctrlLeader ~= round(host) ...
        && ~pairExists(gapSpec, round(host), ctrlLeader)
    gapSpec = appendPairGapSpec(gapSpec, tbl, round(host), ctrlLeader, ...
        "host_direct", true);
end
end

function gapSpec = orderGapSpecForHost(gapSpec, host)
if isempty(gapSpec) || ~isfinite(host)
    return;
end

host = round(host);
isHostGap = false(size(gapSpec));
for k = 1:numel(gapSpec)
    isHostGap(k) = gapSpec(k).isHostDirect && gapSpec(k).follower == host;
end
if ~any(isHostGap)
    for k = 1:numel(gapSpec)
        isHostGap(k) = gapSpec(k).follower == host;
    end
end

gapSpec = [gapSpec(isHostGap), gapSpec(~isHostGap)];
end

function leader = hostLeaderFromGapSpec(gapSpec, host)
leader = NaN;
if isempty(gapSpec) || ~isfinite(host)
    return;
end

host = round(host);
for k = 1:numel(gapSpec)
    if gapSpec(k).isHostDirect && gapSpec(k).follower == host
        leader = gapSpec(k).leader;
        return;
    end
end
for k = 1:numel(gapSpec)
    if gapSpec(k).follower == host
        leader = gapSpec(k).leader;
        return;
    end
end
end

function label = hostPairLabelFromGapSpec(gapSpec, host)
leader = hostLeaderFromGapSpec(gapSpec, host);
if isfinite(host) && isfinite(leader)
    label = sprintf('V%d-V%d gap', round(host), round(leader));
else
    label = 'host gap';
end
end

function gapSpec = buildBatchFleetGapSeries(filepaths, targetT, host)
gapSpec = struct('follower', {}, 'leader', {}, 'role', {}, ...
    'isHostDirect', {}, 'gap', {}, 'label', {});

filepaths = normalizeStringArray(filepaths);
logs = repmat(struct('filepath', '', 'tbl', table(), 'time', [], 'host', NaN), ...
    numel(filepaths), 1);
logCount = 0;
ids = [];
for k = 1:numel(filepaths)
    filepath = char(filepaths(k));
    try
        tbl = readTrustTable(filepath);
    catch err
        warning('Could not read fleet gap file %s: %s', filepath, err.message);
        continue;
    end
    if ~ismember('time', tbl.Properties.VariableNames)
        continue;
    end
    logCount = logCount + 1;
    logs(logCount) = struct( ...
        'filepath', filepath, ...
        'tbl', tbl, ...
        'time', col(tbl, 'time'), ...
        'host', resolveHost(filepath, tbl, NaN));
    ids = [ids; extractVehicleIds(tbl)]; %#ok<AGROW>
end
logs = logs(1:logCount);

ids = sort(unique(ids(isfinite(ids))).');
if isempty(logs) || numel(ids) < 2
    return;
end

trajectories = repmat(struct('vehicle', NaN, 'x', [], 'y', [], 'source', ""), ...
    numel(ids), 1);
trajectoryCount = 0;
for k = 1:numel(ids)
    vehicleId = ids(k);
    [x, y, sourceName] = batchVehicleTrajectory(logs, vehicleId, targetT);
    if any(isfinite(x) & isfinite(y))
        trajectoryCount = trajectoryCount + 1;
        trajectories(trajectoryCount) = struct( ...
            'vehicle', vehicleId, ...
            'x', x, ...
            'y', y, ...
            'source', sourceName);
    end
end
trajectories = trajectories(1:trajectoryCount);

vehicleIds = [trajectories.vehicle];
vehicleIds = sort(vehicleIds(:).');
for k = 2:numel(vehicleIds)
    follower = vehicleIds(k);
    leader = vehicleIds(k - 1);
    isHostDirect = isfinite(host) && follower == round(host);
    gapSpec = appendTrajectoryGapSpec(gapSpec, trajectories, follower, leader, ...
        "direct", isHostDirect);
end

if numel(vehicleIds) >= 3
    follower = vehicleIds(end);
    leader = vehicleIds(1);
    if ~pairExists(gapSpec, follower, leader)
        gapSpec = appendTrajectoryGapSpec(gapSpec, trajectories, follower, leader, ...
            "end_to_end", false);
    end
end
end

function [x, y, sourceName] = batchVehicleTrajectory(logs, vehicleId, targetT)
[x, y] = combineBatchTrajectory(logs, vehicleId, targetT, 'ref');
if any(isfinite(x) & isfinite(y))
    sourceName = "reference";
    return;
end

ownMask = false(numel(logs), 1);
for k = 1:numel(logs)
    ownMask(k) = isfinite(logs(k).host) && round(logs(k).host) == vehicleId;
end
[x, y] = combineBatchTrajectory(logs(ownMask), vehicleId, targetT, 'est');
if any(isfinite(x) & isfinite(y))
    sourceName = "self_state";
    return;
end

[x, y] = combineBatchTrajectory(logs, vehicleId, targetT, 'est');
sourceName = "estimate";
end

function [x, y] = combineBatchTrajectory(logs, vehicleId, targetT, prefix)
targetT = targetT(:);
xMat = nan(numel(targetT), 0);
yMat = nan(numel(targetT), 0);
for k = 1:numel(logs)
    rawX = col(logs(k).tbl, sprintf('%s_x_%d', prefix, vehicleId));
    rawY = col(logs(k).tbl, sprintf('%s_y_%d', prefix, vehicleId));
    sourceT = logs(k).time(:);
    mask = isfinite(sourceT) & isfinite(rawX) & isfinite(rawY);
    if sum(mask) < 2
        continue;
    end

    sourceT = sourceT(mask);
    rawX = rawX(mask);
    rawY = rawY(mask);
    [sourceT, uniqueIdx] = unique(sourceT, 'stable');
    rawX = rawX(uniqueIdx);
    rawY = rawY(uniqueIdx);
    if numel(sourceT) < 2
        continue;
    end

    interpX = interp1(sourceT, rawX, targetT, 'linear', NaN);
    interpY = interp1(sourceT, rawY, targetT, 'linear', NaN);
    if any(isfinite(interpX) & isfinite(interpY))
        xMat(:, end + 1) = interpX; %#ok<AGROW>
        yMat(:, end + 1) = interpY; %#ok<AGROW>
    end
end

x = rowMedianOmitNaN(xMat, numel(targetT));
y = rowMedianOmitNaN(yMat, numel(targetT));
end

function values = rowMedianOmitNaN(matrixValues, rowCount)
values = nan(rowCount, 1);
if isempty(matrixValues)
    return;
end
for k = 1:rowCount
    row = matrixValues(k, :);
    row = row(isfinite(row));
    if ~isempty(row)
        values(k) = median(row);
    end
end
end

function gapSpec = appendTrajectoryGapSpec(gapSpec, trajectories, follower, leader, role, isHostDirect)
followerIdx = trajectoryIndex(trajectories, follower);
leaderIdx = trajectoryIndex(trajectories, leader);
if ~isfinite(followerIdx) || ~isfinite(leaderIdx)
    return;
end

followerX = trajectories(followerIdx).x(:);
followerY = trajectories(followerIdx).y(:);
leaderX = trajectories(leaderIdx).x(:);
leaderY = trajectories(leaderIdx).y(:);
n = min([numel(followerX), numel(followerY), numel(leaderX), numel(leaderY)]);
gap = nan(n, 1);
mask = isfinite(followerX(1:n)) & isfinite(followerY(1:n)) ...
    & isfinite(leaderX(1:n)) & isfinite(leaderY(1:n));
gap(mask) = hypot(leaderX(mask) - followerX(mask), leaderY(mask) - followerY(mask));
if ~any(isfinite(gap))
    return;
end

sourceName = trajectoryPairLabel(trajectories(followerIdx).source, ...
    trajectories(leaderIdx).source);
label = spacingLegendLabel(sourceName, follower, leader, role, isHostDirect);
gapSpec(end + 1) = struct( ... %#ok<AGROW>
    'follower', follower, ...
    'leader', leader, ...
    'role', string(role), ...
    'isHostDirect', logical(isHostDirect), ...
    'gap', gap, ...
    'label', label);
end

function idx = trajectoryIndex(trajectories, vehicleId)
idx = NaN;
for k = 1:numel(trajectories)
    if trajectories(k).vehicle == vehicleId
        idx = k;
        return;
    end
end
end

function label = trajectoryPairLabel(sourceA, sourceB)
sourceA = string(sourceA);
sourceB = string(sourceB);
if sourceA == "reference" && sourceB == "reference"
    label = "batch true gap";
elseif (sourceA == "reference" || sourceA == "self_state") ...
        && (sourceB == "reference" || sourceB == "self_state")
    label = "batch logged gap";
else
    label = "batch estimated gap";
end
end

function ids = extractVehicleIdsWithPosition(tbl)
ids = extractVehicleIds(tbl);
keep = false(size(ids));
for k = 1:numel(ids)
    keep(k) = hasVehiclePosition(tbl, ids(k), 'ref') ...
        || hasVehiclePosition(tbl, ids(k), 'est');
end
ids = ids(keep);
end

function ok = hasVehiclePosition(tbl, vehicleId, prefix)
x = col(tbl, sprintf('%s_x_%d', prefix, vehicleId));
y = col(tbl, sprintf('%s_y_%d', prefix, vehicleId));
ok = any(isfinite(x) & isfinite(y));
end

function gapSpec = appendPairGapSpec(gapSpec, tbl, follower, leader, role, isHostDirect)
[gap, sourceName] = preferredPairGap(tbl, follower, leader);
if ~any(isfinite(gap))
    return;
end

label = spacingLegendLabel(sourceName, follower, leader, role, isHostDirect);
gapSpec(end + 1) = struct( ... %#ok<AGROW>
    'follower', follower, ...
    'leader', leader, ...
    'role', string(role), ...
    'isHostDirect', logical(isHostDirect), ...
    'gap', gap, ...
    'label', label);
end

function label = spacingLegendLabel(~, follower, leader, role, isHostDirect)
if isHostDirect
    label = sprintf('host gap V%d-V%d', follower, leader);
elseif string(role) == "end_to_end"
    label = sprintf('fleet span V%d-V%d', follower, leader);
else
    label = sprintf('platoon gap V%d-V%d', follower, leader);
end
end

function exists = pairExists(gapSpec, follower, leader)
exists = false;
for k = 1:numel(gapSpec)
    if gapSpec(k).follower == follower && gapSpec(k).leader == leader
        exists = true;
        return;
    end
end
end

function [gap, sourceName] = preferredPairGap(tbl, follower, leader)
trueGap = vehiclePairGap(tbl, follower, leader, 'ref');
if any(isfinite(trueGap))
    gap = trueGap;
    sourceName = 'true gap';
    return;
end

gap = vehiclePairGap(tbl, follower, leader, 'est');
sourceName = 'estimated gap';
end

function gap = vehiclePairGap(tbl, follower, leader, prefix)
followerX = col(tbl, sprintf('%s_x_%d', prefix, follower));
followerY = col(tbl, sprintf('%s_y_%d', prefix, follower));
leaderX = col(tbl, sprintf('%s_x_%d', prefix, leader));
leaderY = col(tbl, sprintf('%s_y_%d', prefix, leader));
gap = nan(height(tbl), 1);
mask = isfinite(followerX) & isfinite(followerY) & isfinite(leaderX) & isfinite(leaderY);
gap(mask) = hypot(leaderX(mask) - followerX(mask), leaderY(mask) - followerY(mask));
end

function plotFleetGaps(ax, t, gapSpec, style)
colors = [
    style.blue
    style.orange
    style.green
    style.purple
    style.darkGreen
    style.eventColor];

for k = 1:numel(gapSpec)
    color = colors(mod(k - 1, size(colors, 1)) + 1, :);
    lineStyle = '-';
    lineWidth = style.lineWidth;
    if gapSpec(k).role == "end_to_end"
        lineStyle = '--';
        lineWidth = 1.05;
    elseif gapSpec(k).role == "host_direct"
        lineStyle = '-.';
    end
    if gapSpec(k).isHostDirect
        lineWidth = style.lineWidth + 0.25;
    end
    plotFinite(ax, t, gapSpec(k).gap, lineStyle, color, lineWidth, gapSpec(k).label);
end
end

function values = gapSpecValues(gapSpec, mask)
values = [];
mask = logical(mask(:));
for k = 1:numel(gapSpec)
    gap = gapSpec(k).gap(:);
    n = min(numel(mask), numel(gap));
    if n > 0
        gap = gap(1:n);
        visible = mask(1:n);
        values = [values; gap(visible)]; %#ok<AGROW>
    end
end
end

function y = minGapSeries(gapSpec, n, roleFilter)
y = nan(n, 1);
roleFilter = string(roleFilter);
for k = 1:numel(gapSpec)
    if strlength(roleFilter) > 0 && gapSpec(k).role ~= roleFilter
        continue;
    end
    gap = gapSpec(k).gap(:);
    if numel(gap) ~= n
        continue;
    end
    fillMask = isfinite(gap) & (~isfinite(y) | gap < y);
    y(fillMask) = gap(fillMask);
end
end

function y = hostDirectGapSeries(gapSpec, host, n)
y = nan(n, 1);
if ~isfinite(host)
    return;
end

host = round(host);
for k = 1:numel(gapSpec)
    if gapSpec(k).follower == host && gapSpec(k).isHostDirect
        gap = gapSpec(k).gap(:);
        if numel(gap) == n
            y = gap;
            return;
        end
    end
end

for k = 1:numel(gapSpec)
    if gapSpec(k).follower == host
        gap = gapSpec(k).gap(:);
        if numel(gap) == n
            y = gap;
            return;
        end
    end
end
end

function err = positionErrorWithReference(tbl, vehicleId, prefix)
err = col(tbl, sprintf('%s_pos_err_%d', prefix, vehicleId));
if any(isfinite(err))
    return;
end

x = col(tbl, sprintf('%s_x_%d', prefix, vehicleId));
y = col(tbl, sprintf('%s_y_%d', prefix, vehicleId));
refX = col(tbl, sprintf('ref_x_%d', vehicleId));
refY = col(tbl, sprintf('ref_y_%d', vehicleId));
mask = isfinite(x) & isfinite(y) & isfinite(refX) & isfinite(refY);
err = nan(size(x));
err(mask) = hypot(x(mask) - refX(mask), y(mask) - refY(mask));
end

function v = hostVelocity(tbl, host)
if isfinite(host)
    v = col(tbl, sprintf('est_v_%d', host));
    if any(isfinite(v))
        return;
    end
    v = col(tbl, sprintf('ref_v_%d', host));
    if any(isfinite(v))
        return;
    end
end
v = col(tbl, 'host_velocity');
end

function plotFinite(ax, x, y, lineStyle, color, lineWidth, displayName)
mask = isfinite(x) & isfinite(y);
if any(mask)
    plot(ax, x(mask), y(mask), 'LineStyle', lineStyle, 'Color', color, ...
        'LineWidth', lineWidth, 'DisplayName', displayName);
end
end

function values = textColumn(tbl, name)
name = char(name);
if ismember(name, tbl.Properties.VariableNames)
    raw = tbl.(name);
    if isstring(raw)
        values = raw(:);
    elseif iscell(raw)
        values = string(raw(:));
    elseif iscategorical(raw)
        values = string(raw(:));
    else
        values = string(raw(:));
    end
else
    values = strings(height(tbl), 1);
end
values = strtrim(values);
end

function value = lastNonemptyText(tbl, name)
values = textColumn(tbl, name);
value = "";
for k = numel(values):-1:1
    candidate = strtrim(values(k));
    if strlength(candidate) > 0 && lower(candidate) ~= "nan"
        value = candidate;
        return;
    end
end
end

function data = jsonArrayOrEmpty(raw)
data = struct([]);
raw = strtrim(string(raw));
if strlength(raw) == 0 || raw == "[]" || lower(raw) == "nan"
    return;
end
try
    decoded = jsondecode(char(raw));
catch
    return;
end
if isempty(decoded)
    return;
end
if isstruct(decoded)
    data = decoded(:);
end
end

function value = numericStructField(item, fieldName)
value = NaN;
if ~isfield(item, fieldName)
    return;
end

raw = item.(fieldName);
if isnumeric(raw) || islogical(raw)
    if isscalar(raw)
        value = double(raw);
    end
elseif ischar(raw) || isstring(raw)
    parsed = str2double(string(raw));
    if isscalar(parsed)
        value = double(parsed);
    end
end
end

function value = stringStructField(item, fieldName)
value = "";
if ~isfield(item, fieldName)
    return;
end

raw = item.(fieldName);
if isstring(raw) || ischar(raw)
    value = strtrim(string(raw));
elseif isnumeric(raw) || islogical(raw)
    if isscalar(raw) && isfinite(double(raw))
        value = string(double(raw));
    end
elseif iscell(raw)
    parts = strtrim(string(raw(:)));
    parts = parts(strlength(parts) > 0);
    value = strjoin(parts, "|");
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

function y = firstFiniteSeries(varargin)
if nargin == 0
    y = [];
    return;
end

y = nan(size(varargin{1}(:)));
for k = 1:nargin
    candidate = varargin{k}(:);
    if numel(candidate) ~= numel(y)
        continue;
    end
    fillMask = ~isfinite(y) & isfinite(candidate);
    y(fillMask) = candidate(fillMask);
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

function value = finiteMean(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = mean(values);
end
end

function value = finiteStd(values)
values = values(isfinite(values));
if numel(values) < 2
    value = NaN;
else
    value = std(values);
end
end

function value = percentTrue(condition, universe)
condition = logical(condition(:));
universe = logical(universe(:));
n = min(numel(condition), numel(universe));
if n == 0
    value = NaN;
    return;
end

condition = condition(1:n);
universe = universe(1:n);
denom = sum(universe);
if denom == 0
    value = NaN;
else
    value = 100.0 * sum(condition & universe) / denom;
end
end

function value = finiteMin(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = min(values);
end
end

function value = maxFinite(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = max(values);
end
end

function value = maxOrNan(values)
value = maxFinite(values);
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

function applyTimeWindow(ax, timeWindow)
if numel(timeWindow) == 2 && all(isfinite(timeWindow)) && timeWindow(2) > timeWindow(1)
    xlim(ax, timeWindow);
end
end

function outputDir = resolveOutputDir(outputDirArg, filepath, scriptDir)
outputDirArg = strtrim(outputDirArg);
if strlength(outputDirArg) == 0
    outputDir = fullfile(fileparts(filepath), 'section_f_analysis');
    return;
end

outputDir = char(outputDirArg);
if ~isfolder(outputDir) && ~contains(outputDir, filesep) && ~contains(outputDir, '/')
    outputDir = fullfile(scriptDir, outputDir);
end
end

function ensureFolder(folder)
if ~isfolder(folder)
    mkdir(folder);
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
    out = fullfile(outputDir, string(name) + "." + fmt);
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

function name = fileBaseName(filepath)
[~, name] = fileparts(filepath);
name = string(name);
end

function style = paperStyle()
style.fontName = 'Times New Roman';
style.fontSize = 9;
style.titleSize = 10;
style.lineWidth = 1.25;
style.blue = [0.0000 0.4470 0.7410];
style.orange = [0.8500 0.3250 0.0980];
style.green = [0.1000 0.5500 0.3000];
style.darkGreen = [0.0000 0.3600 0.1800];
style.purple = [0.4940 0.1840 0.5560];
style.attackColor = [0.75 0.20 0.15];
style.attackShade = [1.00 0.78 0.52];
style.turnEdge = [0.00 0.42 0.62];
style.turnShade = [0.68 0.88 1.00];
style.eventColor = [0.20 0.20 0.20];
style.gridAlpha = 0.18;
end
