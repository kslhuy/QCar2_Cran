function artifacts = plot_rknet_comparator(filePath, outputDir, noShow)
%PLOT_RKNET_COMPARATOR Plot Robust KalmanNet vs EKF comparator logs.
%
% Usage:
%   plot_rknet_comparator
%   plot_rknet_comparator("rknet_comparison_20260501_221129.csv")
%   plot_rknet_comparator("logs/comparator/rknet_comparison_20260501_221129.csv")
%   plot_rknet_comparator(csvPath, "logs/plots", true)
%
% If filePath is omitted, the latest CSV under logs/comparator is used.

if nargin < 1
    filePath = "";
end
if nargin < 2
    outputDir = "";
end
if nargin < 3
    noShow = false;
end

robustDir = fileparts(mfilename("fullpath"));
if strlength(string(robustDir)) == 0
    robustDir = pwd;
end

csvPath = resolveInputFile(filePath, robustDir);
if strlength(string(outputDir)) == 0
    exportDir = fullfile(robustDir, "logs", "plots");
else
    exportDir = resolveOutputDir(outputDir, robustDir);
end
if ~exist(exportDir, "dir")
    mkdir(exportDir);
end

[data, rawTable] = loadComparatorCsv(csvPath);
timeAxis = data.timestamp - data.timestamp(1);
sampleCount = numel(timeAxis);
referenceAvailable = any(isfinite(data.ref_x));
attackSegments = extractAttackSegments(data, timeAxis);
updateDiagnostics = deriveUpdateDiagnostics(data);

gainErrors.K_x_x = data.K_x_x - data.ekf_K_x_x;
gainErrors.K_y_y = data.K_y_y - data.ekf_K_y_y;
gainErrors.K_psi_psi = data.K_psi_psi - data.ekf_K_psi_psi;
gainErrors.K_v_v = data.K_v_v - data.ekf_K_v_v;
gainErrors.K_norm = data.K_norm - data.ekf_K_norm;

visibleState = "on";
if noShow
    visibleState = "off";
end

fig1 = figure( ...
    "Name", "RKNet Comparator Figure 1", ...
    "Color", "w", ...
    "Position", [80 40 1300 1100], ...
    "Visible", visibleState);
safeSgtitle(fig1, sprintf("RKNet Comparator (1/4): %s", getFileName(csvPath)));

ax = subplot(4, 2, 1, "Parent", fig1);
plot(ax, timeAxis, data.K_x_x, "DisplayName", "RKNet K(x,x)", "LineWidth", 1.2); hold(ax, "on");
plot(ax, timeAxis, data.K_y_y, "DisplayName", "RKNet K(y,y)", "LineWidth", 1.2);
plot(ax, timeAxis, data.K_psi_psi, "DisplayName", "RKNet K(psi,psi)", "LineWidth", 1.2);
plot(ax, timeAxis, data.K_v_v, "DisplayName", "RKNet K(v,v)", "LineWidth", 1.2);
finishAxes(ax, "RKNet Gain Diagonals", "time [s]", "");
addAttackSpans(ax, attackSegments, false, false);
legend(ax, "show", "FontSize", 8, "Location", "best");

ax = subplot(4, 2, 2, "Parent", fig1);
plot(ax, timeAxis, data.ekf_K_x_x, "DisplayName", "EKF K(x,x)", "LineWidth", 1.2); hold(ax, "on");
plot(ax, timeAxis, data.ekf_K_y_y, "DisplayName", "EKF K(y,y)", "LineWidth", 1.2);
plot(ax, timeAxis, data.ekf_K_psi_psi, "DisplayName", "EKF K(psi,psi)", "LineWidth", 1.2);
plot(ax, timeAxis, data.ekf_K_v_v, "DisplayName", "EKF K(v,v)", "LineWidth", 1.2);
finishAxes(ax, "EKF Gain Diagonals", "time [s]", "");
addAttackSpans(ax, attackSegments, false, false);
legend(ax, "show", "FontSize", 8, "Location", "best");

ax = subplot(4, 2, 3, "Parent", fig1);
plot(ax, timeAxis, gainErrors.K_x_x, "DisplayName", "K(x,x) error", "LineWidth", 1.2); hold(ax, "on");
plot(ax, timeAxis, gainErrors.K_y_y, "DisplayName", "K(y,y) error", "LineWidth", 1.2);
plot(ax, timeAxis, gainErrors.K_psi_psi, "DisplayName", "K(psi,psi) error", "LineWidth", 1.2);
plot(ax, timeAxis, gainErrors.K_v_v, "DisplayName", "K(v,v) error", "LineWidth", 1.2);
plot(ax, timeAxis, gainErrors.K_norm, "--", "DisplayName", "K norm error", "LineWidth", 1.2);
yline(ax, 0.0, ":", "Color", [0 0 0], "LineWidth", 0.8, "HandleVisibility", "off");
finishAxes(ax, "Gain Error (RKNet - EKF)", "time [s]", "");
addAttackSpans(ax, attackSegments, false, false);
legend(ax, "show", "FontSize", 8, "Location", "best");

ax = subplot(4, 2, 4, "Parent", fig1);
selectedIds = selectedBranchIds(data.mask_selected_branch);
plot(ax, timeAxis, selectedIds, "DisplayName", "selected branch", "LineWidth", 1.3); hold(ax, "on");
plot(ax, timeAxis, data.mask_imu_mean, ":", "DisplayName", "pred imu mask", "LineWidth", 1.0);
plot(ax, timeAxis, data.mask_wheel_mean, ":", "DisplayName", "pred wheel mask", "LineWidth", 1.0);
yticks(ax, [0 1 2]);
yticklabels(ax, ["imu", "steer", "wheel"]);
ylim(ax, [-0.2 2.2]);
finishAxes(ax, "Predictor Mask Summary (used only if 'nn' predictor)", "time [s]", "");
addAttackSpans(ax, attackSegments, false, false);
yyaxis(ax, "right");
plot(ax, timeAxis, data.mask_selected_score, "DisplayName", "selected score", "LineWidth", 1.1, "Color", [0.8500 0.3250 0.0980]);
ylim(ax, [-0.05 1.05]);
yyaxis(ax, "left");
legend(ax, "show", "FontSize", 8, "Location", "best");

ax = subplot(4, 2, 5, "Parent", fig1);
plot(ax, timeAxis, data.delta_x, "DisplayName", "dx"); hold(ax, "on");
plot(ax, timeAxis, data.delta_y, "DisplayName", "dy");
plot(ax, timeAxis, data.delta_theta, "DisplayName", "dtheta");
plot(ax, timeAxis, data.delta_v, "DisplayName", "dv");
finishAxes(ax, "Signed State Differences (Robust - EKF)", "time [s]", "");
addAttackSpans(ax, attackSegments, false, false);
legend(ax, "show", "FontSize", 8, "Location", "best");

ax = subplot(4, 2, 6, "Parent", fig1);
plot(ax, timeAxis, double(data.source == "model"), "DisplayName", "model used", "LineWidth", 1.2); hold(ax, "on");
plot(ax, timeAxis, double(data.source == "fallback"), "DisplayName", "fallback used", "LineWidth", 1.2);
plot(ax, timeAxis, data.gps_valid, "DisplayName", "gps fresh", "LineWidth", 1.2);
plot(ax, timeAxis, data.gps_hold_valid, ":", "DisplayName", "gps hold valid", "LineWidth", 1.2);
ylim(ax, [-0.1 1.1]);
finishAxes(ax, "Source / GPS Flags", "time [s]", "");
addAttackSpans(ax, attackSegments, false, false);
legend(ax, "show", "FontSize", 8, "Location", "best");

ax = subplot(4, 2, 7, "Parent", fig1);
plot(ax, timeAxis, data.meas_mask_x, "DisplayName", "mask x (GPS)", "LineWidth", 1.2); hold(ax, "on");
plot(ax, timeAxis, data.meas_mask_y, "DisplayName", "mask y (GPS)", "LineWidth", 1.2);
plot(ax, timeAxis, data.meas_mask_psi, "DisplayName", "mask psi (IMU)", "LineWidth", 1.2);
plot(ax, timeAxis, data.meas_mask_v, "DisplayName", "mask v (Wheel)", "LineWidth", 1.2);
ylim(ax, [-0.05 1.05]);
finishAxes(ax, "Measurement Update Masks (Attenuates Innovation)", "time [s]", "");
addAttackSpans(ax, attackSegments, true, true);
legend(ax, "show", "FontSize", 8, "Location", "best");

ax = subplot(4, 2, 8, "Parent", fig1);
plot(ax, timeAxis, data.innov_x, "DisplayName", "innov x", "LineWidth", 1.2); hold(ax, "on");
plot(ax, timeAxis, data.innov_y, "DisplayName", "innov y", "LineWidth", 1.2);
plot(ax, timeAxis, data.innov_psi, "DisplayName", "innov psi", "LineWidth", 1.2);
plot(ax, timeAxis, data.innov_v, "DisplayName", "innov v", "LineWidth", 1.2);
finishAxes(ax, "Innovations (z - H*x_pred)", "time [s]", "");
addAttackSpans(ax, attackSegments, false, false);
legend(ax, "show", "FontSize", 8, "Location", "best");

fig2 = figure( ...
    "Name", "RKNet Comparator Figure 2", ...
    "Color", "w", ...
    "Position", [110 60 1200 1000], ...
    "Visible", visibleState);
safeSgtitle(fig2, sprintf("RKNet Comparator (2/4): %s", getFileName(csvPath)));

ax = subplot(4, 2, 1, "Parent", fig2);
plot(ax, data.robust_x, data.robust_y, "DisplayName", "Robust (Update)", "LineWidth", 1.8); hold(ax, "on");
plot(ax, data.pred_x, data.pred_y, ":", "DisplayName", "Robust (Pred)", "LineWidth", 1.2);
plot(ax, data.ekf_x, data.ekf_y, "--", "DisplayName", "EKF", "LineWidth", 1.4);
if referenceAvailable
    plot(ax, data.ref_x, data.ref_y, "-.", "DisplayName", "Clean Ref EKF", "LineWidth", 1.4, "Color", [0 0 0]);
end
axis(ax, "normal");
axis(ax, "tight");
finishAxes(ax, "XY Trajectory", "x [m]", "y [m]");
legend(ax, "show", "FontSize", 8, "Location", "best");

ax = subplot(4, 2, 2, "Parent", fig2);
plot(ax, timeAxis, data.position_error_norm, "--", "DisplayName", "|Robust - EKF| pos", "LineWidth", 1.2); hold(ax, "on");
if referenceAvailable
    plot(ax, timeAxis, data.robust_ref_position_error_norm, "DisplayName", "|Robust - Clean Ref| pos", "LineWidth", 1.6);
    plot(ax, timeAxis, data.ekf_ref_position_error_norm, ":", "DisplayName", "|EKF - Clean Ref| pos", "LineWidth", 1.2);
end
plot(ax, timeAxis, abs(data.heading_error), "DisplayName", "|Robust - EKF| heading", "LineWidth", 1.0);
if referenceAvailable
    plot(ax, timeAxis, abs(data.robust_ref_heading_error), "DisplayName", "|Robust - Clean Ref| heading", "LineWidth", 1.1);
    plot(ax, timeAxis, abs(data.ekf_ref_heading_error), ":", "DisplayName", "|EKF - Clean Ref| heading", "LineWidth", 1.0);
end
finishAxes(ax, "Absolute Errors / Clean Reference Deviation", "time [s]", "Error");
addAttackSpans(ax, attackSegments, false, false);
legend(ax, "show", "FontSize", 8, "Location", "best");

ax = subplot(4, 2, 3, "Parent", fig2);
if referenceAvailable
    plot(ax, timeAxis, abs(data.robust_ref_dx), "DisplayName", "|Robust - Clean Ref| x", "LineWidth", 1.6); hold(ax, "on");
    plot(ax, timeAxis, abs(data.ekf_ref_dx), "--", "DisplayName", "|EKF - Clean Ref| x", "LineWidth", 1.3);
    legend(ax, "show", "FontSize", 8, "Location", "best");
else
    noReferenceText(ax);
end
finishAxes(ax, "Absolute X Error to Clean EKF", "time [s]", "|error_x| [m]");
addAttackSpans(ax, attackSegments, false, false);

ax = subplot(4, 2, 4, "Parent", fig2);
if referenceAvailable
    plot(ax, timeAxis, abs(data.robust_ref_dy), "DisplayName", "|Robust - Clean Ref| y", "LineWidth", 1.6); hold(ax, "on");
    plot(ax, timeAxis, abs(data.ekf_ref_dy), "--", "DisplayName", "|EKF - Clean Ref| y", "LineWidth", 1.3);
    legend(ax, "show", "FontSize", 8, "Location", "best");
else
    noReferenceText(ax);
end
finishAxes(ax, "Absolute Y Error to Clean EKF", "time [s]", "|error_y| [m]");
addAttackSpans(ax, attackSegments, false, false);

ax = subplot(4, 2, 5, "Parent", fig2);
if referenceAvailable
    plot(ax, timeAxis, abs(data.robust_ref_dtheta), "DisplayName", "|Robust - Clean Ref| theta", "LineWidth", 1.6); hold(ax, "on");
    plot(ax, timeAxis, abs(data.ekf_ref_dtheta), "--", "DisplayName", "|EKF - Clean Ref| theta", "LineWidth", 1.3);
    legend(ax, "show", "FontSize", 8, "Location", "best");
else
    noReferenceText(ax);
end
finishAxes(ax, "Absolute Heading Error to Clean EKF", "time [s]", "|error_theta| [rad]");
addAttackSpans(ax, attackSegments, false, false);

ax = subplot(4, 2, 6, "Parent", fig2);
if referenceAvailable
    plot(ax, timeAxis, abs(data.robust_ref_dv), "DisplayName", "|Robust - Clean Ref| v", "LineWidth", 1.6); hold(ax, "on");
    plot(ax, timeAxis, abs(data.ekf_ref_dv), "--", "DisplayName", "|EKF - Clean Ref| v", "LineWidth", 1.3);
    legend(ax, "show", "FontSize", 8, "Location", "best");
else
    noReferenceText(ax);
end
finishAxes(ax, "Absolute Velocity Error to Clean EKF", "time [s]", "|error_v| [m/s]");
addAttackSpans(ax, attackSegments, false, false);

ax = subplot(4, 2, 7, "Parent", fig2);
plot(ax, timeAxis, data.steering, "DisplayName", "Steering", "Color", [0.8500 0.3250 0.0980]);
finishAxes(ax, "Steering Angle over Time", "time [s]", "steering [rad]");
addAttackSpans(ax, attackSegments, false, false);
legend(ax, "show", "FontSize", 8, "Location", "best");

ax = subplot(4, 2, 8, "Parent", fig2);
plot(ax, timeAxis, data.throttle, "DisplayName", "Throttle", "Color", [0.4660 0.6740 0.1880]);
finishAxes(ax, "Throttle over Time", "time [s]", "throttle");
addAttackSpans(ax, attackSegments, false, false);
legend(ax, "show", "FontSize", 8, "Location", "best");

fig3 = figure( ...
    "Name", "RKNet Comparator Update Diagnostics", ...
    "Color", "w", ...
    "Position", [140 80 1300 1100], ...
    "Visible", visibleState);
safeSgtitle(fig3, sprintf("RKNet Comparator (3/4): Update Diagnostics %s", getFileName(csvPath)));
channelNames = ["x", "y", "psi", "v"];
channelColors = [
    0.0000 0.4470 0.7410
    0.8500 0.3250 0.0980
    0.4660 0.6740 0.1880
    0.6350 0.0780 0.1840
];

ax = subplot(4, 1, 1, "Parent", fig3);
for idx = 1:numel(channelNames)
    plot(ax, timeAxis, updateDiagnostics.meas_mask(:, idx), ...
        "DisplayName", sprintf("mask %s", channelNames(idx)), ...
        "LineWidth", 1.2, ...
        "Color", channelColors(idx, :));
    hold(ax, "on");
end
ylim(ax, [-0.05 1.05]);
finishAxes(ax, "Measurement Update Mask with Attack Windows", "", "mask");
addAttackSpans(ax, attackSegments, true, true);
legend(ax, "show", "FontSize", 8, "Location", "best");

ax = subplot(4, 1, 2, "Parent", fig3);
for idx = 1:numel(channelNames)
    plot(ax, timeAxis, updateDiagnostics.rknet_effective_K_diag(:, idx), ...
        "DisplayName", sprintf("RKNet K_eff(%s)", channelNames(idx)), ...
        "LineWidth", 1.2, ...
        "Color", channelColors(idx, :));
    hold(ax, "on");
    plot(ax, timeAxis, updateDiagnostics.ekf_K_diag(:, idx), ":", ...
        "DisplayName", sprintf("EKF K(%s)", channelNames(idx)), ...
        "LineWidth", 1.0, ...
        "Color", channelColors(idx, :));
end
finishAxes(ax, "Effective Diagonal Gain: RKNet K_diag * mask vs EKF K_diag", "", "gain");
addAttackSpans(ax, attackSegments, false, false);
legend(ax, "show", "FontSize", 7, "Location", "best");

ax = subplot(4, 1, 3, "Parent", fig3);
for idx = 1:numel(channelNames)
    plot(ax, timeAxis, updateDiagnostics.masked_innovation(:, idx), ...
        "DisplayName", sprintf("masked innov %s", channelNames(idx)), ...
        "LineWidth", 1.1, ...
        "Color", channelColors(idx, :));
    hold(ax, "on");
end
yline(ax, 0.0, ":", "Color", [0 0 0], "LineWidth", 0.8, "HandleVisibility", "off");
finishAxes(ax, "Masked Innovation (measurement mask * innovation)", "", "masked innovation");
addAttackSpans(ax, attackSegments, false, false);
legend(ax, "show", "FontSize", 8, "Location", "best");

ax = subplot(4, 1, 4, "Parent", fig3);
for idx = 1:numel(channelNames)
    plot(ax, timeAxis, updateDiagnostics.update_correction(:, idx), ...
        "DisplayName", sprintf("update corr %s", channelNames(idx)), ...
        "LineWidth", 1.1, ...
        "Color", channelColors(idx, :));
    hold(ax, "on");
end
yline(ax, 0.0, ":", "Color", [0 0 0], "LineWidth", 0.8, "HandleVisibility", "off");
finishAxes(ax, "State Update Correction (full if logged; diagonal fallback for older CSV)", "time [s]", "correction");
addAttackSpans(ax, attackSegments, false, false);
legend(ax, "show", "FontSize", 8, "Location", "best");

fig4 = plotAttackFigure(csvPath, data, timeAxis, attackSegments, visibleState);

baseName = erase(getFileName(csvPath), ".csv");
artifacts = struct();
artifacts.figure1_png = fullfile(exportDir, baseName + "_figure1.png");
artifacts.figure2_png = fullfile(exportDir, baseName + "_figure2.png");
artifacts.update_diagnostics_png = fullfile(exportDir, baseName + "_update_diagnostics.png");
artifacts.attacks_png = fullfile(exportDir, baseName + "_attacks.png");
print(fig1, artifacts.figure1_png, "-dpng", "-r180");
print(fig2, artifacts.figure2_png, "-dpng", "-r180");
print(fig3, artifacts.update_diagnostics_png, "-dpng", "-r180");
print(fig4, artifacts.attacks_png, "-dpng", "-r180");

[diagnosticsMatPath, diagnosticsCsvPath] = exportDiagnosticFiles(csvPath, data, timeAxis, updateDiagnostics, attackSegments, exportDir);
artifacts.diagnostics_mat = diagnosticsMatPath;
artifacts.diagnostics_csv = diagnosticsCsvPath;
artifacts.summary_json = fullfile(exportDir, baseName + "_summary.json");
summary = buildSummary(csvPath, data, timeAxis, sampleCount, attackSegments, artifacts);
writeJsonFile(artifacts.summary_json, summary);

fprintf("Saved figure 1: %s\n", artifacts.figure1_png);
fprintf("Saved figure 2: %s\n", artifacts.figure2_png);
fprintf("Saved update diagnostics figure: %s\n", artifacts.update_diagnostics_png);
fprintf("Saved attack figure: %s\n", artifacts.attacks_png);
fprintf("Saved diagnostics CSV: %s\n", artifacts.diagnostics_csv);
fprintf("Saved diagnostics MAT: %s\n", artifacts.diagnostics_mat);
fprintf("Saved summary: %s\n", artifacts.summary_json);

if noShow
    close(fig1);
    close(fig2);
    close(fig3);
    close(fig4);
end

if nargout == 0
    assignin("base", "rknetArtifacts", artifacts);
    assignin("base", "rknetTable", rawTable);
    assignin("base", "rknetData", data);
    assignin("base", "timeSeconds", timeAxis);
end
end

function csvPath = resolveInputFile(filePath, robustDir)
logDir = fullfile(robustDir, "logs", "comparator");
filePath = string(filePath);
if strlength(strtrim(filePath)) == 0
    files = dir(fullfile(logDir, "*.csv"));
    if isempty(files)
        error("No comparator CSV files found in %s", logDir);
    end
    [~, order] = sort({files.name});
    files = files(order);
    csvPath = fullfile(files(end).folder, files(end).name);
    return;
end

candidates = buildCsvCandidates(filePath, robustDir, logDir);
for idx = 1:numel(candidates)
    if exist(candidates(idx), "file")
        csvPath = char(candidates(idx));
        return;
    end
end
error("Comparator CSV not found: %s", filePath);
end

function outputDir = resolveOutputDir(outputDir, robustDir)
outputDir = string(outputDir);
if isAbsolutePath(outputDir)
    outputDir = char(outputDir);
else
    outputDir = fullfile(robustDir, outputDir);
end
end

function candidates = buildCsvCandidates(filePath, robustDir, logDir)
filePath = string(filePath);
withExt = filePath;
if ~endsWith(lower(filePath), ".csv")
    withExt = filePath + ".csv";
end

if isAbsolutePath(filePath)
    candidates = unique([filePath, withExt], "stable");
    return;
end

candidates = [
    fullfile(pwd, filePath)
    fullfile(pwd, withExt)
    fullfile(logDir, filePath)
    fullfile(logDir, withExt)
    fullfile(robustDir, filePath)
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

function [data, rawTable] = loadComparatorCsv(csvPath)
opts = detectImportOptions(csvPath, ...
    "FileType", "text", ...
    "Delimiter", ",", ...
    "VariableNamingRule", "preserve");

numericColumns = getNumericColumns();
textColumns = getTextColumns();
presentNumeric = intersect(numericColumns, string(opts.VariableNames), "stable");
presentText = intersect(textColumns, string(opts.VariableNames), "stable");
if ~isempty(presentNumeric)
    opts = setvartype(opts, cellstr(presentNumeric), "double");
end
if ~isempty(presentText)
    opts = setvartype(opts, cellstr(presentText), "string");
end

rawTable = readtable(csvPath, opts);
if height(rawTable) == 0
    error("No data rows found in %s", csvPath);
end

data = struct();
for idx = 1:numel(numericColumns)
    name = char(numericColumns(idx));
    data.(name) = numericColumn(rawTable, name);
end
for idx = 1:numel(textColumns)
    name = char(textColumns(idx));
    data.(name) = stringColumn(rawTable, name);
end

inferredRealGpsValid = data.gps_valid;
flip = data.sensor_failure_gps_valid_flip;
flipMask = isfinite(inferredRealGpsValid) & isfinite(flip) & flip > 0.5;
inferredRealGpsValid(flipMask) = 1.0 - double(inferredRealGpsValid(flipMask) > 0.5);
loggedRealGpsValid = data.real_gps_valid;
missingRealGpsValid = ~isfinite(loggedRealGpsValid);
loggedRealGpsValid(missingRealGpsValid) = inferredRealGpsValid(missingRealGpsValid);
data.real_gps_valid = loggedRealGpsValid;

data.innov_psi = atan2(sin(data.innov_psi), cos(data.innov_psi));
end

function numericColumns = getNumericColumns()
numericColumns = [
    "timestamp"
    "tick"
    "dt"
    "motor_tach"
    "steering"
    "throttle"
    "gyro_z"
    "robust_x"
    "robust_y"
    "robust_theta"
    "robust_v"
    "ekf_x"
    "ekf_y"
    "ekf_theta"
    "ekf_v"
    "ref_x"
    "ref_y"
    "ref_theta"
    "ref_v"
    "delta_x"
    "delta_y"
    "delta_theta"
    "delta_v"
    "position_error_norm"
    "heading_error"
    "velocity_error"
    "robust_ref_dx"
    "robust_ref_dy"
    "robust_ref_dtheta"
    "robust_ref_dv"
    "robust_ref_position_error_norm"
    "robust_ref_heading_error"
    "robust_ref_velocity_error"
    "ekf_ref_dx"
    "ekf_ref_dy"
    "ekf_ref_dtheta"
    "ekf_ref_dv"
    "ekf_ref_position_error_norm"
    "ekf_ref_heading_error"
    "ekf_ref_velocity_error"
    "gps_valid"
    "real_gps_valid"
    "gps_hold_valid"
    "gps_age_sec"
    "sensor_failure_active"
    "sensor_failure_remaining_steps"
    "sensor_failure_intensity"
    "sensor_failure_imu_intensity"
    "sensor_failure_steer_intensity"
    "sensor_failure_wheel_intensity"
    "sensor_failure_gps_intensity"
    "sensor_failure_gps_xy_intensity"
    "sensor_failure_gps_valid_flip"
    "gps_x"
    "gps_y"
    "gps_theta"
    "accel_x"
    "accel_y"
    "accel_z"
    "pred_mask_mean"
    "pred_mask_min"
    "pred_mask_max"
    "mask_imu_mean"
    "mask_steer_mean"
    "mask_wheel_mean"
    "mask_selected_score"
    "mask_imu_active"
    "mask_steer_active"
    "mask_wheel_active"
    "meas_mask_x"
    "meas_mask_y"
    "meas_mask_psi"
    "meas_mask_v"
    "meas_mask_w"
    "masked_innov_x"
    "masked_innov_y"
    "masked_innov_psi"
    "masked_innov_v"
    "masked_innov_w"
    "K_norm"
    "K_x_x"
    "K_y_y"
    "K_psi_psi"
    "K_v_v"
    "K_w_w"
    "K_eff_x_x"
    "K_eff_y_y"
    "K_eff_psi_psi"
    "K_eff_v_v"
    "K_eff_w_w"
    "diag_update_x"
    "diag_update_y"
    "diag_update_psi"
    "diag_update_v"
    "diag_update_w"
    "update_corr_x"
    "update_corr_y"
    "update_corr_psi"
    "update_corr_v"
    "update_corr_w"
    "ekf_K_norm"
    "ekf_K_x_x"
    "ekf_K_y_y"
    "ekf_K_psi_psi"
    "ekf_K_v_v"
    "innov_x"
    "innov_y"
    "innov_psi"
    "innov_v"
    "innov_w"
    "pred_x"
    "pred_y"
];
end

function textColumns = getTextColumns()
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
end

function values = numericColumn(tableData, name)
rowCount = height(tableData);
if ~ismember(name, tableData.Properties.VariableNames)
    values = nan(rowCount, 1);
    return;
end
raw = tableData.(name);
if isnumeric(raw) || islogical(raw)
    values = double(raw);
elseif iscell(raw) || isstring(raw) || ischar(raw) || iscategorical(raw)
    values = str2double(string(raw));
else
    try
        values = double(raw);
    catch
        values = nan(rowCount, 1);
    end
end
values = values(:);
end

function values = stringColumn(tableData, name)
rowCount = height(tableData);
if ~ismember(name, tableData.Properties.VariableNames)
    values = strings(rowCount, 1);
    return;
end
raw = tableData.(name);
values = string(raw);
values = values(:);
values(ismissing(values)) = "";
end

function diagnostics = deriveUpdateDiagnostics(data)
measMask = stackColumns(data, ["meas_mask_x", "meas_mask_y", "meas_mask_psi", "meas_mask_v"]);
innovation = stackColumns(data, ["innov_x", "innov_y", "innov_psi", "innov_v"]);
maskedInnovation = preferLogged( ...
    stackColumns(data, ["masked_innov_x", "masked_innov_y", "masked_innov_psi", "masked_innov_v"]), ...
    measMask .* innovation);
rknetKDiag = stackColumns(data, ["K_x_x", "K_y_y", "K_psi_psi", "K_v_v"]);
ekfKDiag = stackColumns(data, ["ekf_K_x_x", "ekf_K_y_y", "ekf_K_psi_psi", "ekf_K_v_v"]);
effectiveKDiag = preferLogged( ...
    stackColumns(data, ["K_eff_x_x", "K_eff_y_y", "K_eff_psi_psi", "K_eff_v_v"]), ...
    rknetKDiag .* measMask);
diagUpdate = preferLogged( ...
    stackColumns(data, ["diag_update_x", "diag_update_y", "diag_update_psi", "diag_update_v"]), ...
    effectiveKDiag .* innovation);
updateCorrection = preferLogged( ...
    stackColumns(data, ["update_corr_x", "update_corr_y", "update_corr_psi", "update_corr_v"]), ...
    diagUpdate);

diagnostics = struct();
diagnostics.meas_mask = measMask;
diagnostics.innovation = innovation;
diagnostics.masked_innovation = maskedInnovation;
diagnostics.rknet_K_diag = rknetKDiag;
diagnostics.ekf_K_diag = ekfKDiag;
diagnostics.rknet_effective_K_diag = effectiveKDiag;
diagnostics.diag_update_contribution = diagUpdate;
diagnostics.update_correction = updateCorrection;
end

function matrix = stackColumns(data, columns)
sampleCount = numel(data.timestamp);
matrix = nan(sampleCount, numel(columns));
for idx = 1:numel(columns)
    name = char(columns(idx));
    if isfield(data, name)
        values = data.(name);
        if numel(values) == sampleCount
            matrix(:, idx) = values(:);
        end
    end
end
end

function result = preferLogged(logged, fallback)
result = logged;
if ~isequal(size(result), size(fallback))
    result = fallback;
    return;
end
missing = ~isfinite(result);
result(missing) = fallback(missing);
end

function ids = selectedBranchIds(branches)
ids = nan(numel(branches), 1);
branches = lower(strtrim(string(branches)));
ids(branches == "imu") = 0.0;
ids(branches == "steer") = 1.0;
ids(branches == "wheel") = 2.0;
end

function segments = extractAttackSegments(data, timeAxis)
active = data.sensor_failure_active > 0.5;
segments = emptyAttackSegments();
startIdx = [];
for idx = 1:numel(active)
    if active(idx) && isempty(startIdx)
        startIdx = idx;
    elseif ~active(idx) && ~isempty(startIdx)
        segments(end + 1) = buildAttackSegment(data, timeAxis, startIdx, idx - 1); %#ok<AGROW>
        startIdx = [];
    end
end
if ~isempty(startIdx)
    segments(end + 1) = buildAttackSegment(data, timeAxis, startIdx, numel(active)); %#ok<AGROW>
end
end

function segment = buildAttackSegment(data, timeAxis, startIdx, endIdx)
idxRange = startIdx:endIdx;
segmentIntensity = data.sensor_failure_intensity(idxRange);
finiteIntensity = isfinite(segmentIntensity);
peakIdxLocal = 1;
if any(finiteIntensity)
    finitePositions = find(finiteIntensity);
    [~, localPos] = max(segmentIntensity(finiteIntensity));
    peakIdxLocal = finitePositions(localPos);
end
peakIdx = startIdx + peakIdxLocal - 1;

labels = strings(numel(idxRange), 1);
for idx = 1:numel(idxRange)
    rowIdx = idxRange(idx);
    labels(idx) = attackTypeLabel(data.sensor_failure_branch_types(rowIdx), data.sensor_failure_gps_type(rowIdx));
end
labels(labels == "" | labels == "none") = [];
dominantLabel = "none";
if ~isempty(labels)
    uniqueLabels = unique(labels);
    counts = zeros(numel(uniqueLabels), 1);
    for idx = 1:numel(uniqueLabels)
        counts(idx) = nnz(labels == uniqueLabels(idx));
    end
    [~, maxIdx] = max(counts);
    dominantLabel = uniqueLabels(maxIdx);
end

peakIntensity = nan;
if ~isempty(segmentIntensity)
    peakIntensity = segmentIntensity(peakIdxLocal);
end

segment = struct( ...
    "start_index", startIdx, ...
    "end_index", endIdx, ...
    "start_time_seconds", timeAxis(startIdx), ...
    "end_time_seconds", timeAxis(endIdx), ...
    "duration_seconds", timeAxis(endIdx) - timeAxis(startIdx), ...
    "peak_time_seconds", timeAxis(peakIdx), ...
    "peak_intensity", peakIntensity, ...
    "dominant_type", dominantLabel, ...
    "branch_types", cleanLabel(data.sensor_failure_branch_types(peakIdx)), ...
    "gps_type", cleanLabel(data.sensor_failure_gps_type(peakIdx)), ...
    "peak_remaining_steps", data.sensor_failure_remaining_steps(peakIdx));
end

function segments = emptyAttackSegments()
segments = struct( ...
    "start_index", {}, ...
    "end_index", {}, ...
    "start_time_seconds", {}, ...
    "end_time_seconds", {}, ...
    "duration_seconds", {}, ...
    "peak_time_seconds", {}, ...
    "peak_intensity", {}, ...
    "dominant_type", {}, ...
    "branch_types", {}, ...
    "gps_type", {}, ...
    "peak_remaining_steps", {});
end

function label = attackTypeLabel(branchTypes, gpsType)
labels = strings(0, 1);
branchLabel = cleanLabel(branchTypes);
gpsLabel = cleanLabel(gpsType);
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

function label = cleanLabel(value)
label = strtrim(string(value));
if ismissing(label) || label == "" || lower(label) == "nan" || lower(label) == "none"
    label = "";
end
end

function addAttackSpans(ax, attackSegments, labelFirst, annotate)
if isempty(attackSegments)
    return;
end
colors = [
    0.8500 0.3250 0.0980
    0.9290 0.6940 0.1250
    0.4940 0.1840 0.5560
    0.6350 0.0780 0.1840
    0.4660 0.6740 0.1880
    0.3010 0.7450 0.9330
];
yl = ylim(ax);
hold(ax, "on");
for idx = 1:numel(attackSegments)
    startValue = attackSegments(idx).start_time_seconds;
    endValue = attackSegments(idx).end_time_seconds;
    if ~isfinite(startValue) || ~isfinite(endValue)
        continue;
    end
    visibility = "off";
    displayName = "";
    if labelFirst && idx == 1
        visibility = "on";
        displayName = "attack interval";
    end
    patchHandle = patch(ax, ...
        [startValue endValue endValue startValue], ...
        [yl(1) yl(1) yl(2) yl(2)], ...
        colors(mod(idx - 1, size(colors, 1)) + 1, :), ...
        "FaceAlpha", 0.08, ...
        "EdgeColor", "none", ...
        "HandleVisibility", visibility, ...
        "DisplayName", displayName);
    try
        uistack(patchHandle, "bottom");
    catch
    end
    if annotate
        text(ax, 0.5 * (startValue + endValue), yl(2), char(attackSegments(idx).dominant_type), ...
            "HorizontalAlignment", "center", ...
            "VerticalAlignment", "top", ...
            "FontSize", 7, ...
            "Color", [0.8500 0.3250 0.0980], ...
            "Interpreter", "none", ...
            "BackgroundColor", [1 1 1], ...
            "Margin", 1, ...
            "HandleVisibility", "off");
    end
end
ylim(ax, yl);
end

function fig = plotAttackFigure(csvPath, data, timeAxis, attackSegments, visibleState)
fig = figure( ...
    "Name", "RKNet Comparator Attack Timeline", ...
    "Color", "w", ...
    "Position", [170 100 1300 900], ...
    "Visible", visibleState);
safeSgtitle(fig, sprintf("RKNet Comparator (4/4): Attack Timeline %s", getFileName(csvPath)));

hasAttackMetadata = any(isfinite(data.sensor_failure_intensity)) ...
    || any(data.sensor_failure_active > 0.5) ...
    || any(strlength(strtrim(data.sensor_failure_branch_types)) > 0) ...
    || any(strlength(strtrim(data.sensor_failure_gps_type)) > 0);

if ~hasAttackMetadata
    for idx = 1:3
        ax = subplot(3, 1, idx, "Parent", fig);
        text(ax, 0.5, 0.5, "No attack metadata in this comparator CSV", ...
            "Units", "normalized", ...
            "HorizontalAlignment", "center", ...
            "VerticalAlignment", "middle", ...
            "FontSize", 12);
        axis(ax, "off");
    end
    return;
end

ax = subplot(3, 1, 1, "Parent", fig);
plot(ax, timeAxis, data.sensor_failure_active, "DisplayName", "attack active", "LineWidth", 1.2, "Color", [0 0 0]); hold(ax, "on");
plot(ax, timeAxis, data.sensor_failure_intensity, "DisplayName", "overall intensity", "LineWidth", 1.6, "Color", [0.8500 0.3250 0.0980]);
finishAxes(ax, "Attack Activity and Overall Intensity", "", "active / intensity");
yyaxis(ax, "right");
plot(ax, timeAxis, data.sensor_failure_remaining_steps, "--", "DisplayName", "remaining steps", "LineWidth", 1.1, "Color", [0.0000 0.4470 0.7410]);
ylabel(ax, "remaining steps");
yyaxis(ax, "left");
addAttackSpans(ax, attackSegments, false, false);
legend(ax, "show", "FontSize", 8, "Location", "best");

ax = subplot(3, 1, 2, "Parent", fig);
plot(ax, timeAxis, data.sensor_failure_imu_intensity, "DisplayName", "imu intensity", "LineWidth", 1.3); hold(ax, "on");
plot(ax, timeAxis, data.sensor_failure_steer_intensity, "DisplayName", "steer intensity", "LineWidth", 1.3);
plot(ax, timeAxis, data.sensor_failure_wheel_intensity, "DisplayName", "wheel intensity", "LineWidth", 1.3);
plot(ax, timeAxis, data.sensor_failure_gps_xy_intensity, "DisplayName", "gps xy intensity", "LineWidth", 1.3);
plot(ax, timeAxis, data.sensor_failure_gps_valid_flip, ":", "DisplayName", "gps valid flip", "LineWidth", 1.1, "Color", [0.5 0.5 0.5]);
finishAxes(ax, "Per-Channel Attack Intensity", "", "intensity");
addAttackSpans(ax, attackSegments, false, false);
legend(ax, "show", "FontSize", 8, "Location", "best");

ax = subplot(3, 1, 3, "Parent", fig);
[typeCodes, typeLabels] = attackTypeCodes(data);
plot(ax, timeAxis, typeCodes, "DisplayName", "attack type", "LineWidth", 1.4, "Color", [0.4940 0.1840 0.5560]);
if ~isempty(typeLabels)
    yticks(ax, 0:(numel(typeLabels) - 1));
    yticklabels(ax, cellstr(typeLabels));
end
finishAxes(ax, "Attack Type Timeline", "time [s]", "type id");
addAttackSpans(ax, attackSegments, false, false);
for idx = 1:numel(attackSegments)
    startValue = attackSegments(idx).start_time_seconds;
    endValue = attackSegments(idx).end_time_seconds;
    peak = attackSegments(idx).peak_intensity;
    if ~isfinite(startValue) || ~isfinite(endValue)
        continue;
    end
    label = string(attackSegments(idx).dominant_type);
    if isfinite(peak)
        label = sprintf("%s\npeak=%.3f", label, peak);
    end
    yl = ylim(ax);
    text(ax, 0.5 * (startValue + endValue), yl(2), label, ...
        "HorizontalAlignment", "center", ...
        "VerticalAlignment", "top", ...
        "FontSize", 7, ...
        "Interpreter", "none", ...
        "BackgroundColor", [1 1 1], ...
        "Margin", 2, ...
        "HandleVisibility", "off");
end
end

function [typeCodes, typeLabels] = attackTypeCodes(data)
sampleCount = numel(data.timestamp);
typeCodes = nan(sampleCount, 1);
typeLabels = strings(0, 1);
for idx = 1:sampleCount
    active = data.sensor_failure_active(idx);
    if ~isfinite(active) || active <= 0.5
        continue;
    end
    label = attackTypeLabel(data.sensor_failure_branch_types(idx), data.sensor_failure_gps_type(idx));
    if label == "none"
        label = "active";
    end
    labelIndex = find(typeLabels == label, 1);
    if isempty(labelIndex)
        typeLabels(end + 1) = label; %#ok<AGROW>
        labelIndex = numel(typeLabels);
    end
    typeCodes(idx) = labelIndex - 1;
end
end

function [matPath, csvPath] = exportDiagnosticFiles(csvPathInput, data, timeAxis, diagnostics, attackSegments, exportDir)
baseName = erase(getFileName(csvPathInput), ".csv");
sampleCount = numel(timeAxis);
[phaseName, phaseId] = buildPhaseNames(data, timeAxis, attackSegments);
attackLabel = strings(sampleCount, 1);
for idx = 1:sampleCount
    attackLabel(idx) = attackTypeLabel(data.sensor_failure_branch_types(idx), data.sensor_failure_gps_type(idx));
end

matPath = fullfile(exportDir, baseName + "_diagnostics.mat");
time_seconds = timeAxis; %#ok<NASGU>
timestamp = data.timestamp; %#ok<NASGU>
tick = data.tick; %#ok<NASGU>
phase_id = phaseId; %#ok<NASGU>
phase_name = phaseName; %#ok<NASGU>
attack_label = attackLabel; %#ok<NASGU>
attack_active = data.sensor_failure_active; %#ok<NASGU>
attack_intensity = data.sensor_failure_intensity; %#ok<NASGU>
gps_valid = data.gps_valid; %#ok<NASGU>
real_gps_valid = data.real_gps_valid; %#ok<NASGU>
gps_hold_valid = data.gps_hold_valid; %#ok<NASGU>
gps_age_sec = data.gps_age_sec; %#ok<NASGU>
meas_mask = diagnostics.meas_mask; %#ok<NASGU>
innovation = diagnostics.innovation; %#ok<NASGU>
masked_innovation = diagnostics.masked_innovation; %#ok<NASGU>
rknet_K_diag = diagnostics.rknet_K_diag; %#ok<NASGU>
rknet_effective_K_diag = diagnostics.rknet_effective_K_diag; %#ok<NASGU>
ekf_K_diag = diagnostics.ekf_K_diag; %#ok<NASGU>
diag_update_contribution = diagnostics.diag_update_contribution; %#ok<NASGU>
update_correction = diagnostics.update_correction; %#ok<NASGU>
robust_state = [data.robust_x, data.robust_y, data.robust_theta, data.robust_v]; %#ok<NASGU>
ekf_state = [data.ekf_x, data.ekf_y, data.ekf_theta, data.ekf_v]; %#ok<NASGU>
reference_state = [data.ref_x, data.ref_y, data.ref_theta, data.ref_v]; %#ok<NASGU>
controls = [data.steering, data.throttle, data.motor_tach]; %#ok<NASGU>
attack_segment_times = attackSegmentTimes(attackSegments); %#ok<NASGU>
attack_segment_indices = attackSegmentIndices(attackSegments); %#ok<NASGU>
save(matPath, ...
    "time_seconds", "timestamp", "tick", "phase_id", "phase_name", "attack_label", ...
    "attack_active", "attack_intensity", "gps_valid", "real_gps_valid", "gps_hold_valid", "gps_age_sec", ...
    "meas_mask", "innovation", "masked_innovation", "rknet_K_diag", "rknet_effective_K_diag", ...
    "ekf_K_diag", "diag_update_contribution", "update_correction", ...
    "robust_state", "ekf_state", "reference_state", "controls", ...
    "attack_segment_times", "attack_segment_indices");

diagnosticTable = table(timeAxis, data.timestamp, data.tick, phaseName, data.sensor_failure_active, attackLabel, ...
    data.sensor_failure_intensity, data.sensor_failure_gps_valid_flip, data.gps_valid, data.real_gps_valid, ...
    data.gps_hold_valid, data.gps_age_sec, data.robust_ref_position_error_norm, data.ekf_ref_position_error_norm, ...
    abs(data.robust_ref_heading_error), abs(data.ekf_ref_heading_error), ...
    abs(data.robust_ref_velocity_error), abs(data.ekf_ref_velocity_error), ...
    'VariableNames', { ...
    'time_seconds', 'timestamp', 'tick', 'phase', 'attack_active', 'attack_label', ...
    'sensor_failure_intensity', 'sensor_failure_gps_valid_flip', 'gps_valid', 'real_gps_valid', ...
    'gps_hold_valid', 'gps_age_sec', 'robust_ref_position_error_norm', 'ekf_ref_position_error_norm', ...
    'robust_ref_heading_error_abs', 'ekf_ref_heading_error_abs', ...
    'robust_ref_velocity_error_abs', 'ekf_ref_velocity_error_abs'});

channels = ["x", "y", "psi", "v"];
for idx = 1:numel(channels)
    ch = char(channels(idx));
    diagnosticTable.(sprintf('mask_%s', ch)) = diagnostics.meas_mask(:, idx);
    diagnosticTable.(sprintf('innov_%s', ch)) = diagnostics.innovation(:, idx);
    diagnosticTable.(sprintf('masked_innov_%s', ch)) = diagnostics.masked_innovation(:, idx);
    diagnosticTable.(sprintf('rknet_K_%s', ch)) = diagnostics.rknet_K_diag(:, idx);
    diagnosticTable.(sprintf('rknet_K_eff_%s', ch)) = diagnostics.rknet_effective_K_diag(:, idx);
    diagnosticTable.(sprintf('ekf_K_%s', ch)) = diagnostics.ekf_K_diag(:, idx);
    diagnosticTable.(sprintf('diag_update_%s', ch)) = diagnostics.diag_update_contribution(:, idx);
    diagnosticTable.(sprintf('update_corr_%s', ch)) = diagnostics.update_correction(:, idx);
end

csvPath = fullfile(exportDir, baseName + "_diagnostics.csv");
writetable(diagnosticTable, csvPath);
end

function times = attackSegmentTimes(attackSegments)
times = nan(numel(attackSegments), 4);
for idx = 1:numel(attackSegments)
    times(idx, :) = [ ...
        attackSegments(idx).start_time_seconds, ...
        attackSegments(idx).end_time_seconds, ...
        attackSegments(idx).peak_time_seconds, ...
        attackSegments(idx).peak_intensity];
end
end

function indices = attackSegmentIndices(attackSegments)
indices = nan(numel(attackSegments), 2);
for idx = 1:numel(attackSegments)
    indices(idx, :) = [attackSegments(idx).start_index, attackSegments(idx).end_index];
end
end

function [phaseName, phaseId] = buildPhaseNames(data, timeAxis, attackSegments)
sampleCount = numel(timeAxis);
attackMask = data.sensor_failure_active > 0.5;
recoveryMask = false(sampleCount, 1);
recoveryWindowSeconds = 2.0;
for idx = 1:numel(attackSegments)
    endTime = attackSegments(idx).end_time_seconds;
    recoveryMask = recoveryMask | (timeAxis > endTime & timeAxis <= endTime + recoveryWindowSeconds);
end
recoveryMask = recoveryMask & ~attackMask;
phaseName = repmat("clean", sampleCount, 1);
phaseName(attackMask) = "attack";
phaseName(recoveryMask) = "recovery";
phaseId = zeros(sampleCount, 1);
phaseId(attackMask) = 1;
phaseId(recoveryMask) = 2;
end

function summary = buildSummary(csvPath, data, timeAxis, sampleCount, attackSegments, artifacts)
summary = struct();
summary.schema_version = 1;
summary.generated_at_utc = char(datetime("now", "TimeZone", "UTC", "Format", "yyyy-MM-dd'T'HH:mm:ss.SSS'Z'"));
summary.source_csv = char(csvPath);
summary.log_name = char(getFileName(csvPath));
summary.sample_count = sampleCount;
summary.duration_seconds = scalarOrNull(timeAxis(end));
summary.usage_summary = struct( ...
    "model_samples", nnz(data.source == "model"), ...
    "fallback_samples", nnz(data.source == "fallback"), ...
    "gps_valid_samples", nnz(data.gps_valid > 0.5), ...
    "real_gps_valid_samples", nnz(data.real_gps_valid > 0.5), ...
    "gps_hold_valid_samples", nnz(data.gps_hold_valid > 0.5), ...
    "attack_active_samples", nnz(data.sensor_failure_active > 0.5), ...
    "mask_imu_active_samples", nnz(data.mask_imu_active > 0.5), ...
    "mask_steer_active_samples", nnz(data.mask_steer_active > 0.5), ...
    "mask_wheel_active_samples", nnz(data.mask_wheel_active > 0.5));
summary.reference_available = any(isfinite(data.ref_x));
summary.attack_segment_count = numel(attackSegments);
summary.artifacts = artifacts;
end

function value = scalarOrNull(raw)
if isempty(raw) || ~isfinite(raw)
    value = [];
else
    value = raw;
end
end

function writeJsonFile(path, value)
jsonText = jsonencode(value);
fid = fopen(path, "w");
if fid < 0
    error("Unable to write %s", path);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, "%s\n", jsonText);
delete(cleanup);
end

function finishAxes(ax, titleText, xText, yText)
title(ax, titleText, "Interpreter", "none");
if strlength(string(xText)) > 0
    xlabel(ax, xText);
end
if strlength(string(yText)) > 0
    ylabel(ax, yText);
end
grid(ax, "on");
box(ax, "on");
end

function noReferenceText(ax)
text(ax, 0.5, 0.5, "No clean EKF reference available", ...
    "Units", "normalized", ...
    "HorizontalAlignment", "center", ...
    "VerticalAlignment", "middle", ...
    "FontSize", 10);
end

function safeSgtitle(fig, textValue)
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
