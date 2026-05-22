% identify_longitudinal_params_vehicle0.m
% 车辆0纵向动力学非线性参数辨识脚本
% MATLAB R2023a
%
% 对应 Markdown 文档中的“20 Hz 数据积分形式 + 固定 tau 参数辨识方案”：
%   true_throttle_0 是归一化油门 throttle_raw，而不是牛顿单位驱动力。
%   固定 tau = 0.16 s，使用积分形式辨识 k_T, c_r, beta：
%     a_j - a_i + (1/tau)*int(a)
%       = (1/tau)*k_T*int(throttle_raw)
%         -(1/tau)*c_r*Delta_t
%         -(1/tau)*beta*int(v^2 + 2*tau*a)
%   该形式避免直接拟合 noisy 的瞬时 adot，更适合 20 Hz 数据。
%
% 说明：
%   1. 本脚本默认使用 CSV 中 0 号车的 true_velocity_0 和 true_throttle_0。
%      true_acceleration_0 来自 IMU，只有幅值没有方向；脚本根据速度变化趋势给减速段赋负号。
%   2. 本脚本辨识归一化油门到线性模型等效输入的比例 k_T。
%   3. 若可以重构 feedforward_throttle(v)，可进一步使用 throttle_raw - throttle_ff(v)
%      分析前馈补偿后的残差动力学。

clear; clc; close all;

%% 1. 已知车辆参数
% 根据实验经验和理论模型，固定动力系统惯性时间常数 tau = 0.16 s。
tau = 0.16;             % 动力系统惯性时间常数，单位 s
m = 3.3;                % 车辆质量，单位 kg，用于后续由 beta 反算 c_d
inv_tau = 1 / tau;      % 1/tau = 6.25

%% 1.1 辨识配置
% 对应当前数据说明：true_acceleration_0 来自 IMU，但没有方向信息。
% 因此默认用速度微分的符号给 IMU 加速度幅值赋方向：
%   a = sign(dv/dt) * abs(true_acceleration_0)
% 当 |dv/dt| 很小时认为车辆近似匀速，置 a = 0，避免把 IMU 噪声当作正加速度。
useSignedImuAcceleration = true;
accSignThreshold = 0.02; % 单位 m/s^2，速度微分小于该阈值时认为加速度方向不可靠

% 对应 Markdown 第 11.4 节：剔除阶跃切换附近数据，降低执行器延迟和采样不同步影响。
excludeThrottleTransition = false;
transitionWindow = 0.020; % 单位 s，剔除油门阶跃变化前后数据

% 对应积分形式辨识方案：20 Hz 下推荐 0.25 s ~ 0.50 s 的积分窗口。
integralWindow = 0.750;   % 单位 s，滑动积分窗口长度
integralStep = 1;        % 滑动步长，单位：样本点

%% 2. 读取 CSV 数据
% CSV 与本脚本位于同一文件夹下。
scriptFullPath = mfilename('fullpath');
scriptDir = fileparts(scriptFullPath);
csvFile = fullfile(scriptDir, 'dist_luenberger_v1_20260522_161051.csv');

opts = detectImportOptions(csvFile, 'VariableNamingRule', 'preserve');
data = readtable(csvFile, opts);

%% 3. 选择 0 号车数据列
% 对应 Markdown 第 9.2 节：需要采集 t_k, v_k, a_k, u_k。
% 这里使用 0 号车的真实速度、记录加速度和真实油门输入。
time = data.time;
v_raw = data.true_velocity_0;
a_meas_raw = data.true_acceleration_0;
u_raw = data.true_throttle_0;

% 将绝对时间转换为从 0 开始的相对时间，便于计算和绘图。
time = time - time(1);

%% 4. 数据有效性检查与预处理
% 对应 Markdown 第 11 节：数据同步、滤波、计算加速度导数、剔除异常数据。
valid = isfinite(time) & isfinite(v_raw) & isfinite(a_meas_raw) & isfinite(u_raw);
time = time(valid);
v_raw = v_raw(valid);
a_meas_raw = a_meas_raw(valid);
u_raw = u_raw(valid);

% 按时间排序，避免日志中出现时间戳乱序。
[time, sortIdx] = sort(time);
v_raw = v_raw(sortIdx);
a_meas_raw = a_meas_raw(sortIdx);
u_raw = u_raw(sortIdx);

% 去除重复时间戳，避免 gradient 计算异常。
[time, uniqueIdx] = unique(time, 'stable');
v_raw = v_raw(uniqueIdx);
a_meas_raw = a_meas_raw(uniqueIdx);
u_raw = u_raw(uniqueIdx);

% 估计采样周期。
dt_vec = diff(time);
dt = median(dt_vec(dt_vec > 0));
fs = 1 / dt;

%% 5. 信号滤波
% 对应 Markdown 第 11.2 节：对速度和加速度进行滤波。
% 优先使用 Savitzky-Golay 滤波；若数据太短，则自动降低窗口长度。
% 注意：油门阶跃输入不做平滑处理，避免滤波改变阶跃边沿和输入幅值。
sgolayOrder = 3;
sgolayFrame = max(11, 2*floor(0.25*fs/2) + 1); % 约 0.25 s 的奇数窗口
sgolayFrame = min(sgolayFrame, 2*floor((numel(time)-1)/2) + 1);

if exist('sgolayfilt', 'file') == 2 && sgolayFrame > sgolayOrder
    v_filt = sgolayfilt(v_raw, sgolayOrder, sgolayFrame);
    a_meas_filt = sgolayfilt(a_meas_raw, sgolayOrder, sgolayFrame);
    u_proc = u_raw;
else
    % 若未安装 Signal Processing Toolbox 或数据点过少，则使用移动平均作为回退。
    movWindow = max(3, min(11, numel(time)));
    v_filt = movmean(v_raw, movWindow);
    a_meas_filt = movmean(a_meas_raw, movWindow);
    u_proc = u_raw;
end

% 由速度微分得到加速度方向参考，用于判断加速段和减速段。
a_from_v = gradient(v_filt, time);
if exist('sgolayfilt', 'file') == 2 && sgolayFrame > sgolayOrder
    a_from_v = sgolayfilt(a_from_v, sgolayOrder, sgolayFrame);
else
    a_from_v = movmean(a_from_v, movWindow);
end

% true_acceleration_0 是 IMU 加速度幅值，没有方向信息。
% 使用速度微分 a_from_v 的符号给 IMU 加速度赋方向，减速段变为负值。
accSign = sign(a_from_v);
accSign(abs(a_from_v) < accSignThreshold) = 0;
a_imu_signed = abs(a_meas_filt) .* accSign;

if useSignedImuAcceleration
    a_filt = a_imu_signed;
    accelerationSource = 'IMU true_acceleration_0 幅值 + 速度微分符号';
else
    a_filt = a_from_v;
    accelerationSource = '由 true_velocity_0 微分得到的加速度';
end

% 诊断带符号 IMU 加速度与速度微分加速度的一致性。
accDiff = a_imu_signed - a_from_v;
accDiffRmse = sqrt(mean(accDiff.^2));

%% 6. 计算加速度导数 adot
% 对应 Markdown 第 11.3 节：由滤波后的加速度计算 jerk，即 adot。
adot = gradient(a_filt, time);

% 对微分结果再进行一次轻度平滑，降低数值微分放大的高频噪声。
if exist('sgolayfilt', 'file') == 2 && sgolayFrame > sgolayOrder
    adot = sgolayfilt(adot, sgolayOrder, sgolayFrame);
else
    adot = movmean(adot, movWindow);
end

%% 7. 剔除异常和低质量数据
% 对应 Markdown 第 11.4 节：剔除静止、低速抖动、突变和异常数据。
vMin = 0.05;                  % 低速阈值，单位 m/s
accLimit = 8.0;               % 加速度幅值阈值，按小车实验经验设置
jerkLimit = 80.0;             % jerk 幅值阈值，抑制微分异常点
trimSamples = ceil(0.5 / dt); % 去掉首尾 0.5 s，降低滤波边界影响

idx = true(size(time));
idx = idx & abs(v_filt) > vMin;

if excludeThrottleTransition
    du = [0; abs(diff(u_proc))];
    transitionIdx = find(du > 1e-6);
    transitionSamples = ceil(transitionWindow / dt);
    for iTransition = 1:numel(transitionIdx)
        iStart = max(1, transitionIdx(iTransition) - transitionSamples);
        iEnd = min(numel(idx), transitionIdx(iTransition) + transitionSamples);
        idx(iStart:iEnd) = false;
    end
end

if numel(idx) > 2*trimSamples
    idx(1:trimSamples) = false;
    idx(end-trimSamples+1:end) = false;
end

% 若低速剔除后可用数据过少，则放宽低速条件，避免脚本无法运行。
if nnz(idx) < 50
    warning('有效数据点过少，已放宽低速剔除条件。请检查实验数据是否包含足够激励。');
    idx = isfinite(v_filt) & isfinite(u_proc);
end

%% 8. 分别使用两种加速度源进行积分形式辨识
% 对应用户要求：同时输出两套结果对比
%   1) a_from_v：由 true_velocity_0 微分得到的加速度
%   2) a_imu_signed：IMU true_acceleration_0 幅值 + 速度微分符号
%
% 两种数据源均使用相同的固定 tau 积分窗口回归：
%   a_j - a_i + (1/tau)*int(a)
%     = (1/tau)*k_T*int(throttle_raw)
%       -(1/tau)*c_r*Delta_t
%       -(1/tau)*beta*int(v^2 + 2*tau*a)
windowSamples = max(2, round(integralWindow / dt));

result_vdot = identifyWithAccelerationSource( ...
    'a_from_v', '由 true_velocity_0 微分得到的加速度', ...
    a_from_v, v_filt, u_proc, time, idx, tau, integralWindow, integralStep, accLimit, jerkLimit);

result_imu = identifyWithAccelerationSource( ...
    'a_imu_signed', 'IMU true_acceleration_0 幅值 + 速度微分符号', ...
    a_imu_signed, v_filt, u_proc, time, idx, tau, integralWindow, integralStep, accLimit, jerkLimit);

% 默认用于详细绘图的结果。可改为 result_vdot 或 result_imu。
primaryResult = result_imu;

%% 9. 输出辨识结果
fprintf('\n========== 车辆0纵向动力学参数辨识结果：两种加速度源对比 ==========' );
fprintf('\n数据文件: %s\n', csvFile);
fprintf('原始样本数: %d\n', height(data));
fprintf('估计采样周期 dt: %.6f s, 采样频率 fs: %.2f Hz\n', dt, fs);
fprintf('积分窗口长度: %.3f s, 窗口样本数: %d, 滑动步长: %d 样本\n', integralWindow, windowSamples, integralStep);
fprintf('固定 tau: %.4f s\n', tau);
fprintf('车辆质量 m: %.4f kg\n', m);
fprintf('加速度符号判定阈值 |dv/dt|: %.4f m/s^2\n', accSignThreshold);
fprintf('带符号 IMU 加速度与速度微分加速度的 RMSE: %.6f m/s^2\n', accDiffRmse);
fprintf('油门阶跃切换附近数据剔除: %d, 窗口: %.3f s\n', excludeThrottleTransition, transitionWindow);

printIdentificationResult(result_vdot);
printIdentificationResult(result_imu);

fprintf('\n若已知空气密度 rho 和迎风面积 Af，可用 c_d = 2*m*beta/(rho*Af) 反算空气阻力系数。\n');
fprintf('注意：当前 RMSE/R^2 基于固定 tau 的积分窗口回归量 Y，而不是 noisy 的瞬时 adot。\n');

%% 10. 保存结果
% 将两种加速度源、LS/NNLS 的结果保存到同一个 CSV 中，便于对比。
resultTable = table( ...
    [string(result_vdot.sourceKey); string(result_vdot.sourceKey); string(result_imu.sourceKey); string(result_imu.sourceKey)], ...
    [string(result_vdot.sourceName); string(result_vdot.sourceName); string(result_imu.sourceName); string(result_imu.sourceName)], ...
    ["LS"; "NNLS"; "LS"; "NNLS"], ...
    [tau; tau; tau; tau], ...
    [result_vdot.kT_ls; result_vdot.kT_nnls; result_imu.kT_ls; result_imu.kT_nnls], ...
    [result_vdot.cr_ls; result_vdot.cr_nnls; result_imu.cr_ls; result_imu.cr_nnls], ...
    [result_vdot.beta_ls; result_vdot.beta_nnls; result_imu.beta_ls; result_imu.beta_nnls], ...
    [result_vdot.rmse_ls; result_vdot.rmse_nnls; result_imu.rmse_ls; result_imu.rmse_nnls], ...
    [result_vdot.r2_ls; result_vdot.r2_nnls; result_imu.r2_ls; result_imu.r2_nnls], ...
    [result_vdot.numWindows; result_vdot.numWindows; result_imu.numWindows; result_imu.numWindows], ...
    'VariableNames', {'acceleration_source', 'acceleration_description', 'method', ...
    'tau', 'k_T', 'c_r', 'beta', 'RMSE_IntegralY', 'R2_IntegralY', 'num_windows'});

resultFile = fullfile(scriptDir, 'vehicle0_parameter_identification_results.csv');
writetable(resultTable, resultFile);

matFile = fullfile(scriptDir, 'vehicle0_parameter_identification_workspace.mat');
save(matFile, 'tau', 'm', 'time', 'v_filt', 'u_proc', 'adot', ...
    'a_meas_filt', 'a_imu_signed', 'a_from_v', 'accSignThreshold', ...
    'result_vdot', 'result_imu', 'primaryResult', 'integralWindow', 'windowSamples');

fprintf('\n结果 CSV 已保存至: %s\n', resultFile);
fprintf('MAT 工作区数据已保存至: %s\n', matFile);
%% 13. 绘图检查
% 图 1：滤波前后速度、加速度和输入信号。
figure('Name', '车辆0数据预处理检查');
tiledlayout(3,1);
nexttile;
plot(time, v_raw, 'Color', [0.75 0.75 0.75]); hold on;
plot(time, v_filt, 'b', 'LineWidth', 1.2);
ylabel('v (m/s)'); grid on;
legend('原始速度', '滤波速度');

nexttile;
plot(time, a_meas_raw, 'Color', [0.75 0.75 0.75]); hold on;
plot(time, a_meas_filt, 'Color', [0.85 0.35 0.35], 'LineWidth', 1.0);
plot(time, a_imu_signed, 'r', 'LineWidth', 1.2);
plot(time, a_from_v, 'b--', 'LineWidth', 1.0);
ylabel('a (m/s^2)'); grid on;
legend('CSV 原始 IMU 幅值', 'CSV 滤波 IMU 幅值', '带符号 IMU 加速度', '速度微分加速度');

nexttile;
plot(time, u_raw, 'Color', [0.75 0.75 0.75]); hold on;
plot(time, u_proc, 'k', 'LineWidth', 1.2);
ylabel('u'); xlabel('time (s)'); grid on;
legend('原始输入', '用于辨识的输入');

% 图 2：积分窗口回归量 Y 验证，这是当前 20 Hz 数据下的主要评价指标。
figure('Name', '车辆0积分窗口参数辨识验证');
tiledlayout(2,1);
nexttile;
plot(primaryResult.windowTime, primaryResult.Y, 'k', 'LineWidth', 1.1); hold on;
plot(primaryResult.windowTime, primaryResult.Y_hat_ls, 'b--', 'LineWidth', 1.1);
plot(primaryResult.windowTime, primaryResult.Y_hat_nnls, 'r-.', 'LineWidth', 1.1);
grid on;
xlabel('time (s)');
ylabel('Integral Y');
legend('实际窗口 Y', 'LS 预测 Y', 'NNLS 预测 Y', 'Location', 'best');
title(['积分窗口模型验证：', primaryResult.sourceName]);

nexttile;
plot(primaryResult.windowTime, primaryResult.Y - primaryResult.Y_hat_ls, 'b', 'LineWidth', 1.0); hold on;
plot(primaryResult.windowTime, primaryResult.Y - primaryResult.Y_hat_nnls, 'r', 'LineWidth', 1.0);
grid on;
xlabel('time (s)');
ylabel('residual');
legend('LS 残差', 'NNLS 残差', 'Location', 'best');

% 图 3：瞬时 adot 辅助检查。20 Hz 下该图只作为参考，不作为主要评价指标。
figure('Name', '车辆0瞬时adot辅助检查');
plot(time, primaryResult.adot, 'k', 'LineWidth', 1.0); hold on;
plot(time, primaryResult.adot_hat_ls, 'b--', 'LineWidth', 1.0);
plot(time, primaryResult.adot_hat_nnls, 'r-.', 'LineWidth', 1.0);
grid on;
xlabel('time (s)');
ylabel('adot / jerk (m/s^3)');
legend('由数据估计的 \dot{a}', 'LS 预测 \dot{a}', 'NNLS 预测 \dot{a}', 'Location', 'best');
title(['瞬时 jerk 辅助检查：', primaryResult.sourceName]);

%% 14. 本脚本使用的局部函数
function result = identifyWithAccelerationSource(sourceKey, sourceName, aSource, vSource, uSource, time, idxBase, tau, integralWindow, integralStep, accLimit, jerkLimit)
% 使用指定加速度源进行固定 tau 的积分窗口辨识。
    invTau = 1 / tau;
    dtLocal = median(diff(time));
    windowSamples = max(2, round(integralWindow / dtLocal));
    numSamples = numel(time);

    adot = gradient(aSource, time);
    idx = idxBase & isfinite(vSource) & isfinite(aSource) & isfinite(uSource) & isfinite(adot);
    idx = idx & abs(aSource) < accLimit & abs(adot) < jerkLimit;

    Y = [];
    Psi = [];
    windowTime = [];
    windowStartIndex = [];
    windowEndIndex = [];

    for iStart = 1:integralStep:(numSamples - windowSamples)
        iEnd = iStart + windowSamples;

        if ~all(idx(iStart:iEnd))
            continue;
        end

        tSeg = time(iStart:iEnd);
        vSeg = vSource(iStart:iEnd);
        aSeg = aSource(iStart:iEnd);
        uSeg = uSource(iStart:iEnd);

        deltaA = aSeg(end) - aSeg(1);
        intA = trapz(tSeg, aSeg);
        intU = trapz(tSeg, uSeg);
        deltaT = tSeg(end) - tSeg(1);
        intResistanceBasis = trapz(tSeg, vSeg.^2 + 2*tau*aSeg);

        Y(end+1, 1) = deltaA + invTau * intA; %#ok<AGROW>
        Psi(end+1, :) = [ ...
            invTau * intU, ...
            -invTau * deltaT, ...
            -invTau * intResistanceBasis ...
        ]; %#ok<AGROW>
        windowTime(end+1, 1) = 0.5 * (tSeg(1) + tSeg(end)); %#ok<AGROW>
        windowStartIndex(end+1, 1) = iStart; %#ok<AGROW>
        windowEndIndex(end+1, 1) = iEnd; %#ok<AGROW>
    end

    if numel(Y) < 20
        warning('加速度源 %s 的积分窗口有效样本过少。', sourceKey);
    end

    theta_ls = Psi \ Y;
    theta_nnls = lsqnonneg(Psi, Y);

    Y_hat_ls = Psi * theta_ls;
    Y_hat_nnls = Psi * theta_nnls;

    rmse_ls = sqrt(mean((Y - Y_hat_ls).^2));
    rmse_nnls = sqrt(mean((Y - Y_hat_nnls).^2));

    sst = sum((Y - mean(Y)).^2);
    if sst > eps
        r2_ls = 1 - sum((Y - Y_hat_ls).^2) / sst;
        r2_nnls = 1 - sum((Y - Y_hat_nnls).^2) / sst;
    else
        r2_ls = NaN;
        r2_nnls = NaN;
    end

    params_ls = struct('tau', tau, 'kT', theta_ls(1), 'cr', theta_ls(2), 'beta', theta_ls(3), 'valid', true);
    params_nnls = struct('tau', tau, 'kT', theta_nnls(1), 'cr', theta_nnls(2), 'beta', theta_nnls(3), 'valid', true);

    adot_hat_ls = physicalAdot(params_ls, vSource, aSource, uSource);
    adot_hat_nnls = physicalAdot(params_nnls, vSource, aSource, uSource);

    result = struct();
    result.sourceKey = sourceKey;
    result.sourceName = sourceName;
    result.numWindows = numel(Y);
    result.windowSamples = windowSamples;
    result.windowTime = windowTime;
    result.windowStartIndex = windowStartIndex;
    result.windowEndIndex = windowEndIndex;
    result.Psi = Psi;
    result.Y = Y;
    result.Y_hat_ls = Y_hat_ls;
    result.Y_hat_nnls = Y_hat_nnls;
    result.theta_ls = theta_ls;
    result.theta_nnls = theta_nnls;
    result.kT_ls = theta_ls(1);
    result.cr_ls = theta_ls(2);
    result.beta_ls = theta_ls(3);
    result.kT_nnls = theta_nnls(1);
    result.cr_nnls = theta_nnls(2);
    result.beta_nnls = theta_nnls(3);
    result.rmse_ls = rmse_ls;
    result.rmse_nnls = rmse_nnls;
    result.r2_ls = r2_ls;
    result.r2_nnls = r2_nnls;
    result.params_ls = params_ls;
    result.params_nnls = params_nnls;
    result.adot = adot;
    result.adot_hat_ls = adot_hat_ls;
    result.adot_hat_nnls = adot_hat_nnls;
end

function printIdentificationResult(result)
% 输出单个加速度源的 LS/NNLS 辨识结果。
    fprintf('\n========== 加速度源：%s ==========', result.sourceName);
    fprintf('\n有效积分窗口数: %d\n', result.numWindows);

    fprintf('\n--- 普通最小二乘 LS ---\n');
    fprintf('k_T   = %.8f\n', result.kT_ls);
    fprintf('c_r   = %.8f\n', result.cr_ls);
    fprintf('beta  = %.8f\n', result.beta_ls);
    fprintf('RMSE(Integral Y) = %.8f\n', result.rmse_ls);
    fprintf('R^2(Integral Y)  = %.8f\n', result.r2_ls);

    fprintf('\n--- 非负最小二乘 NNLS ---\n');
    fprintf('k_T   = %.8f\n', result.kT_nnls);
    fprintf('c_r   = %.8f\n', result.cr_nnls);
    fprintf('beta  = %.8f\n', result.beta_nnls);
    fprintf('RMSE(Integral Y) = %.8f\n', result.rmse_nnls);
    fprintf('R^2(Integral Y)  = %.8f\n', result.r2_nnls);
end

function adotHat = physicalAdot(params, v, a, throttleRaw)
% 使用固定 tau 和辨识参数计算瞬时 adot，用于辅助检查。
    if ~params.valid
        adotHat = NaN(size(v));
        return;
    end

    adotHat = ...
        -(1 / params.tau) * a ...
        + (params.kT / params.tau) * throttleRaw ...
        - (1 / params.tau) * (params.cr + params.beta * (v.^2 + 2*params.tau*a));
end

