# 车辆纵向动力学非线性参数辨识方案

## 1. 问题描述

车辆纵向动力学模型为：

```math
\left\{
\begin{aligned}
\dot{p} &= v, \\
\dot{v} &= a, \\
\dot{a} &= -\frac{1}{\tau}a + \frac{1}{m\tau}\bar{u} + f(v,a),
\end{aligned}
\right.
```

其中非线性项为：

```math
f(v,a)
=
-\frac{1}{\tau}
\left(
c_r
+
\frac{\rho A_f c_d}{2m}
\left(v^2+2\tau a\right)
\right)
```

已知参数为：

```math
\tau = 0.16 \ \mathrm{s}
```

```math
m = 3.3 \ \mathrm{kg}
```

需要辨识的参数主要包括：

```math
c_r, \quad c_d
```

其中：

- `p`：车辆绝对位置；
- `v`：车辆速度；
- `a`：车辆加速度；
- `\bar{u}`：控制输入，即油门/制动命令；
- `m`：车辆质量；
- `\tau`：动力系统惯性时间常数；
- `c_r`：滚动阻力相关系数；
- `c_d`：空气阻力系数；
- `\rho`：空气密度；
- `A_f`：车辆迎风面积。

---

## 2. 参数辨识基本原理

将车辆动力学方程展开：

```math
\dot{a}
=
-\frac{1}{\tau}a
+
\frac{1}{m\tau}\bar{u}
-
\frac{1}{\tau}
\left(
c_r
+
\frac{\rho A_f c_d}{2m}
(v^2+2\tau a)
\right)
```

移项可得：

```math
\dot{a}
+
\frac{1}{\tau}a
-
\frac{1}{m\tau}\bar{u}
=
-
\frac{1}{\tau}c_r
-
\frac{\rho A_f}{2m\tau}
(v^2+2\tau a)c_d
```

定义输出量：

```math
y
=
\dot{a}
+
\frac{1}{\tau}a
-
\frac{1}{m\tau}\bar{u}
```

则模型可写成线性参数回归形式：

```math
y = \phi_1 c_r + \phi_2 c_d
```

其中：

```math
\phi_1 = -\frac{1}{\tau}
```

```math
\phi_2
=
-\frac{\rho A_f}{2m\tau}
(v^2+2\tau a)
```

因此：

```math
y = \Phi \theta
```

```math
\theta
=
\begin{bmatrix}
c_r \\
c_d
\end{bmatrix}
```

```math
\Phi
=
\begin{bmatrix}
-\frac{1}{\tau} &
-\frac{\rho A_f}{2m\tau}(v^2+2\tau a)
\end{bmatrix}
```

该模型虽然对状态 `v` 和 `a` 是非线性的，但对未知参数 `c_r` 和 `c_d` 是线性的，因此可以使用最小二乘法进行辨识。

---

## 3. 推荐的等效参数辨识形式

在实际实验中，空气密度 `\rho` 和迎风面积 `A_f` 可能存在误差，因此更推荐先辨识组合参数：

```math
\beta = \frac{\rho A_f c_d}{2m}
```

此时非线性项写为：

```math
f(v,a)
=
-\frac{1}{\tau}
\left(
c_r + \beta(v^2+2\tau a)
\right)
```

动力学方程整理为：

```math
\dot{a}
+
\frac{1}{\tau}a
-
\frac{1}{m\tau}\bar{u}
=
-
\frac{1}{\tau}c_r
-
\frac{1}{\tau}(v^2+2\tau a)\beta
```

定义：

```math
y
=
\dot{a}
+
\frac{1}{\tau}a
-
\frac{1}{m\tau}\bar{u}
```

```math
\theta
=
\begin{bmatrix}
c_r \\
\beta
\end{bmatrix}
```

```math
\Phi
=
\begin{bmatrix}
-\frac{1}{\tau} &
-\frac{1}{\tau}(v^2+2\tau a)
\end{bmatrix}
```

则：

```math
y = \Phi \theta
```

对于第 `k` 个采样点：

```math
y_k
=
\begin{bmatrix}
-\frac{1}{\tau} &
-\frac{1}{\tau}(v_k^2+2\tau a_k)
\end{bmatrix}
\begin{bmatrix}
c_r \\
\beta
\end{bmatrix}
```

如果后续已知 `\rho` 和 `A_f`，则可以由 `\beta` 反算：

```math
c_d
=
\frac{2m}{\rho A_f}\beta
```

代入 `m = 3.3 kg`：

```math
c_d = \frac{6.6}{\rho A_f}\beta
```

---

## 4. 代入已知参数后的辨识模型

已知：

```math
\tau = 0.16
```

```math
m = 3.3
```

因此：

```math
\frac{1}{\tau} = 6.25
```

```math
\frac{1}{m\tau}
=
\frac{1}{3.3\times0.16}
\approx 1.8939
```

若 `\bar{u}` 已经是物理意义上的等效驱动力输入，则输出量为：

```math
y_k
=
\dot{a}_k
+
6.25a_k
-
1.8939\bar{u}_k
```

回归向量为：

```math
\psi_k
=
\begin{bmatrix}
-6.25 &
-6.25(v_k^2+0.32a_k)
\end{bmatrix}
```

参数向量为：

```math
\theta
=
\begin{bmatrix}
c_r \\
\beta
\end{bmatrix}
```

于是：

```math
y_k = \psi_k \theta
```

---

## 5. 批量最小二乘辨识方法

采集 `N` 组数据：

```math
\{v_k, a_k, \dot{a}_k, \bar{u}_k\}_{k=1}^{N}
```

构造输出向量：

```math
Y
=
\begin{bmatrix}
y_1 \\
y_2 \\
\vdots \\
y_N
\end{bmatrix}
```

构造回归矩阵：

```math
\Psi
=
\begin{bmatrix}
\psi_1 \\
\psi_2 \\
\vdots \\
\psi_N
\end{bmatrix}
```

整体模型为：

```math
Y = \Psi\theta
```

最小二乘估计为：

```math
\hat{\theta}
=
(\Psi^\mathrm{T}\Psi)^{-1}\Psi^\mathrm{T}Y
```

即：

```math
\begin{bmatrix}
\hat{c}_r \\
\hat{\beta}
\end{bmatrix}
=
(\Psi^\mathrm{T}\Psi)^{-1}\Psi^\mathrm{T}Y
```

如果存在明显测量噪声，可使用加权最小二乘：

```math
\hat{\theta}
=
(\Psi^\mathrm{T}W\Psi)^{-1}\Psi^\mathrm{T}WY
```

其中 `W` 为权重矩阵，用于降低噪声较大数据段的影响。

---

## 6. 输入增益同时辨识方案

在小车实验中，`\bar{u}` 往往是 PWM、归一化油门指令或电机控制命令，而不一定是真实物理驱动力。

此时建议引入输入增益 `k_u`：

```math
F = k_u\bar{u}
```

动力学模型变为：

```math
\dot{a}
=
-\frac{1}{\tau}a
+
\frac{k_u}{m\tau}\bar{u}
-
\frac{1}{\tau}
\left(
c_r + \beta(v^2+2\tau a)
\right)
```

整理得到：

```math
\dot{a}
+
\frac{1}{\tau}a
=
\frac{k_u}{m\tau}\bar{u}
-
\frac{1}{\tau}c_r
-
\frac{1}{\tau}(v^2+2\tau a)\beta
```

代入 `m = 3.3 kg` 和 `\tau = 0.16 s`：

```math
\dot{a} + 6.25a
=
1.8939k_u\bar{u}
-
6.25c_r
-
6.25(v^2+0.32a)\beta
```

定义：

```math
y_k = \dot{a}_k + 6.25a_k
```

```math
\psi_k
=
\begin{bmatrix}
1.8939\bar{u}_k &
-6.25 &
-6.25(v_k^2+0.32a_k)
\end{bmatrix}
```

```math
\theta
=
\begin{bmatrix}
k_u \\
c_r \\
\beta
\end{bmatrix}
```

于是：

```math
y_k = \psi_k\theta
```

该形式在实际 QCar 或小型无人车实验中通常更加实用。

### 6.1 已知前馈油门结构时的辨识方案

在当前 QCar 实验中，实际下发给车辆的油门命令为：

```math
throttle_{raw}
=
throttle_{ff}(v)
+
u_c
```

其中：

- `throttle_{raw}`：实际记录到的归一化油门，即 CSV 中的 `true_throttle_0`；
- `throttle_{ff}(v)`：根据当前车速计算的前馈油门，用于补偿车辆稳态阻力；
- `u_c`：控制器输出的动态补偿项；
- `u_c` 是根据线性模型计算得到的控制输入。

控制器使用的线性模型为：

```math
\left\{
\begin{aligned}
\dot{p} &= v, \\
\dot{v} &= a, \\
\dot{a} &= -\frac{1}{\tau}a + \frac{1}{\tau}u + \omega.
\end{aligned}
\right.
```

也就是说，控制器中的 `u` 不是物理驱动力，而是一个与加速度动态同量纲的等效控制输入。实际车辆执行的是归一化油门 `throttle_{raw}`。因此，若直接将 `true_throttle_0` 代入：

```math
\frac{1}{m\tau}\bar{u}
```

通常会导致模型输入尺度错误。

在该条件下，更合理的辨识思路是将油门前馈和控制器动态输入分开处理。

#### 6.1.1 推荐模型一：辨识油门到等效输入的增益

若只能从日志中获得 `throttle_{raw}`，而无法精确重构 `throttle_{ff}(v)` 和 `u_c`，则推荐使用如下模型：

```math
\dot{a}
+
\frac{1}{\tau}a
=
\frac{k_T}{\tau}throttle_{raw}
-
\frac{1}{\tau}c_r
-
\frac{1}{\tau}\beta(v^2+2\tau a)
```

其中 `k_T` 表示归一化油门到线性模型等效输入 `u` 的比例关系。代入 `\tau=0.16` 后：

```math
\dot{a}+6.25a
=
6.25k_T throttle_{raw}
-
6.25c_r
-
6.25\beta(v^2+0.32a)
```

构造线性回归：

```math
y_k
=
\dot{a}_k+6.25a_k
```

```math
\psi_k
=
\begin{bmatrix}
6.25throttle_{raw,k} &
-6.25 &
-6.25(v_k^2+0.32a_k)
\end{bmatrix}
```

```math
\theta
=
\begin{bmatrix}
k_T \\
c_r \\
\beta
\end{bmatrix}
```

于是：

```math
y_k = \psi_k\theta
```

该模型比直接使用 `1/(m\tau) true_throttle_0` 更合理，因为 `true_throttle_0` 是归一化油门，而不是牛顿单位驱动力。

#### 6.1.2 推荐模型二：可重构前馈时辨识残差动力学

如果可以在离线脚本中复现函数：

```math
throttle_{ff}(v)
```

则可先计算：

```math
u_{c,k}
=
throttle_{raw,k}
-
throttle_{ff}(v_k)
```

此时建议采用：

```math
\dot{a}
+
\frac{1}{\tau}a
=
\frac{k_c}{\tau}u_c
+
d_0
+
d_1v
+
d_2v^2
+
d_3a
```

其中：

- `k_c`：控制器命令到线性模型等效输入的比例系数；
- `d_0,d_1,d_2,d_3`：前馈补偿不完全、滚阻、气动阻力、执行器误差和未建模项的等效残差。

如果前馈设计非常准确，理论上 `d_0,d_1,d_2,d_3` 应较小。该模型适合验证前馈设计是否已经抵消了主要阻力。

#### 6.1.3 不建议在该条件下直接固定输入增益

在 `true_throttle_0` 为归一化油门时，不建议使用：

```math
\dot{a}
+
6.25a
-
1.8939 true\_throttle\_0
```

作为辨识输出。该写法隐含假设 `true_throttle_0` 已经是物理驱动力，但这与当前实验事实不符，因此会导致 `R^2` 较差或阻力参数被迫吸收输入尺度误差。

---

## 7. 约束最小二乘辨识

阻力相关参数通常应满足非负约束：

```math
c_r \ge 0
```

```math
\beta \ge 0
```

若同时辨识输入增益，也通常要求：

```math
k_u \ge 0
```

因此可使用非负最小二乘：

```math
\min_{\theta}\|Y-\Psi\theta\|_2^2
```

约束为：

```math
\theta \ge 0
```

相比普通最小二乘，非负最小二乘更符合车辆阻力参数的物理意义。

---

## 8. 在线递推最小二乘方案

如果希望在车辆运行过程中实时更新参数，可采用递推最小二乘法，RLS。

模型为：

```math
y_k = \psi_k\theta
```

递推公式为：

```math
K_k
=
P_{k-1}\psi_k^\mathrm{T}
\left(
\lambda + \psi_kP_{k-1}\psi_k^\mathrm{T}
\right)^{-1}
```

```math
\hat{\theta}_k
=
\hat{\theta}_{k-1}
+
K_k
\left(
y_k - \psi_k\hat{\theta}_{k-1}
\right)
```

```math
P_k
=
\frac{1}{\lambda}
\left(
P_{k-1} - K_k\psi_kP_{k-1}
\right)
```

其中：

- `\lambda`：遗忘因子；
- `P_k`：协方差矩阵；
- `K_k`：递推增益。

遗忘因子的推荐取值为：

```math
\lambda \in [0.95, 1.0]
```

若车辆参数基本不变，可取：

```math
\lambda = 0.99
```

若希望适应路面、电池状态等变化，可取：

```math
\lambda = 0.95 \sim 0.99
```

---

## 9. 数据采集方案设计

### 9.1 实验环境要求

建议满足以下条件：

- 平直路面，坡度尽量小；
- 路面附着条件稳定；
- 避免强风环境；
- 车辆质量保持不变；
- 电池电压尽量稳定；
- 采样频率建议不低于 `50 Hz`，更推荐 `100 Hz` 或以上；若当前只能使用约 `20 Hz` 数据，仍可进行离线批量辨识，但必须加强滤波、剔除阶跃切换附近数据，并优先使用三参数输入增益模型；
- 记录时间建议不少于 `60 s`，最好采集多组实验数据。

### 9.2 需要采集的数据

每个采样时刻建议记录：

```math
t_k, \quad p_k, \quad v_k, \quad a_k, \quad \bar{u}_k
```

其中：

- `t_k`：时间戳；
- `p_k`：车辆位置；
- `v_k`：车辆速度；
- `a_k`：车辆加速度；
- `\bar{u}_k`：控制输入；
- `\dot{a}_k`：加速度导数，可由加速度数据计算得到。

如果传感器不能直接测量 `a` 和 `\dot{a}`，可由速度或位置数据数值微分得到，但必须配合滤波。

对于当前约 `20 Hz` 的 recorder 数据，建议优先使用 `v_k` 经平滑后计算：

```math
a_k \approx \dot{v}_k
```

再由平滑后的 `a_k` 计算：

```math
\dot{a}_k
```

这样可以保证辨识数据满足模型中的基本关系 `\dot{v}=a`。如果日志中的加速度列与速度微分不一致，应优先以速度微分得到的加速度作为辨识用 `a_k`。

---

## 10. 推荐实验激励信号

### 10.1 阶跃油门实验

施加不同幅值的阶跃输入：

```math
\bar{u}(t) = u_0
```

例如：

```math
u_0 \in \{0.1, 0.2, 0.3, 0.4, 0.5\}
```

每个输入保持若干秒，使车辆经历明显加速过程。

优点：

- 实现简单；
- 数据直观；
- 适合初步辨识。

缺点：

- 激励频率较单一；
- 对高动态区域覆盖不足。

### 10.2 PRBS 输入实验

使用伪随机二进制序列作为油门输入：

```math
\bar{u}(t) \in \{u_{\min}, u_{\max}\}
```

例如：

```math
\bar{u}(t) \in \{0.15, 0.45\}
```

每隔 `0.5 s` 到 `2 s` 随机切换一次输入。

优点：

- 激励充分；
- 适合系统辨识；
- 能提高参数估计精度。

缺点：

- 车辆运动可能较剧烈；
- 需要注意实验安全。

### 10.3 加速-滑行实验

先给车辆一个油门输入，使其加速到一定速度，然后令：

```math
\bar{u} = 0
```

让车辆自然滑行减速。

此时动力学方程变为：

```math
\dot{a}
+
\frac{1}{\tau}a
=
-
\frac{1}{\tau}
\left(
c_r + \beta(v^2+2\tau a)
\right)
```

该实验对辨识阻力项非常有用，因为输入项消失后，车辆减速主要由阻力决定。

建议采集多个不同初始速度下的滑行数据。

---

## 11. 数据预处理方案

### 11.1 数据同步

确保以下信号时间戳对齐：

```math
v(t), \quad a(t), \quad \bar{u}(t)
```

如果控制输入和传感器数据存在延迟，应进行延迟补偿。

例如假设执行器延迟为 `T_d`，则应使用：

```math
\bar{u}(t-T_d)
```

参与辨识。

### 11.2 信号滤波

对速度和加速度进行低通滤波，例如：

- Butterworth 低通滤波；
- Savitzky-Golay 滤波；
- 移动平均滤波；
- 卡尔曼滤波。

推荐使用 Savitzky-Golay 滤波，因为它适合在平滑信号的同时计算导数。

对于当前约 `20 Hz` 数据，推荐使用以下设置作为初值：

- 对速度 `v` 使用 `0.25 s ~ 0.50 s` 的 Savitzky-Golay 窗口；
- 多项式阶数可取 `2` 或 `3`；
- 先平滑 `v`，再计算 `a=\dot{v}`；
- 再对 `a` 做一次平滑后计算 `\dot{a}`；
- 不建议对阶跃油门 `throttle_{raw}` 做强滤波，以免改变阶跃幅值和切换时刻。

### 11.3 计算加速度导数

如果已有加速度测量值，可用中心差分：

```math
\dot{a}_k
=
\frac{a_{k+1}-a_{k-1}}{2T_s}
```

其中 `T_s` 是采样周期。

更推荐使用局部多项式拟合方法，例如 Savitzky-Golay 方法，直接从平滑后的加速度估计 `\dot{a}_k`。

由于 `20 Hz` 下采样周期约为：

```math
T_s \approx 0.05 \ \mathrm{s}
```

直接二次微分会显著放大噪声，因此不建议使用未经滤波的：

```math
v \rightarrow a \rightarrow \dot{a}
```

链式差分。应采用“平滑—微分—再平滑—再微分”的方式，并通过验证集检查模型预测效果。

### 11.4 剔除异常数据

应剔除以下数据：

- 车辆静止或低速抖动段；
- 传感器突变点；
- 轮胎打滑数据；
- 输入饱和严重的数据；
- 油门阶跃切换前后 `0.1 s ~ 0.3 s` 的数据；
- 通信丢包或时间戳异常的数据；
- 制动/油门死区未建模的数据。

例如可设置低速剔除条件：

```math
|v_k| > v_{\min}
```

```math
v_{\min} = 0.05 \ \mathrm{m/s}
```

对于当前前馈加油门结构，还应特别注意：如果 `throttle_{raw}` 发生阶跃变化，车辆实际执行器和记录时间戳之间可能存在延迟，因此推荐剔除阶跃变化前后约 `0.2 s` 的数据，再进行最小二乘辨识。

---

## 12. 完整辨识流程

### Step 1：采集实验数据

记录：

```math
t_k, \quad v_k, \quad a_k, \quad \bar{u}_k
```

建议采集多组实验，包括：

- 不同油门阶跃实验；
- PRBS 输入实验；
- 不同初始速度下的滑行实验。

### Step 2：信号滤波

对 `v_k` 和 `a_k` 进行滤波，得到：

```math
\tilde{v}_k, \quad \tilde{a}_k
```

### Step 3：估计加速度导数

由滤波后的加速度计算：

```math
\dot{\tilde{a}}_k
```

### Step 4：构造输出向量

在当前 `true_throttle_0` 为归一化油门的情况下，不推荐使用“不辨识输入增益”的输出构造方式作为最终结果，只可作为对照实验。

若不辨识输入增益：

```math
y_k
=
\dot{\tilde{a}}_k
+6.25\tilde{a}_k
-1.8939\bar{u}_k
```

若同时辨识输入增益 `k_u`：

```math
y_k
=
\dot{\tilde{a}}_k
+6.25\tilde{a}_k
```

若使用当前前馈油门结构，并直接使用 `true_throttle_0=throttle_{raw}`，推荐构造：

```math
y_k
=
\dot{\tilde{a}}_k
+6.25\tilde{a}_k
```

### Step 5：构造回归矩阵

若辨识 `c_r` 和 `\beta`：

```math
\psi_k
=
\begin{bmatrix}
-6.25 &
-6.25(\tilde{v}_k^2+0.32\tilde{a}_k)
\end{bmatrix}
```

若辨识 `k_u`、`c_r` 和 `\beta`：

```math
\psi_k
=
\begin{bmatrix}
1.8939\bar{u}_k &
-6.25 &
-6.25(\tilde{v}_k^2+0.32\tilde{a}_k)
\end{bmatrix}
```

若 `\bar{u}_k` 是归一化油门 `throttle_{raw,k}`，更推荐将输入增益写成 `k_T`，使用：

```math
\psi_k
=
\begin{bmatrix}
6.25throttle_{raw,k} &
-6.25 &
-6.25(\tilde{v}_k^2+0.32\tilde{a}_k)
\end{bmatrix}
```

```math
\theta
=
\begin{bmatrix}
k_T \\
c_r \\
\beta
\end{bmatrix}
```

### Step 6：最小二乘求解

```math
\hat{\theta}
=
(\Psi^\mathrm{T}\Psi)^{-1}\Psi^\mathrm{T}Y
```

或使用非负最小二乘：

```math
\min_{\theta\ge0}\|Y-\Psi\theta\|_2^2
```

---

## 13. 模型验证方法

辨识完成后，应使用另一组未参与辨识的数据进行验证。

模型预测为：

```math
\hat{\dot{a}}_k
=
-\frac{1}{\tau}a_k
+
\frac{1}{m\tau}\bar{u}_k
-
\frac{1}{\tau}
\left(
\hat{c}_r
+
\hat{\beta}(v_k^2+2\tau a_k)
\right)
```

如果同时辨识了输入增益 `k_u`，则为：

```math
\hat{\dot{a}}_k
=
-\frac{1}{\tau}a_k
+
\frac{\hat{k}_u}{m\tau}\bar{u}_k
-
\frac{1}{\tau}
\left(
\hat{c}_r
+
\hat{\beta}(v_k^2+2\tau a_k)
\right)
```

比较 `\hat{\dot{a}}_k` 与实测或估计的 `\dot{a}_k`。

常用评价指标包括均方根误差：

```math
RMSE
=
\sqrt{
\frac{1}{N}
\sum_{k=1}^{N}
(\dot{a}_k-\hat{\dot{a}}_k)^2
}
```

以及拟合优度：

```math
R^2
=
1
-
\frac{
\sum_{k=1}^{N}
(\dot{a}_k-\hat{\dot{a}}_k)^2
}{
\sum_{k=1}^{N}
(\dot{a}_k-\bar{\dot{a}})^2
}
```

若 `RMSE` 较小且 `R^2` 接近 `1`，说明辨识效果较好。

---

## 14. Python 伪代码

```python
import numpy as np

# 已知参数
tau = 0.16
m = 3.3

# 已采集数据
# v: velocity array
# a: acceleration array
# u: throttle/brake command array
# dt: sampling time

# 计算 jerk
adot = np.gradient(a, dt)

# 构造输出，适用于 true_throttle_0 为归一化油门时辨识 kT, cr, beta
Y = adot + (1 / tau) * a

# 构造回归矩阵
Phi = np.column_stack([
    (1 / tau) * u,
    -(1 / tau) * np.ones_like(v),
    -(1 / tau) * (v**2 + 2 * tau * a)
])

# 最小二乘辨识
theta_hat = np.linalg.lstsq(Phi, Y, rcond=None)[0]

kT_hat = theta_hat[0]
cr_hat = theta_hat[1]
beta_hat = theta_hat[2]

print("kT =", kT_hat)
print("cr =", cr_hat)
print("beta =", beta_hat)

# 如果 rho 和 Af 已知，可进一步反算 cd
rho = 1.225
Af = 0.05  # 示例值，应根据车辆实际迎风面积修改

cd_hat = 2 * m * beta_hat / (rho * Af)
print("cd =", cd_hat)
```

---

## 15. 推荐最终实施方案

对于实际 QCar 或小型无人车，推荐采用以下方案：

1. 不直接辨识 `c_d`，先辨识等效参数 `\beta`；
2. 当前 `true_throttle_0` 是归一化油门或油门比例，不是牛顿单位驱动力，因此必须辨识输入尺度；
3. 如果使用 `true_throttle_0=throttle_{raw}`，推荐辨识 `k_T,c_r,\beta`，其中 `k_T` 表示归一化油门到线性模型等效输入的比例；
4. 使用如下辨识模型：

```math
\dot{a}
+
6.25a
=
6.25k_T throttle_{raw}
-
6.25c_r
-
6.25(v^2+0.32a)\beta
```

5. 构造线性回归：

```math
y_k = \psi_k\theta
```

其中：

```math
y_k = \dot{a}_k + 6.25a_k
```

```math
\psi_k
=
\begin{bmatrix}
6.25throttle_{raw,k} &
-6.25 &
-6.25(v_k^2+0.32a_k)
\end{bmatrix}
```

```math
\theta
=
\begin{bmatrix}
k_T \\
c_r \\
\beta
\end{bmatrix}
```

6. 如果可以重构 `throttle_{ff}(v)`，则可进一步计算 `u_c=throttle_{raw}-throttle_{ff}(v)`，用于分析前馈补偿后的残差动力学；
7. 当前采样频率约为 `20 Hz`，应使用速度平滑微分得到 `a`，并剔除油门阶跃切换前后约 `0.2 s` 的数据；
8. 使用普通最小二乘或非负最小二乘求解：

```math
\hat{\theta}
=
(\Psi^\mathrm{T}\Psi)^{-1}\Psi^\mathrm{T}Y
```

或：

```math
\min_{\theta\ge0}\|Y-\Psi\theta\|_2^2
```

---

## 16. 总结

车辆纵向非线性阻力项的参数辨识可以转化为线性参数估计问题。核心思路是将动力学方程整理为：

```math
y_k = \psi_k\theta
```

由于模型对未知参数 `c_r`、`\beta` 以及可选的 `k_u` 是线性的，因此可以直接使用最小二乘法进行辨识。

实际实验中推荐辨识：

```math
k_T, \quad c_r, \quad \beta
```

其中 `k_T` 用于描述归一化油门 `true_throttle_0` 到线性模型等效输入 `u` 的比例关系。

若控制输入 `\bar{u}` 已经是物理意义上的等效驱动力，则只需辨识：

```math
c_r, \quad \beta
```

最后，如果空气密度 `\rho` 和车辆迎风面积 `A_f` 已知，则可由：

```math
c_d = \frac{2m}{\rho A_f}\beta
```

进一步得到空气阻力系数 `c_d`。
