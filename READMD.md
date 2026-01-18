# Khronos

Khronos 是一个使用 Rust 编写的高精度软件时钟。它通过 NTP 协议获取时间样本，并利用**卡尔曼滤波器**对测量结果进行平滑处理，从而提供一个稳定、抗抖动且能自动补偿时钟漂移（Drift）的高精度时间源。

## 主要特性

*   **卡尔曼滤波平滑**：本项目使用卡尔曼滤波器维护时间偏移（Offset）和频率漂移（Drift）两个状态，有效过滤网络延迟带来的测量噪声。
*   **自适应过程噪声**：通过 NIS（Normalized Innovation Squared）指标动态调整过程噪声 Q，能够在时钟行为发生突变时快速响应，在稳定时保持平滑。
*   **高精度单调计时**：内部使用 `quanta` 库提供的 TSC（Time Stamp Counter）级单调时钟作为基准，避免系统时钟回跳或调整带来的影响。
*   **高度可配置**：支持自定义 NTP 服务器列表、滤波参数、同步频率及 UI 刷新率。
*   **TUI 界面**：基于 `crossterm` 的终端界面，实时显示当前时间、滤波状态、漂移率（PPM）及最近一次 NTP 同步的详细数据。

## 构建与运行

### 前置要求

*   Rust 工具链 (Cargo)。

### 运行步骤

1.  **克隆项目**
    ```bash
    git clone https://github.com/DamoyY/khronos.git
    cd khronos
    ```

2.  **编译并运行**
    ```bash
    cargo run --release
    ```

3.  **退出**
    在终端中按下 `Ctrl + C` 即可退出程序。

## 配置说明

项目根目录下的 `config.yaml` 文件控制了程序的所有行为。修改配置无需重新编译，重启程序即可生效。

### 核心配置项概览

| 配置段 | 关键参数 | 说明 |
| :--- | :--- | :--- |
| **clock** | `initial_utc` | 程序启动时的初始时间基准（在首次 NTP 成功前使用）。 |
| **ui** | `refresh_interval_millis` | 界面刷新频率，默认 2ms。 |
| **ntp** | `servers` | NTP 服务器列表，默认包含阿里云、腾讯云、Google 等公共节点。 |
| **ntp** | `sync_interval_max_secs` | NTP 同步的最大间隔，默认 3秒。 |
| **kalman** | `initial_uncertainty` | 初始协方差矩阵的不确定度，越大表示越不信任初始状态。 |
| **kalman** | `delay_to_r_factor` | 将网络延迟转换为测量噪声 R 的系数。 |

## 原理简介

### 1. 时间模型
程序维护一个虚拟的软件时钟 `ProgramClock`。该时钟的时间计算公式为：
$$ T_{now} = T_{base} + \Delta T_{monotonic} $$
其中 $T_{base}$ 是基准 UTC 时间，$\Delta T_{monotonic}$ 是自基准点以来流逝的高精度单调时间。

### 2. 卡尔曼滤波器设计
Khronos 使用一个二维状态的卡尔曼滤波器：
*   **状态向量 $x$**: $[\theta, \dot{\theta}]^T$，其中 $\theta$ 是时间偏移（Offset），$\dot{\theta}$ 是时钟漂移率（Drift）。
*   **预测阶段 (Predict)**: 根据上一时刻的漂移率，预测当前时刻的偏移。
*   **更新阶段 (Correct)**:
    *   通过 NTP 获取测量值：$Offset_{measured}$。
    *   计算测量噪声 $R$：基于 NTP 的往返延迟（RTT），延迟越高，测量结果越不可信，$R$ 值越大。
    *   更新状态向量，得到最优估计的偏移量和平滑后的漂移率。

### 3. 自适应机制
程序计算 **NIS (Normalized Innovation Squared)** 指标并进行指数移动平均（EMA）。
*   当观测值与预测值偏差过大时，NIS 升高，程序会自动增大过程噪声 $Q$，使滤波器对新数据更敏感（快速收敛）。
*   当系统稳定时，NIS 降低，$Q$ 减小，滤波器更倾向于保持当前状态，过滤掉网络抖动。

## 目录结构

```text
khronos/
├── Cargo.toml          # 项目依赖与构建配置
├── config.yaml         # 运行时配置文件
└── src/
    ├── main.rs         # 程序入口
    ├── app.rs          # 应用程序主循环与 UI 逻辑
    ├── config.rs       # 配置加载与校验逻辑
    ├── ntp.rs          # NTP 协议实现与网络通信
    ├── kalman_filter.rs# 卡尔曼滤波器数学模型实现
    └── program_clock.rs# 软件时钟抽象
```

## 许可证

本项目遵循 [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0) 协议。详情请参阅 [LICENSE](LICENSE) 文件。