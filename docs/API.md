# robotpilots_algorithm API 文档

嵌入式机器人算法库，提供 CRC 校验、PID 控制、卡尔曼滤波、扩展卡尔曼滤波、矩阵运算等算法组件。

所有组件均位于命名空间 `robotpilots::algorithm` 中。

## 目录

1. [CRC 校验](#1-crc-校验)
2. [数学工具](#2-数学工具)
3. [PID 控制器](#3-pid-控制器)
4. [矩阵库](#4-矩阵库)
5. [卡尔曼滤波](#5-卡尔曼滤波)
6. [扩展卡尔曼滤波](#6-扩展卡尔曼滤波)
7. [类型与枚举](#7-类型与枚举)
8. [使用示例](#8-使用示例)

---

## 1. CRC 校验

头文件：`robotpilots/algorithm/algo_crc.hpp`

提供 CRC8 / CRC16 的校验与计算，支持 `etl::span` 与裸指针两种重载。

### 校验

```cpp
// span 重载：buffer 为数据视图（不持有数据）
bool Crc8Verify(const DataReceiver<uint8_t> buffer, uint8_t crc);
bool Crc16Verify(const DataReceiver<uint8_t> buffer, uint16_t crc);

// 裸指针重载：buffer 为数据指针，len 为数据长度
bool Crc8Verify(const uint8_t *buffer, uint8_t crc, size_t len);
bool Crc16Verify(const uint8_t *buffer, uint16_t crc, size_t len);
```

- 参数 `buffer`：待校验数据；`crc`：期望的校验值；`len`：数据长度。
- 返回值：校验通过返回 `true`，否则返回 `false`。
- 说明：裸指针重载已做判空，`buffer == nullptr && len > 0` 时直接返回 `false`。

### 计算

```cpp
// span 重载
uint8_t Crc8Calculate(const DataReceiver<uint8_t> buffer);
uint16_t Crc16Calculate(const DataReceiver<uint8_t> buffer);

// 裸指针重载
uint8_t Crc8Calculate(const uint8_t *buffer, size_t len);
uint16_t Crc16Calculate(const uint8_t *buffer, size_t len);
```

- 返回值：计算得到的 CRC 校验值（`uint8_t` / `uint16_t`）。

---

## 2. 数学工具

头文件：`robotpilots/algorithm/algo_math.hpp`（纯头文件）

```cpp
// 循环限幅：将 val 映射到 [lo, hi) 区间
inline float wrap(float val, float lo, float hi);

// 符号函数：返回 1 / -1 / 0
inline float sign(float val);

// 快速平方根倒数 1/sqrt(x)（Quake 算法，精度约 0.1%）
inline float invSqrt(float x);

// 一阶低通滤波：result = last * alpha + current * (1 - alpha)
inline float lowPassFilter(float last, float current, float alpha);
```

- `wrap`：`lo >= hi` 时直接返回 `val`；`val` 超出区间时按 `hi - lo` 循环映射，结果落在 `[lo, hi)`。
- `lowPassFilter`：`alpha` 越接近 1 越平滑（`0 < alpha < 1`）。

---

## 3. PID 控制器

头文件：`robotpilots/algorithm/algo_pid.hpp`

类 `CAlgoPid`。

### 误差计算模式

```cpp
enum class EPidErrorMode
{
    NORMAL,   // 正常模式
    ANGLE,    // 角度模式（误差范围 -180.0f ~ 180.0f）
    MACHINE,  // 机械模式（误差范围由 MachineModeErrorRange 决定）
};
```

### 初始化参数

```cpp
struct SAlgoInitParam_Pid
{
    uint16_t tickRate = 1000;                  // 定时器频率（Hz）
    float_t kp = 0.0f;                         // 比例系数
    float_t ki = 0.0f;                         // 积分系数
    float_t kd = 0.0f;                         // 微分系数
    float_t Input_deadband = 0.0f;             // 输入死区
    float_t maxOutput = 0.0f;                  // 最大输出
    float_t maxIntegral = 0.0f;                // 最大积分
    float_t input_integralSeparation = 0.0f;   // 积分分离值
    float_t sustainable_output = 0.0f;         // 可持续输出
    uint32_t sustainable_time = 0;             // 可持续时间
    uint32_t recover_time = 0;                 // 恢复时间
    uint16_t MachineModeErrorRange = 8192;     // 机械模式误差范围
    EPidErrorMode errorMode = EPidErrorMode::NORMAL;
};
```

### 成员函数

```cpp
// 初始化（参数不能为空，tickRate 不能为 0）
EAppStatus InitPID(const SAlgoInitParam_Pid *pStructInitParam);

// 更新并返回输出
float_t UpdatePidController(const float_t &target, const float_t &measure);

// 重置（清零积分/微分/上次误差等内部状态）
EAppStatus ResetPidController();
```

- `InitPID`：成功返回 `APP_OK`；参数为空、状态忙或 `tickRate == 0` 时返回 `APP_ERROR`。
- `UpdatePidController`：`target` 目标值、`measure` 测量值；返回 PID 输出，未初始化时返回 0。
- `ResetPidController`：仅在 `APP_BUSY` 时返回 `APP_ERROR`，其余情况清零内部状态并返回 `APP_OK`。
- 公开成员 `EAppStatus PidStatus` 用于查询当前状态。

---

## 4. 矩阵库

头文件：`robotpilots/algorithm/rp_matrix.hpp`（纯头文件，基于 CMSIS-DSP）

模板类 `Matrixt<T, N>`：`T` 为元素类型（通常为 `float`），`N` 为矩阵元素总数容量上限（编译期确定，栈上分配，无运行时内存分配）。

### 构造

```cpp
Matrixt();                      // 默认：0x0 空矩阵
Matrixt(int rows, int cols);    // rows x cols 矩阵（元素总数须 <= N）
Matrixt(Matrixt&& mat);         // 移动构造
Matrixt(const Matrixt& mat);    // 拷贝构造
```

说明：`rows * cols > N` 或行列非法时退化为 0x0 空矩阵。

### 常用成员

```cpp
int rows() const;               // 行数
int cols() const;               // 列数
bool empty() const;             // 是否为空矩阵
size_t size() const;            // 元素总数
T* get_data();                  // 底层数据指针

T* operator[](const int& row);              // 取第 row 行首元素指针（不检查边界）
Matrixt<T,N>& operator=(const Matrixt&);    // 拷贝赋值
// 另有 + - *（矩阵乘/标量乘）/ 及对应复合赋值、比较等运算符重载
```

- `mat[i][j]` 等价于 `(mat[i])[j]`；`operator[]` 不检查边界，越界由调用方负责（符合 C++ 惯例）。

### 自由函数

```cpp
Matrixt<T, N> zeros(int rows, int cols);             // 零矩阵
Matrixt<T, N> ones(int rows, int cols);              // 全 1 矩阵
Matrixt<T, N> eye(int dims);                         // 单位阵
Matrixt<T, N> trans(const Matrixt<T, N>& mat);       // 转置
float trace(const Matrixt<T, N>& mat);               // 迹
float norm(const Matrixt<T, N>& mat);                // 范数
Matrixt<T, N> inv(const Matrixt<T, N>& mat);         // 逆（奇异时返回零矩阵）
Matrixt<T, N> hat(const Matrixt<T, N>& mat);         // 3x1 -> 3x3 反对称矩阵
Matrixt<T, N> vee(const Matrixt<T, N>& mat);         // 3x3 反对称矩阵 -> 3x1
Matrixt<T, N> cross(const Matrixt<T, N>& a, const Matrixt<T, N>& b); // 叉乘
```

- `hat`/`vee`/`cross` 用于三维姿态/旋转运算，要求 3x1 或 3x3 维度。

---

## 5. 卡尔曼滤波

头文件：`robotpilots/algorithm/algo_kf_filter.hpp`（纯头文件）

模板类 `CAlgo_Kf<N>`：`N` 为矩阵元素总数容量上限（编译期确定）。继承 `CFilterBase`。

### 初始化参数

```cpp
struct SAlgoKfInitParam : public SFilterInitParam_Base
{
    float_t DT = 0.0f;                        // 调度周期
    uint8_t z_size = 0;                       // 观测量维度
    uint8_t u_size = 0;                       // 输入量维度
    uint8_t x_size = 0;                       // 状态量维度
    bool use_auto_adjustment = false;         // 是否动态调整 H/R/K
    etl::vector<uint8_t, 16> measurement_map;      // 观测量->状态量映射
    etl::vector<float, 16> measurement_degree;     // 观测量与状态量缩放关系
    etl::vector<float, 16> r_diagonal_elements;    // R 矩阵对角线元素
    etl::vector<float, 16> state_min_variance;     // P 对角线最小值
};
```

约束：`x_size`/`z_size`/`u_size` 必须非零且不超过 N，且各矩阵元素总数（`x_size²`、`z_size²`、`x_size*z_size` 等）不超过 N。

### 输出信息

```cpp
struct SAlgoKfInfo
{
    bool initialized = false;         // 是否完成初始化
    Matrixt<float, N> filtered_value; // 滤波输出
} Kf_Info;
```

### 成员函数

```cpp
void Set_F(const Matrixt<float, N>& F);   // 状态转移矩阵
void Set_B(const Matrixt<float, N>& B);   // 控制矩阵
void Set_Q(const Matrixt<float, N>& Q);   // 过程噪声协方差
void Set_H(const Matrixt<float, N>& H);   // 观测矩阵
void Set_R(const Matrixt<float, N>& R);   // 观测噪声协方差
void Set_Measured_Vector(const Matrixt<float, N>& v); // 原始测量向量
void Set_xhat(const Matrixt<float, N>& xhat);  // 初始状态
void Set_DT(float dt);                     // 调度周期
void Set_P(const Matrixt<float, N>& P);    // 后验协方差

EAppStatus InitAlgo_(SFilterInitParam_Base &param);  // 初始化
EAppStatus UpdateHandler_();                          // 更新（一步滤波）
```

- `InitAlgo_`：传入 `SAlgoKfInitParam`，完成维度校验与矩阵分配，失败返回 `APP_ERROR`。
- `UpdateHandler_`：执行一步卡尔曼滤波（预测 + 更新），结果存于 `Kf_Info.filtered_value`。

---

## 6. 扩展卡尔曼滤波

头文件：`robotpilots/algorithm/algo_ekf_filter.hpp`（纯头文件）

模板类 `CAlgo_Ekf<N>`：继承 `CAlgo_Kf<N>`，面向 IMU 姿态估计（四元数 EKF）。

### 回调函数类型

```cpp
using StateFunc = std::function<Matrixt<float, N>(
    Matrixt<float, N>&, Matrixt<float, N>&, float, Matrixt<float, N>&, Matrixt<float, N>&)>;
using MeasureFunc = std::function<Matrixt<float, N>(const Matrixt<float, N>&)>;
using PFunc = std::function<void(Matrixt<float, N>&)>;
using JacobianF = std::function<void(Matrixt<float, N>&, Matrixt<float, N>&, float, Matrixt<float, N>&)>;
using JacobianH = std::function<void(const Matrixt<float, N>&, Matrixt<float, N>&)>;
using Chi = std::function<bool(Matrixt<float, N>&, Matrixt<float, N>&, const Matrixt<float, N>&,
                               const Matrixt<float, N>&, const Matrixt<float, N>&, float)>;
using Xhat = std::function<void(Matrixt<float, N>&, const Matrixt<float, N>&, Matrixt<float, N>&, float)>;
using Kfunc = std::function<void(Matrixt<float, N>&)>;
```

### 初始化参数

```cpp
struct SAlgoEKfInitParam : public CAlgo_Kf<N>::SAlgoKfInitParam
{
    bool Chi_Set = false;   // 是否启用卡方检验
    StateFunc f;            // 状态转移非线性函数 F(x, u)
    MeasureFunc h;          // 观测非线性函数 H(x)
    JacobianF jacF;         // 状态雅可比
    JacobianH jacH;         // 观测雅可比
    PFunc p_func;           // 协方差更新函数
    Chi chi;                // 卡方检验函数
    Xhat Xhat_func;         // 状态估计函数
    Kfunc k_func;           // 增益更新函数
};
```

说明：`f`/`h`/`jacF`/`jacH`/`p_func`/`Xhat_func` 为必需回调；`Chi_Set` 为真时 `chi`/`k_func` 也必需。缺少必需回调时 `InitAlgo_` 返回 `APP_ERROR`。

### 输出信息

```cpp
struct SAlgoEKfInfo
{
    bool initialized = false;
    Matrixt<float, N> filtered_value;
} Ekf_Info;
```

### 成员函数

```cpp
EAppStatus InitAlgo_(CFilterBase::SFilterInitParam_Base &param) override;
EAppStatus UpdateHandler_() override;
```

- `InitAlgo_`：校验 `AlgoID == ALGO_IMU_EKF` 与必需回调，并复用 KF 初始化。
- `UpdateHandler_`：执行一步 EKF（预测 + 更新），结果存于 `Ekf_Info.filtered_value`。

---

## 7. 类型与枚举

### DataReceiver（algo_types.hpp）

```cpp
template<typename T> using DataReceiver = etl::span<const T>;
```

只读数据视图，不持有数据，用于接收数据进行计算。

### 算法 ID（conf_algo.hpp）

```cpp
enum class EAlgoID
{
    ALGO_NULL = -1,  // 空算法
    ALGO_KF = 1,     // 基本卡尔曼滤波
    ALGO_IMU_EKF,    // IMU 扩展卡尔曼滤波
};
```

### 应用状态（conf_algo.hpp）

```cpp
enum EAppStatus
{
    APP_RESET = 0, APP_OK, APP_ERROR, APP_BUSY,
    APP_TIMEOUT, APP_FULL, APP_EMPTY, APP_INVALID, APP_UNKNOWN
};
```

### 滤波器基类（algo_filter_common.hpp）

```cpp
class CFilterBase
{
protected:
    struct SFilterInitParam_Base { EAlgoID AlgoID = EAlgoID::ALGO_NULL; };
    virtual EAppStatus InitAlgo_(SFilterInitParam_Base &param) = 0;  // 纯虚
    virtual EAppStatus UpdateHandler_() = 0;                          // 纯虚
public:
    virtual ~CFilterBase() = default;
};
```

- 所有滤波器的抽象基类，`InitAlgo_` / `UpdateHandler_` 为纯虚函数，派生类必须实现。

### 公共入口（algorithm.hpp）

包含上述全部子模块，使用时只需 `#include "robotpilots/algorithm.hpp"`。

---

## 8. 使用示例

### CRC

```cpp
#include "robotpilots/algorithm.hpp"
using namespace robotpilots::algorithm;

uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
uint8_t crc = Crc8Calculate(data, sizeof(data));
bool ok = Crc8Verify(data, crc, sizeof(data));
```

### PID

```cpp
CAlgoPid pid;
CAlgoPid::SAlgoInitParam_Pid param;
param.kp = 1.0f;
param.ki = 0.1f;
param.kd = 0.01f;
param.tickRate = 1000;
pid.InitPID(&param);

float output = pid.UpdatePidController(target, measure);
```

### 卡尔曼滤波

```cpp
CAlgo_Kf<64> kf;   // N = 64：矩阵元素总数上限
CAlgo_Kf<64>::SAlgoKfInitParam param;
param.AlgoID = EAlgoID::ALGO_KF;
param.x_size = 4;   // 状态维
param.z_size = 2;   // 观测维
param.u_size = 0;   // 输入维
param.DT = 0.001f;
kf.InitAlgo_(param);

kf.Set_F(F);
kf.Set_H(H);
kf.Set_Q(Q);
kf.Set_R(R);
kf.Set_Measured_Vector(z);
kf.UpdateHandler_();

auto state = kf.Kf_Info.filtered_value;  // 滤波结果
```

### 扩展卡尔曼滤波

```cpp
CAlgo_Ekf<64> ekf;
CAlgo_Ekf<64>::SAlgoEKfInitParam param;
param.AlgoID = EAlgoID::ALGO_IMU_EKF;
param.x_size = 7;   // 四元数 4 + 陀螺仪零偏 3
param.z_size = 3;   // 加速度计
param.u_size = 3;   // 陀螺仪角速度
param.DT = 0.001f;
param.f = state_func;    // 状态转移非线性函数
param.h = measure_func;  // 观测非线性函数
param.jacF = jac_f;
param.jacH = jac_h;
param.p_func = p_func;
param.Xhat_func = xhat_func;
ekf.InitAlgo_(param);

ekf.Set_Measured_Vector(z);
ekf.UpdateHandler_();
auto state = ekf.Ekf_Info.filtered_value;
```
---
（注：本文档主要由ai编写，现在为试行文档；有问题去github提issue，看到就处理）
