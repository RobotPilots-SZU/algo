# robotpilots_algorithm

嵌入式机器人算法库，提供 CRC 校验、PID 控制、卡尔曼滤波、扩展卡尔曼滤波、矩阵运算等算法组件。

## 开发环境

通过 breeze-workspace 构建工作空间，在 breeze-app 下创建应用目录进行开发。
详情跳转 [breeze/README.md](https://github.com/RobotPilots-SZU/breeze/blob/main/README.md)

## 目录结构

```
algo/
├── CMakeLists.txt              # 库构建配置
├── README.md
├── include/
│   └── robotpilots/
│       ├── algorithm.hpp       # 公共入口
│       └── algorithm/
│           ├── algo_crc.hpp        # CRC8/CRC32
│           ├── algo_math.hpp       # 数学工具（纯头文件）
│           ├── algo_pid.hpp        # PID 控制器
│           ├── algo_types.hpp      # 类型定义
│           ├── conf_algo.hpp       # 算法配置与枚举
│           ├── algo_filter_common.hpp  # 滤波器基类
│           ├── rp_matrix.hpp       # 矩阵库（基于 CMSIS-DSP）
│           ├── algo_kf_filter.hpp  # 卡尔曼滤波
│           └── algo_ekf_filter.hpp # 扩展卡尔曼滤波
└── src/
    ├── algo_crc.cpp
    ├── algo_pid.cpp
    ├── algo_kf_filter.cpp
    └── algo_ekf_filter.cpp
```

## 依赖

- Zephyr RTOS
- CMSIS-DSP（矩阵运算加速）
- ETL（嵌入式模板库，用于 span 等）
