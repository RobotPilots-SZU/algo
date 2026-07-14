# robotpilots_algorithm

嵌入式机器人算法库，提供 CRC 校验、PID 控制、数学工具等常用算法组件。

## 开发环境

通过breeze-workspace构建工作空间，在breeze-app下创建应用目录进行开发。
详情跳转[breeze/README.md](https://github.com/RobotPilots-SZU/breeze/blob/main/README.md)

## 目录结构

```
algo/
├── CMakeLists.txt              # 库构建配置
├── README.md
├── include/
│   └── robotpilots/
│       ├── algorithm.hpp       # 公共入口
│       └── algorithm/
│           ├── algo_crc.hpp    # CRC8/CRC16
│           ├── algo_math.hpp   # 数学工具（纯头文件）
│           ├── algo_pid.hpp    # PID 控制器
│           └── algo_types.hpp  # 类型定义
└── src/
     ├── algo_crc.cpp
     └── algo_pid.cpp


```
