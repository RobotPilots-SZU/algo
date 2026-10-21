# robotpilots_algorithm

嵌入式机器人算法库，提供 CRC 校验、PID 控制、卡尔曼滤波、扩展卡尔曼滤波、矩阵运算等算法组件。

## 开发环境

通过 breeze-workspace 构建工作空间，在 breeze-app 下创建应用目录进行开发。
详情跳转 [breeze/README.md](https://github.com/RobotPilots-SZU/breeze/blob/main/README.md)

## 依赖

- Zephyr RTOS
- CMSIS-DSP（矩阵运算加速）
- ETL（嵌入式模板库，用于 span 等）
