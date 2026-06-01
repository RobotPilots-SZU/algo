# robotpilots_algorithm

嵌入式机器人算法库，提供 CRC 校验、PID 控制、数学工具等常用算法组件。

## 生产使用

将本仓库作为 CMake 子项目引入：

```cmake
# 1. 确保已安装 ETL（Embedded Template Library）
find_package(etl REQUIRED)

# 2. 引入本库
add_subdirectory(path/to/algo)
target_link_libraries(your_target PRIVATE robotpilots_algorithm)
```

## 开发环境

### 一键构建

```bash
./scripts/dev-setup.sh [目标目录]
# 默认 ../algo-workspace
```

脚本自动完成：

- 检测并安装 direnv（支持 dnf / yum / apt-get / brew）
- 检测并安装 VSCode direnv 插件（优先市场直装，失败回退 curl 离线安装）
- 从 GitHub 拉取 ETL 20.47.1 并编译安装到工作区
- 复制仓库副本到工作区
- 通过 direnv + `.envrc` 注入 `CMAKE_PREFIX_PATH`，cmake 零参数构建

**依赖**：VSCode（必须）、git、cmake、curl、gunzip。

### 虚拟环境

工作区通过 direnv 实现环境隔离——无需传 `-D` 参数，`find_package(etl)` 像系统库一样透明生效：

```bash
cd algo-workspace
cmake -B build           # CMAKE_PREFIX_PATH 由 .envrc 自动注入
cmake --build build
```

在 `tests/` 目录下编写测试代码（`.cpp` 文件会被自动扫描编译），测试可执行文件输出到 `bin/`。

> 确保 shell 已 hook direnv：`echo 'eval "$(direnv hook bash)"' >> ~/.bashrc`

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
├── src/
│   ├── algo_crc.cpp
│   └── algo_pid.cpp
└── scripts/
    ├── dev-setup.sh            # 开发环境一键构建
    └── workspace/              # workspace 模板
        ├── .envrc              # direnv 环境配置
        ├── CMakeLists.txt      # workspace 顶层构建
        ├── algo/setup.sh       # 仓库副本初始化
        ├── external/setup.sh   # ETL 依赖安装
        └── tests/
            ├── CMakeLists.txt  # 测试自动扫描
            └── setup.sh        # 测试初始化（占位）

algo-workspace/                 # 脚本产出（direnv 环境）
├── .envrc                      # export CMAKE_PREFIX_PATH=$PWD/external
├── CMakeLists.txt              # 顶层构建
├── algo/                       # 仓库副本
├── external/                   # ETL 安装目录
├── build/                      # 中间产物
├── bin/                        # 库 & 可执行文件
└── tests/                      # 测试程序
```

## Git 提交规范

```
feat:     新功能
fix:      修复 bug
docs:     文档更新
refactor: 重构
chore:    杂项
```
