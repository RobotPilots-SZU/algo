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

或通过 `CMAKE_PREFIX_PATH` 指向已安装的 ETL：

```bash
cmake -B build -DCMAKE_PREFIX_PATH=/path/to/etl
cmake --build build
```

## 开发环境

运行脚本一键构建隔离开发空间：

```bash
./scripts/dev-setup.sh [目标目录]
# 默认 ../algo-workspace
```

脚本自动完成：
- 从 GitHub 拉取 ETL 20.47.1
- 构建算法库
- 生成测试框架

构建产物可直接在 workspace 内重新构建：

```bash
cd algo-workspace
cmake -B build -DCMAKE_PREFIX_PATH=external
cmake --build build
```

在 `tests/` 目录下编写测试代码（`.cpp` 文件会被自动扫描编译），测试可执行文件输出到 `bin/`。

## 目录结构

```
algo/
├── CMakeLists.txt              # 构建配置
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
    ├── dev-setup.sh            # 开发环境构建入口
    └── workspace/              # workspace 模板（生成 algo-workspace/）
        └── ...

algo-workspace/                    # 脚本产出
├── CMakeLists.txt
├── algo/                          # 仓库副本
├── external/                      # ETL
├── build/                         # 中间产物
├── bin/                           # 库 & 可执行文件
└── tests/                         # 测试程序
```

## Git & GitHub 使用

### 克隆仓库

使用脚本构建开发环境需要配置ssh。

### 提交规范

```
feat: 新功能
fix: 修复 bug
docs: 文档更新
refactor: 重构
chore: 杂项（没事别用）
```
