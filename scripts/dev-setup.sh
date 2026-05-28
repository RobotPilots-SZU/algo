#!/bin/bash
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_NAME="$(basename "$REPO_ROOT")"
WORKSPACE="${1:-$(dirname "$REPO_ROOT")/algo-workspace}"

if [ -d "$WORKSPACE" ]; then
  read -p "Workspace $WORKSPACE already exists. Remove and rebuild? [y/N] " -r
  if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Aborted."
    exit 0
  fi
  rm -rf "$WORKSPACE"
fi

cp -r "$SCRIPT_DIR/workspace" "$WORKSPACE"

export REPO_ROOT REPO_NAME WORKSPACE

"$WORKSPACE/external/setup.sh"
"$WORKSPACE/algo/setup.sh"

# 清理环境构建脚本，只保留仓库自带的
rm -f "$WORKSPACE/external/setup.sh"
rm -f "$WORKSPACE/tests/setup.sh"
rm -f "$WORKSPACE/algo/setup.sh"

cmake -B "$WORKSPACE/build" -S "$WORKSPACE" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$WORKSPACE/external"
cmake --build "$WORKSPACE/build"

echo ""
echo "虚拟环境: $WORKSPACE"
echo "  CMakeLists.txt  — 顶层构建文件"
echo "  $REPO_NAME/     — 完整 Git 仓库（git clone）"
echo "  external/       — 外部依赖"
echo "  build/          — 中间产物"
echo "  bin/            — 库 & 可执行文件"
echo "  tests/          — 测试程序"
echo ""
echo "重新构建: cmake -B build -DCMAKE_PREFIX_PATH=external && cmake --build build"
