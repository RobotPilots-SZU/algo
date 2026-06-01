#!/bin/bash
set -e

# ── 工具检测 ──────────────────────────────────────────────

# 1. VSCode（code 命令）
if ! command -v code &>/dev/null; then
  echo "错误: 未检测到 VSCode（code 命令），请先安装 VSCode。" >&2
  exit 1
fi

# 2. direnv
if ! command -v direnv &>/dev/null; then
  read -p "未检测到 direnv，是否安装？[Y/n] " -r
  if [[ ! $REPLY =~ ^[Yy]?$ ]]; then
    echo "已取消。"
    exit 1
  fi

  if command -v dnf &>/dev/null; then
    sudo dnf install -y direnv
  elif command -v yum &>/dev/null; then
    sudo yum install -y direnv
  elif command -v apt-get &>/dev/null; then
    sudo apt-get install -y direnv
  elif command -v brew &>/dev/null; then
    brew install direnv
  else
    echo "错误: 无法自动安装 direnv，请手动安装后重试。" >&2
    echo "  https://direnv.net/docs/installation.html" >&2
    exit 1
  fi
fi

# 3. VSCode direnv 插件
if ! code --list-extensions 2>/dev/null | grep -qx 'mkhl.direnv'; then
  read -p "未检测到 VSCode direnv 插件，是否安装？[Y/n] " -r
  if [[ ! $REPLY =~ ^[Yy]?$ ]]; then
    echo "已取消。"
    exit 1
  fi

  # 优先走 VSCode 自带的市场安装
  if code --install-extension mkhl.direnv 2>/dev/null; then
    echo "插件安装成功。"
  else
    echo "插件市场安装失败，尝试 curl 离线安装..."
    read -p "是否使用 curl 下载并离线安装？[Y/n] " -r
    if [[ ! $REPLY =~ ^[Yy]?$ ]]; then
      echo "已取消。"
      exit 1
    fi

    VSIX_URL='https://marketplace.visualstudio.com/_apis/public/gallery/publishers/mkhl/vsextensions/direnv/latest/vspackage'
    VSIX_FILE="/tmp/mkhl.direnv-$$.vsix"
    curl -sL "$VSIX_URL" | gunzip > "$VSIX_FILE"
    code --install-extension "$VSIX_FILE"
    rm -f "$VSIX_FILE"
    echo "插件安装成功（curl 离线方式）。"
  fi
fi

# ── 构建虚拟环境 ──────────────────────────────────────────

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

# direnv 授权 .envrc
cd "$WORKSPACE"
direnv allow .

# .envrc 注入 CMAKE_PREFIX_PATH，cmake 零参数
direnv exec . cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
direnv exec . cmake --build build

echo ""
echo "虚拟环境: $WORKSPACE"
echo "  .envrc          — direnv 配置（CLI + VSCode 自动加载）"
echo "  CMakeLists.txt  — 顶层构建文件"
echo "  $REPO_NAME/     — 完整 Git 仓库（git clone）"
echo "  external/       — 外部依赖"
echo "  build/          — 中间产物"
echo "  bin/            — 库 & 可执行文件"
echo "  tests/          — 测试程序"
echo ""
echo "重新构建: cd $WORKSPACE && cmake --build build"
echo ""
echo "提示: 确保 shell 已 hook direnv，否则环境变量不会自动加载："
echo "  echo 'eval \"\$(direnv hook bash)\"' >> ~/.bashrc"
