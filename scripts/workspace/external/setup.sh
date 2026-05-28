#!/bin/bash
set -e
git clone --depth 1 --branch 20.47.1 https://github.com/ETLCPP/etl.git "$WORKSPACE/etl-src"
cmake -B "$WORKSPACE/etl-src/build" -S "$WORKSPACE/etl-src" \
  -DCMAKE_INSTALL_PREFIX="$WORKSPACE/external"
cmake --install "$WORKSPACE/etl-src/build"
rm -rf "$WORKSPACE/etl-src"
