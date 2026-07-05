#!/bin/bash
# 在 RK3588 上编译 smart_camera
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/build"

echo "=== 编译 smart_camera ==="
echo "项目路径: $PROJECT_DIR"

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

cmake "$PROJECT_DIR" \
    -DCMAKE_BUILD_TYPE=Release \

make -j$(nproc)

echo ""
echo "=== 编译完成 ==="
echo "运行: $BUILD_DIR/smart_camera --config $PROJECT_DIR/config/cameras.yaml"
