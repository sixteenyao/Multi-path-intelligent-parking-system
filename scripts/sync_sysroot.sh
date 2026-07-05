#!/bin/bash
# =============================================================================
# 从 RK3588 开发板同步 sysroot（头文件 + 库文件）
# 交叉编译需要 ARM64 版本的 .so 和 .h，一次同步，之后增量更新
#
# 使用方法: ./scripts/sync_sysroot.sh root@192.168.131.86
# =============================================================================
set -e

BOARD=${1:-root@192.168.131.86}
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
SYSROOT="$PROJECT_DIR/sysroot"

echo "=== 同步 RK3588 sysroot ==="
echo "开发板: $BOARD"
echo "本地目录: $SYSROOT"

mkdir -p "$SYSROOT"/{lib,usr/lib,usr/include/rknn,usr/include/rockchip,usr/include/rga,usr/local/include}

# ----- 1. 运行时库（二进制链接用）-----
echo ""
echo "[1/4] 同步硬件加速库..."
rsync -avz --progress "$BOARD:/lib/librknnrt.so"      "$SYSROOT/lib/" 2>/dev/null || true
rsync -avz --progress "$BOARD:/usr/lib/librknn_api.so" "$SYSROOT/usr/lib/" 2>/dev/null || true
rsync -avz --progress "$BOARD:/usr/lib/librockchip_mpp.so" "$SYSROOT/usr/lib/" 2>/dev/null || true
rsync -avz --progress "$BOARD:/usr/lib/librockchip_mpp.so.1" "$SYSROOT/usr/lib/" 2>/dev/null || true
rsync -avz --progress "$BOARD:/usr/lib/librga.so"      "$SYSROOT/usr/lib/" 2>/dev/null || true
rsync -avz --progress "$BOARD:/usr/lib/librga.so.2"    "$SYSROOT/usr/lib/" 2>/dev/null || true

# ----- 2. 标准 C/C++ 库 -----
echo ""
echo "[2/4] 同步标准库..."
for lib in libc.so.6 libpthread.so.0 libdl.so.2 librt.so.1 libm.so.6 libstdc++.so.6; do
    rsync -avz "$BOARD:/lib/$lib" "$SYSROOT/lib/" 2>/dev/null || true
done

# 动态链接器（必须）
rsync -avz "$BOARD:/lib/ld-linux-aarch64.so.1" "$SYSROOT/lib/" 2>/dev/null || true

# ----- 3. OpenCV + FFmpeg + yaml-cpp（如果板子上有）-----
echo ""
echo "[3/4] 同步 OpenCV / FFmpeg / yaml-cpp..."
rsync -avz --include='*.so*' --include='*/' --exclude='*' \
    "$BOARD:/usr/lib/aarch64-linux-gnu/" "$SYSROOT/usr/lib/" 2>/dev/null || true

for lib in libopencv_core.so libopencv_imgproc.so libopencv_imgcodecs.so \
           libopencv_videoio.so libopencv_highgui.so \
           libavcodec.so libavformat.so libavutil.so libswscale.so \
           libyaml-cpp.so; do
    # 尝试找到这些库
    ssh "$BOARD" "find /usr/lib /lib -name '$lib*' -type f 2>/dev/null | head -3" 2>/dev/null | while read -r f; do
        rsync -avz "$BOARD:$f" "$SYSROOT/lib/" 2>/dev/null || true
    done
done

# ----- 4. 头文件 -----
echo ""
echo "[4/4] 同步头文件..."
rsync -avz "$BOARD:/usr/include/rknn/"     "$SYSROOT/usr/include/rknn/" 2>/dev/null || true
rsync -avz "$BOARD:/usr/include/rockchip/" "$SYSROOT/usr/include/rockchip/" 2>/dev/null || true
rsync -avz "$BOARD:/usr/include/rga/"      "$SYSROOT/usr/include/rga/" 2>/dev/null || true

echo ""
echo "=== sysroot 同步完成 ==="
echo "目录: $SYSROOT"
echo "文件数: $(find $SYSROOT -type f | wc -l)"
echo ""
echo "下一步: cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchain.rk3588.cmake"
