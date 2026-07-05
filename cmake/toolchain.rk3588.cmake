# =============================================================================
# RK3588 交叉编译工具链文件
# 使用方法: cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchain.rk3588.cmake
# =============================================================================

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# ----- 交叉编译器 -----
set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

set(CMAKE_AR      aarch64-linux-gnu-ar)
set(CMAKE_LINKER  aarch64-linux-gnu-ld)
set(CMAKE_STRIP   aarch64-linux-gnu-strip)
set(CMAKE_OBJCOPY aarch64-linux-gnu-objcopy)

# ----- SDK sysroot（从开发板同步）-----
# 执行 scripts/sync_sysroot.sh 将 RK3588 上的库拉取到本地
set(CMAKE_SYSROOT ${CMAKE_CURRENT_LIST_DIR}/../sysroot)
set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})

# 只在 sysroot 里找库和头文件，不找 host 系统
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# ----- pkg-config 路径 -----
set(PKG_CONFIG_EXECUTABLE aarch64-linux-gnu-pkg-config)
set(ENV{PKG_CONFIG_PATH} "${CMAKE_SYSROOT}/usr/lib/pkgconfig:${CMAKE_SYSROOT}/usr/share/pkgconfig")
set(ENV{PKG_CONFIG_SYSROOT_DIR} "${CMAKE_SYSROOT}")
