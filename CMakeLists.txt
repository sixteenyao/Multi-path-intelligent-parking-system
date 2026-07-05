cmake_minimum_required(VERSION 3.14)
project(smart_camera VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =============================================================================
# 两种编译模式:
#   交叉编译 (x86 → RK3588):
#     cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchain.rk3588.cmake
#   板子本地编译 (RK3588 上):
#     cmake .. && make
# =============================================================================

# =============================================================================
# 源文件
# =============================================================================
set(SOURCES
    src/main.cc
    src/camera_input.cc
    src/detector.cc
    src/tracker.cc
    src/encoder.cc
    src/pipeline.cc
)

# =============================================================================
# 可执行文件
# =============================================================================
add_executable(smart_camera ${SOURCES})
add_executable(smart_camera_gui src/main_gui.cc src/detector.cc)

# =============================================================================
# 头文件路径
# =============================================================================
target_include_directories(smart_camera_gui PRIVATE
    ${CMAKE_SOURCE_DIR}/include/
    ${CMAKE_SYSROOT}/usr/include/rknn
    ${CMAKE_SYSROOT}/usr/include/freetype2
    /usr/include/drm
    /usr/include/libdrm
    /usr/include/freetype2
)

target_link_libraries(smart_camera_gui PRIVATE
    ${RKNN_API_LIB} ${RKNNRT_LIB} ${MPP_LIB} ${RGA_LIB}
    pthread dl atomic z drm freetype
    ${OpenCV_CORE_LIB} ${OpenCV_IMGPROC_LIB} ${OpenCV_VIDEOIO_LIB} ${OpenCV_IMGCODECS_LIB}
)
target_link_options(smart_camera_gui PRIVATE "LINKER:--allow-shlib-undefined")

target_compile_options(smart_camera_gui PRIVATE -Wall -O2)

target_include_directories(smart_camera PRIVATE
    ${CMAKE_SOURCE_DIR}/include/
    ${CMAKE_SYSROOT}/usr/include/rknn
    ${CMAKE_SYSROOT}/usr/include/rockchip
    ${CMAKE_SYSROOT}/usr/include/rga
    /usr/include/drm
    /usr/include/libdrm
)

# =============================================================================
# 查找并链接库
# =============================================================================

# --- RKNN (NPU 推理) ---
find_library(RKNN_API_LIB  NAMES rknn_api  PATHS ${CMAKE_SYSROOT}/usr/lib ${CMAKE_SYSROOT}/lib)
find_library(RKNNRT_LIB    NAMES rknnrt    PATHS ${CMAKE_SYSROOT}/usr/lib ${CMAKE_SYSROOT}/lib)

# --- MPP (硬件编解码) ---
find_library(MPP_LIB       NAMES rockchip_mpp PATHS ${CMAKE_SYSROOT}/usr/lib ${CMAKE_SYSROOT}/lib)

# --- RGA (硬件图像加速) ---
find_library(RGA_LIB       NAMES rga PATHS ${CMAKE_SYSROOT}/usr/lib ${CMAKE_SYSROOT}/lib)

# --- OpenCV (图像处理 / 拉流备选) ---
find_package(OpenCV QUIET)
if(NOT OpenCV_FOUND)
    find_library(OpenCV_CORE_LIB     NAMES opencv_core     PATHS ${CMAKE_SYSROOT}/usr/lib ${CMAKE_SYSROOT}/lib)
    find_library(OpenCV_IMGPROC_LIB  NAMES opencv_imgproc  PATHS ${CMAKE_SYSROOT}/usr/lib ${CMAKE_SYSROOT}/lib)
    find_library(OpenCV_IMGCODECS_LIB NAMES opencv_imgcodecs PATHS ${CMAKE_SYSROOT}/usr/lib ${CMAKE_SYSROOT}/lib)
    find_library(OpenCV_VIDEOIO_LIB  NAMES opencv_videoio  PATHS ${CMAKE_SYSROOT}/usr/lib ${CMAKE_SYSROOT}/lib)
    find_library(OpenCV_HIGHGUI_LIB  NAMES opencv_highgui  PATHS ${CMAKE_SYSROOT}/usr/lib ${CMAKE_SYSROOT}/lib)
    if(OpenCV_CORE_LIB AND OpenCV_IMGPROC_LIB)
        set(OpenCV_LIBS ${OpenCV_CORE_LIB} ${OpenCV_IMGPROC_LIB} ${OpenCV_IMGCODECS_LIB} ${OpenCV_VIDEOIO_LIB} ${OpenCV_HIGHGUI_LIB})
        set(OpenCV_FOUND TRUE)
    endif()
endif()

# --- FFmpeg (RTSP 拉流) ---
find_package(FFmpeg QUIET)
if(NOT FFmpeg_FOUND)
    find_library(AVCODEC_LIB  NAMES avcodec  PATHS ${CMAKE_SYSROOT}/usr/lib ${CMAKE_SYSROOT}/lib)
    find_library(AVFORMAT_LIB NAMES avformat PATHS ${CMAKE_SYSROOT}/usr/lib ${CMAKE_SYSROOT}/lib)
    find_library(AVUTIL_LIB   NAMES avutil   PATHS ${CMAKE_SYSROOT}/usr/lib ${CMAKE_SYSROOT}/lib)
    find_library(SWSCALE_LIB  NAMES swscale  PATHS ${CMAKE_SYSROOT}/usr/lib ${CMAKE_SYSROOT}/lib)
    if(AVCODEC_LIB AND AVFORMAT_LIB AND AVUTIL_LIB AND SWSCALE_LIB)
        set(FFmpeg_LIBRARIES ${AVCODEC_LIB} ${AVFORMAT_LIB} ${AVUTIL_LIB} ${SWSCALE_LIB})
        set(FFmpeg_FOUND TRUE)
    endif()
endif()

# --- yaml-cpp (配置文件解析) ---
find_package(yaml-cpp QUIET)
if(NOT yaml-cpp_FOUND)
    find_library(YAMLCPP_LIB NAMES yaml-cpp PATHS ${CMAKE_SYSROOT}/usr/lib ${CMAKE_SYSROOT}/lib)
    if(YAMLCPP_LIB)
        set(yaml-cpp_FOUND TRUE)
    endif()
endif()

# pthread 和 dl 是编译器内置的，不需要从 sysroot 找

# =============================================================================
# 链接
# =============================================================================
target_link_libraries(smart_camera PRIVATE
    ${RKNN_API_LIB}
    ${RKNNRT_LIB}
    ${MPP_LIB}
    ${RGA_LIB}
    pthread
    dl
    atomic
    z
    drm
)

# 板子 GLIBC 2.37 > 交叉编译器 2.31，放宽符号检查
# 所有符号在板子运行时都能正确解析
target_link_options(smart_camera PRIVATE
    "LINKER:--allow-shlib-undefined"
    "LINKER:--copy-dt-needed-entries"
)

# OpenCV
if(OpenCV_FOUND)
    target_link_libraries(smart_camera PRIVATE ${OpenCV_LIBS})
endif()

# FFmpeg
if(FFmpeg_FOUND)
    target_link_libraries(smart_camera PRIVATE ${FFmpeg_LIBRARIES})
endif()

# yaml-cpp
if(yaml-cpp_FOUND)
    target_link_libraries(smart_camera PRIVATE ${YAMLCPP_LIB})
endif()

# =============================================================================
# 编译选项
# =============================================================================
target_compile_options(smart_camera PRIVATE
    -Wall -Wextra
    -O2
    $<$<STREQUAL:${CMAKE_SYSTEM_PROCESSOR},aarch64>:-march=armv8.2-a>
)

# =============================================================================
# 编译后打印摘要
# =============================================================================
message(STATUS "===== smart_camera 编译配置 =====")
message(STATUS "系统:      ${CMAKE_SYSTEM_NAME} / ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "编译器:    ${CMAKE_CXX_COMPILER}")
message(STATUS "sysroot:   ${CMAKE_SYSROOT}")
message(STATUS "RKNN:      ${RKNN_API_LIB}")
message(STATUS "MPP:       ${MPP_LIB}")
message(STATUS "RGA:       ${RGA_LIB}")
message(STATUS "OpenCV:    ${OpenCV_FOUND}")
message(STATUS "FFmpeg:    ${FFmpeg_FOUND}")
message(STATUS "yaml-cpp:  ${yaml-cpp_FOUND}")
message(STATUS "=================================")
