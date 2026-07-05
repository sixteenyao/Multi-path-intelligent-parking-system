// =============================================================================
// camera_input.h — RTSP 拉流模块接口
//
// 职责: 从 RTSP 摄像头拉流 → MPP 硬解码 → 输出 RGB 帧
//
// 设计思路:
//   1. 用 FFmpeg 的 av_read_frame 拉 RTSP 流
//   2. 用 RK3588 的 MPP (Media Process Platform) 硬解码 H.264/H.265
//   3. 解码后的 NV12 帧通过 RGA 转成 RGB（在 detector 里做）
//
// 为什么用 MPP 硬解码而不是 OpenCV?
//   - RK3588 有硬件解码器，解码 8路1080p 时 CPU 几乎不占
//   - OpenCV 软解一路就吃掉一个 CPU 核
// =============================================================================

#pragma once

#include <string>
#include <functional>
#include <memory>
#include <opencv2/opencv.hpp>

struct CameraConfig {
    int id;
    std::string name;
    std::string rtsp_url;
    bool enabled;
    int width;
    int height;
    int fps;
    std::vector<int> detect_classes;  // 要检测的类别，空=全部
    float threshold;
};

class CameraInput {
public:
    // 回调类型: 每解码完一帧就调用
    using FrameCallback = std::function<void(int camera_id, const cv::Mat& rgb_frame, uint64_t timestamp_ms)>;

    CameraInput() = default;
    ~CameraInput();

    // 初始化一路摄像头
    bool init(const CameraConfig& config);

    // 开始拉流（非阻塞，内部启动解码线程）
    bool start(FrameCallback on_frame);

    // 停止拉流
    void stop();

    // 是否正在运行
    bool is_running() const;

    // 获取当前帧率统计
    float get_actual_fps() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
