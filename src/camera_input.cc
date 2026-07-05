// =============================================================================
// camera_input.cc — RTSP/视频文件拉流模块
// 使用 OpenCV VideoCapture (背后是 FFmpeg), 支持 RTSP 和本地文件
// =============================================================================
#include "camera_input.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <opencv2/videoio.hpp>

class CameraInput::Impl {
public:
    CameraConfig config;
    cv::VideoCapture cap;
    FrameCallback callback;
    std::thread worker;
    std::atomic<bool> running{false};
    int frame_count = 0;
};

CameraInput::~CameraInput() { stop(); }

bool CameraInput::init(const CameraConfig& config) {
    impl_->config = config;
    std::cout << "[cam " << config.id << "] init: " << config.name
              << " url=" << config.rtsp_url << std::endl;
    return true;
}

bool CameraInput::start(FrameCallback on_frame) {
    impl_->callback = on_frame;

    // 打开视频源 (RTSP地址 或 本地文件)
    if (!impl_->cap.open(impl_->config.rtsp_url)) {
        std::cerr << "[cam " << impl_->config.id << "] 打开失败: "
                  << impl_->config.rtsp_url << std::endl;
        return false;
    }
    impl_->cap.set(cv::CAP_PROP_FRAME_WIDTH,  impl_->config.width);
    impl_->cap.set(cv::CAP_PROP_FRAME_HEIGHT, impl_->config.height);
    impl_->cap.set(cv::CAP_PROP_FPS,          impl_->config.fps);

    impl_->running = true;

    // 启动拉流线程
    impl_->worker = std::thread([this]() {
        cv::Mat frame;
        int64_t frame_id = 0;
        while (impl_->running) {
            if (!impl_->cap.read(frame) || frame.empty()) {
                // 文件播完 或 连接断开 → 重试
                std::cerr << "[cam " << impl_->config.id << "] 读取失败, 重试..." << std::endl;
                impl_->cap.release();
                if (!impl_->cap.open(impl_->config.rtsp_url)) {
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    continue;
                }
                continue;
            }
            frame_id++;
            auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            if (impl_->callback)
                impl_->callback(impl_->config.id, frame, (uint64_t)now);
        }
    });

    std::cout << "[cam " << impl_->config.id << "] started" << std::endl;
    return true;
}

void CameraInput::stop() {
    impl_->running = false;
    if (impl_->worker.joinable()) impl_->worker.join();
    impl_->cap.release();
}

bool CameraInput::is_running() const { return impl_->running; }
float CameraInput::get_actual_fps() const { return 0; }
