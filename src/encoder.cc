// =============================================================================
// encoder.cc — MPP 编码 + RTMP 推流实现
//
// TODO Phase 3: MPP 硬编码 + RTMP 推流
//
// MPP (Media Process Platform):
//   Rockchip 的硬件编解码框架
//   API 参考: https://github.com/rockchip-linux/mpp
//
// 编码流程:
//   1. mpp_create / mpp_init  → 创建 MPP 实例
//   2. mpp_start              → 开始编码
//   3. 循环: mpp_frame_get → 填充 YUV 数据 → mpp_encode → mpp_packet_get → 拿 H.264
//   4. mpp_stop → mpp_destroy
//
// RTMP 推流:
//   方式1: H.264 裸流 → ffmpeg libavformat → RTMP
//   方式2: MPP 输出 → 直接通过 librtmp 发送
//   推荐方式1，ffmpeg 封装简单可靠
//
// 本地录像:
//   每5分钟切一个 MP4 文件: 03_20260702_150000.mp4
//   超过 retention_days 天自动删除
// =============================================================================

#include "encoder.h"
#include <iostream>

class Encoder::Impl {
public:
    int width = 1920;
    int height = 1080;
    int bitrate = 2000000;
    int fps = 15;
    std::string rtmp_url;
    std::string record_path;
    bool running = false;
    // MppCtx mpp_ctx;  ← MPP 上下文
};

Encoder::~Encoder() { stop(); }

bool Encoder::init(int width, int height, int bitrate, int fps) {
    impl_->width = width;
    impl_->height = height;
    impl_->bitrate = bitrate;
    impl_->fps = fps;
    std::cout << "[encoder] init " << width << "x" << height
              << " @" << bitrate/1000000.0 << "Mbps" << std::endl;
    // TODO: MPP 编码器初始化
    impl_->running = true;
    return true;
}

void Encoder::set_rtmp_url(const std::string& url) {
    impl_->rtmp_url = url;
    std::cout << "[encoder] rtmp: " << url << std::endl;
}

void Encoder::set_record_path(const std::string& path) {
    impl_->record_path = path;
    std::cout << "[encoder] record: " << path << std::endl;
}

bool Encoder::encode_frame(const cv::Mat& rgb_frame, uint64_t timestamp_ms) {
    // TODO: BGR → NV12 → MPP encode → H.264 packet → RTMP push + MP4 write
    return true;
}

void Encoder::stop() {
    impl_->running = false;
    // TODO: 关闭 MPP、关闭文件、断开 RTMP
}
