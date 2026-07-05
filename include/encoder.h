// =============================================================================
// encoder.h — 视频编码 + RTMP 推流模块接口
//
// 职责: 把画了框的 RGB 帧 → MPP 硬编码 H.264 → RTMP 推流 / 存 MP4
//
// 为什么用 MPP 硬编码?
//   - RK3588 有硬件 H.264/H.265 编码器
//   - 8路1080p编码几乎不占CPU
//   - 功耗远低于软编码
//
// 输出:
//   RTMP: 推送到 nginx-rtmp / SRS / ZLMediaKit 等流媒体服务器
//   本地: 存为 MP4 分段文件 (如 00_20260702_150000.mp4)
// =============================================================================

#pragma once

#include <string>
#include <memory>
#include <opencv2/opencv.hpp>

class Encoder {
public:
    Encoder() = default;
    ~Encoder();

    // 初始化编码器
    // width/height: 输出分辨率
    // bitrate:      码率 (bps)，如 2000000 = 2Mbps
    // fps:          帧率
    bool init(int width, int height, int bitrate, int fps);

    // 设置 RTMP 推流地址
    void set_rtmp_url(const std::string& url);

    // 设置本地录像路径
    void set_record_path(const std::string& path);

    // 编码一帧并推送/存储
    // rgb_frame: 画好框的 BGR 图像
    bool encode_frame(const cv::Mat& rgb_frame, uint64_t timestamp_ms);

    // 停止编码（关闭文件/断开推流）
    void stop();

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
