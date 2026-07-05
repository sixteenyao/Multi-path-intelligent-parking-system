// =============================================================================
// tracker.cc — ByteTrack 多目标跟踪实现
//
// TODO Phase 3: 集成 ByteTrack (C++ 版)
//
// ByteTrack 仓库: https://github.com/ifzhang/ByteTrack
//
// 核心算法简述:
//   输入: 当前帧的检测框列表
//   输出: 每个框关联一个稳定的 track_id
//
//   第1步: 用 Kalman 滤波器预测每个已有轨迹的下一帧位置
//   第2步: 高置信度框(>0.5) 用 IoU 匹配已有轨迹
//   第3步: 低置信度框(0.1~0.5) 只和"第2步没匹配上的轨迹"匹配
//   第4步: 没匹配上的高置信度框 → 创建新轨迹
//   第5步: 连续丢失 N 帧的轨迹 → 删除
//
// ByteTrack vs DeepSORT:
//   DeepSORT 用 ReID 特征做外观匹配 → 精度高但慢
//   ByteTrack 只用 IoU 做运动匹配 → 速度快，适合嵌入式
// 面试时能说清楚这个权衡就是加分项
// =============================================================================

#include "tracker.h"
#include <iostream>

class Tracker::Impl {
public:
    int max_lost_frames = 30;
    float track_thresh = 0.5f;
    int next_track_id = 1;
    // std::vector<STrack> tracked_stracks;  ← ByteTrack 的轨迹列表
};

Tracker::~Tracker() = default;

bool Tracker::init(int max_lost_frames, float track_thresh) {
    impl_->max_lost_frames = max_lost_frames;
    impl_->track_thresh = track_thresh;
    std::cout << "[tracker] init max_lost=" << max_lost_frames
              << " thresh=" << track_thresh << std::endl;
    return true;
}

TrackResult Tracker::update(int camera_id, const std::vector<DetectBox>& detections, uint64_t timestamp_ms) {
    TrackResult result;
    result.camera_id = camera_id;
    result.timestamp_ms = timestamp_ms;

    // TODO: ByteTrack 匹配逻辑
    // std::vector<STrack> output = byte_track_.update(detections);

    return result;
}
