// =============================================================================
// tracker.h — ByteTrack 多目标跟踪模块接口
//
// 职责: 把逐帧检测结果关联成轨迹，给每个目标分配稳定ID
//
// ByteTrack 核心思想:
//   高置信度框: 直接匹配已有轨迹
//   低置信度框: 只和未匹配的轨迹尝试关联（防止新目标误匹配）
//
// 面试价值:
//   - 能讲清楚 MOT (Multi-Object Tracking) 的基本流程
//   - 能解释为什么用 ByteTrack 而不是 DeepSORT
//     (ByteTrack 不需要 ReID 特征，速度更快，适合嵌入式)
// =============================================================================

#pragma once

#include <vector>
#include <memory>
#include "detector.h"

struct Track {
    int track_id;          // 稳定ID（同一人不会变）
    DetectBox box;         // 当前帧的框
    int lost_frames;       // 连续丢失帧数
    bool is_active;        // 是否活跃（丢失太久就标记为失活）
};

struct TrackResult {
    int camera_id;
    uint64_t timestamp_ms;
    std::vector<Track> tracks;
};

class Tracker {
public:
    Tracker() = default;
    ~Tracker();

    // 初始化
    // max_lost_frames: 连续多少帧没检测到就认为目标消失（如30帧=2秒@15fps）
    // track_thresh:    只有置信度高于此值才创建新轨迹
    bool init(int max_lost_frames = 30, float track_thresh = 0.5);

    // 更新跟踪: 输入新一帧的检测结果，输出关联后的轨迹
    TrackResult update(int camera_id, const std::vector<DetectBox>& detections, uint64_t timestamp_ms);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
