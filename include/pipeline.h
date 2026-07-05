// =============================================================================
// pipeline.h — 多路调度核心
//
// 职责:
//   1. 管理多路摄像头 → NPU 核心的映射关系
//   2. 把 camera → detect → track → encode 串成管道
//   3. 动态帧率控制（画面没动静降帧率，有人出现提帧率）
//
// 调度策略:
//   静态分配: 8路 ÷ 3核 = 每核2-3路（简单但可能不均衡）
//   动态轮转: 3核轮流转发处理，每核处理所有路（复杂但均衡）
//
// Phase 3 先用静态分配，跑通以后改为轮转（面试加分项）
// =============================================================================

#pragma once

#include <vector>
#include <memory>
#include "camera_input.h"
#include "detector.h"
#include "tracker.h"
#include "encoder.h"

class Pipeline {
public:
    Pipeline() = default;
    ~Pipeline();

    // 从 YAML 配置文件加载
    bool load_config(const std::string& config_path);

    // 启动管道（阻塞，直到收到停止信号）
    bool run();

    // 停止管道
    void stop();

    // 状态查询
    struct Stats {
        int total_cameras;
        int active_cameras;
        float overall_fps;
        float npu_usage[3];     // 3个NPU核心各自的利用率
        float cpu_usage;
        float memory_mb;
    };
    Stats get_stats() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
