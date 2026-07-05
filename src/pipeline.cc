// =============================================================================
// pipeline.cc — 多路调度核心实现
//
// TODO Phase 3: 实现完整的多路调度逻辑
//
// 核心设计:
//
//   cameras[8]  ──┬──► frame queue[0] ──► Detector(NPU core 0) ──┐
//                  ├──► frame queue[1] ──► Detector(NPU core 1) ──┤───► Tracker
//                  └──► frame queue[2] ──► Detector(NPU core 2) ──┘      │
//                                                                        ▼
//                                                                     Encoder
//                                                                     ├ RTMP
//                                                                     └ MP4
//
// NPU 核心分配策略 (静态分配):
//   - 8路摄像头 → 3个NPU核心
//   - 核心0: 路1,4,7  (帧率高的放前面)
//   - 核心1: 路2,5,8
//   - 核心2: 路3,6
//
// 动态帧率控制 (Phase 3 加分项):
//   - 连续N帧没有检测到人 → 采集帧率降到 5fps, 推理降到 1fps
//   - 有人出现 → 立即恢复到正常帧率
//   - 既能省NPU算力, 又能保证关键帧不丢
// =============================================================================

#include "pipeline.h"
#include <iostream>
#include <thread>
#include <chrono>

class Pipeline::Impl {
public:
    std::vector<CameraConfig> camera_configs;
    std::vector<std::unique_ptr<CameraInput>> cameras;
    std::vector<std::unique_ptr<Detector>> detectors;
    std::unique_ptr<Tracker> tracker;
    std::unique_ptr<Encoder> encoder;
    std::string model_path;
    std::string rtmp_url;
    std::string record_path;
    bool running = false;
};

Pipeline::~Pipeline() { stop(); }

bool Pipeline::load_config(const std::string& config_path) {
    std::cout << "[pipeline] loading config: " << config_path << std::endl;
    // TODO: 解析 YAML，填充 camera_configs
    return true;
}

bool Pipeline::run() {
    impl_->running = true;
    std::cout << "[pipeline] starting with " << impl_->camera_configs.size() << " cameras" << std::endl;

    // TODO 完整实现:
    //
    // 1. 创建 Detector 实例（绑定NPU核心）
    //    for (int core : {0, 1, 2}) {
    //        auto det = make_unique<Detector>();
    //        det->init(model_path, core);
    //        detectors.push_back(move(det));
    //    }
    //
    // 2. 创建 CameraInput 实例
    //    for (auto& cfg : camera_configs) {
    //        auto cam = make_unique<CameraInput>();
    //        cam->init(cfg);
    //        cameras.push_back(move(cam));
    //    }
    //
    // 3. 启动各路拉流
    //    for (int i = 0; i < cameras.size(); i++) {
    //        cameras[i]->start([this, i](auto& frame, auto ts) {
    //            // 分配 NPU 核心: i % 3
    //            int core = i % 3;
    //            auto& det = detectors[core];
    //            vector<DetectBox> boxes;
    //            det->detect(frame, boxes);
    //            auto tracks = tracker->update(camera_id, boxes, ts);
    //            // 画框 + 编码输出
    //        });
    //    }

    // 主循环: 等待停止信号
    while (impl_->running) {
        auto stats = get_stats();
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }

    return true;
}

void Pipeline::stop() {
    impl_->running = false;
    for (auto& cam : impl_->cameras) cam->stop();
    for (auto& det : impl_->detectors) det->release();
    if (impl_->encoder) impl_->encoder->stop();
    std::cout << "[pipeline] stopped" << std::endl;
}

Pipeline::Stats Pipeline::get_stats() const {
    Stats s{};
    // TODO: 统计 NPU 利用率、帧率、内存
    return s;
}
