// =============================================================================
// detector.h — YOLO 检测模块接口
//
// 职责: 加载 RKNN 模型 → 推理 → 返回检测结果
//
// 核心设计:
//   1. 支持多实例（每个 NPU 核一个实例，互不干扰）
//   2. 输入 RGB 图，输出检测框列表
//   3. 内部封装了 rknn_init → load → init_runtime 的完整生命周期
//
// 关键优化点（Phase 3 区别）:
//   - 零拷贝: 使用 rknn_create_mem 让 NPU 直接访问 DDR 中的帧数据
//   - RGA 预处理: 用硬件做 resize + 格式转换，不占 CPU
//   - 核心绑定: 每个 detector 实例绑定一个 NPU 核心
// =============================================================================

#pragma once

#include <vector>
#include <string>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

struct DetectBox {
    float left;
    float top;
    float right;
    float bottom;
    float confidence;
    int class_id;
};

struct DetectResult {
    int camera_id;
    uint64_t timestamp_ms;
    std::vector<DetectBox> boxes;
};

class Detector {
public:
    Detector();
    ~Detector();

    // 初始化: 加载模型，指定使用哪个NPU核心 (0/1/2)
    // model_path: .rknn 模型路径
    // npu_core:   绑定到哪个NPU核心
    bool init(const std::string& model_path, int npu_core);

    // 推理一帧 (阻塞，返回耗时微秒)
    // 输入: RGB 图像 (640×640，这步预处理由调用者完成)
    // 输出: 检测框列表
    int64_t detect(const cv::Mat& rgb_frame, std::vector<DetectBox>& boxes);

    // 释放模型
    void release();

    // 获取模型信息
    int input_width()  const;
    int input_height() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
