// =============================================================================
// llm_engine.h — 端侧 LLM/VLM 推理模块接口
//
// 职责:
//   1. 场景描述: 检测到事件 → 裁剪图 + prompt → LLM 生成自然语言描述
//   2. 自然语言查询: "今天谁来过" → 查事件日志 → LLM 总结 → 返回自然语言
//
// 为什么在 RK3588 上能跑 LLM?
//   - Rockchip RKLLM 工具链: 把 LLM 编译成 .rkllm 格式，跑在 NPU 上
//   - Qwen2.5-0.5B (0.5B 参数): 仅需 ~350MB 内存，NPU 推理 ~200-500ms
//   - Qwen2.5-1.5B (1.5B 参数): ~1GB 内存，NPU 推理 ~500ms-1s
//
// RKLLM 和 RKNN 的区别:
//   RKNN:  跑 YOLO 这种 CNN 模型（卷积、池化、全连接）
//   RKLLM: 跑 LLM 这种 Transformer 模型（注意力机制、自回归生成）
//   两者可以用同一块 NPU，但不能同时跑（需要分时复用）
//
// 面试加分: 能讲清楚 NPU 上如何分时复用来跑两个不同类别的模型
// =============================================================================

#pragma once

#include <string>
#include <vector>
#include <functional>
#include <memory>
#include "detector.h"

// 事件定义：值得 LLM 关注的事情
struct Event {
    uint64_t timestamp_ms;
    int camera_id;
    std::string camera_name;
    std::string event_type;         // "person_appeared", "package_detected", "lingering", etc.
    int track_id;
    DetectBox box;
    std::string thumbnail_path;     // 裁剪图的存储路径（给 VLM 看）
};

// LLM 描述结果
struct LLMDescription {
    uint64_t timestamp_ms;
    std::string short_text;         // 一句话: "快递员把包裹放在门口"
    std::string detail_text;        // 详细描述: "一名穿着...的快递员..."
    float confidence;
};

class LLMEngine {
public:
    LLMEngine() = default;
    ~LLMEngine();

    // =========================================================================
    // 初始化 LLM
    // model_path:  .rkllm 模型路径（由 RKLLM 工具链转换）
    // mode:        "vision" (VLM, 输入图片+文本) 或 "text" (纯文本)
    //
    // RK3588 上:
    //   VLM 模式跑 Qwen2.5-VL-2B 的 RKLLM 版本
    //   纯文本模式跑 Qwen2.5-0.5B 或 TinyLlama
    // =========================================================================
    bool init(const std::string& model_path, const std::string& mode = "vision");

    // =========================================================================
    // 场景描述（VLM 模式）: 图片 + 提示词 → 自然语言描述
    //
    // 输入: 裁剪后的事件帧 (roi_image) + 提示词
    // 提示词示例:
    //   "描述图中的人在做什么，穿着什么颜色的衣服，是否携带物品"
    // 输出: "一个穿红色外套的人站在门口，手里拿着一个棕色包裹"
    //
    // 耗时: 约 200-500ms (Qwen2.5-0.5B) 或 500ms-1s (1.5B)
    // =========================================================================
    LLMDescription describe_event(const cv::Mat& roi_image, const std::string& prompt);

    // =========================================================================
    // 自然语言查询（纯文本模式）: 用户问题 → LLM 总结事件日志
    //
    // 输入: "今天下午有几个人来过？"
    // 内部: 把今天的事件日志拼接成 context + 问题 → LLM
    // 输出: "今天下午共有3人来访: 1名快递员于1:23到达..."
    // =========================================================================
    std::string answer_query(const std::string& question, const std::vector<Event>& recent_events);

    // =========================================================================
    // 判断是否需要触发 LLM（避免每帧都跑）
    //
    // 触发条件:
    //   1. 新的 track_id 出现（新人/新车）
    //   2. 目标进入/离开 ROI 区域
    //   3. 目标停留超过阈值时间（如30秒）
    //   4. 检测到特定物品（包裹）
    //
    // 不触发:
    //   - 同一目标持续存在
    //   - 低置信度检测
    // =========================================================================
    bool should_trigger(const std::vector<Track>& active_tracks,
                        const std::vector<Track>& previous_tracks);

    // 释放模型
    void release();

    // 获取模型信息
    std::string model_name() const;
    float avg_inference_time_ms() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
