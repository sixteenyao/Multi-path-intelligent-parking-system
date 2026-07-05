// =============================================================================
// detector.cc — YOLO 检测模块实现
// 从阶段2 object_detection_tracking_rk3588 的 yolov8.cc + postprocess.cc 适配而来
// =============================================================================

#include "detector.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <cmath>
#include <cstring>
#include <algorithm>

// Rockchip RKNN C API
#include "rknn_api.h"

// =============================================================================
// 常量
// =============================================================================
#define OBJ_CLASS_NUM      80
#define OBJ_NUMB_MAX_SIZE  128
#define NMS_THRESH         0.45f
#define BOX_THRESH         0.25f

// =============================================================================
// 标签
// =============================================================================
static const char* const LABELS[OBJ_CLASS_NUM] = {
    "person","bicycle","car","motorbike","aeroplane","bus","train","truck","boat",
    "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat",
    "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack",
    "umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball",
    "kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket",
    "bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple",
    "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair",
    "sofa","pottedplant","bed","diningtable","toilet","tvmonitor","laptop","mouse",
    "remote","keyboard","cell phone","microwave","oven","toaster","sink",
    "refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"
};

// =============================================================================
// 工具函数: 读文件到内存
// =============================================================================
static unsigned char* read_file(const char* path, int* out_len) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) { std::cerr << "open model failed: " << path << std::endl; return nullptr; }
    *out_len = file.tellg();
    file.seekg(0);
    auto* data = (unsigned char*)malloc(*out_len);
    file.read((char*)data, *out_len);
    return data;
}

// =============================================================================
// 量化/反量化
// =============================================================================
static inline int32_t clip(float val, float min, float max) {
    float f = val <= min ? min : (val >= max ? max : val);
    return (int32_t)f;
}
static int8_t qnt_f32_to_affine(float f32, int32_t zp, float scale) {
    return (int8_t)clip((f32 / scale) + zp, -128, 127);
}
static float deqnt_affine_to_f32(int8_t qnt, int32_t zp, float scale) {
    return ((float)qnt - (float)zp) * scale;
}

// =============================================================================
// DFL 解码
// =============================================================================
static void compute_dfl(float* tensor, int dfl_len, float* box) {
    for (int b = 0; b < 4; b++) {
        float exp_t[16], exp_sum = 0, acc_sum = 0;
        for (int i = 0; i < dfl_len; i++) {
            exp_t[i] = exp(tensor[i + b * dfl_len]);
            exp_sum += exp_t[i];
        }
        for (int i = 0; i < dfl_len; i++) acc_sum += exp_t[i] / exp_sum * i;
        box[b] = acc_sum;
    }
}

// =============================================================================
// IoU 和 NMS
// =============================================================================
static float iou(float xmin0, float ymin0, float xmax0, float ymax0,
                 float xmin1, float ymin1, float xmax1, float ymax1) {
    float w = fmax(0.f, fmin(xmax0, xmax1) - fmax(xmin0, xmin1) + 1.0f);
    float h = fmax(0.f, fmin(ymax0, ymax1) - fmax(ymin0, ymin1) + 1.0f);
    float i = w * h;
    float u = (xmax0 - xmin0 + 1.f) * (ymax0 - ymin0 + 1.f)
            + (xmax1 - xmin1 + 1.f) * (ymax1 - ymin1 + 1.f) - i;
    return u <= 0.f ? 0.f : (i / u);
}

static void nms(int validCount, std::vector<float>& boxes, std::vector<int>& classIds,
                std::vector<int>& order, int filterId, float threshold) {
    for (int i = 0; i < validCount; ++i) {
        int n = order[i];
        if (n == -1 || classIds[n] != filterId) continue;
        for (int j = i + 1; j < validCount; ++j) {
            int m = order[j];
            if (m == -1 || classIds[m] != filterId) continue;
            float ovr = iou(boxes[n*4+0], boxes[n*4+1], boxes[n*4+0]+boxes[n*4+2], boxes[n*4+1]+boxes[n*4+3],
                            boxes[m*4+0], boxes[m*4+1], boxes[m*4+0]+boxes[m*4+2], boxes[m*4+1]+boxes[m*4+3]);
            if (ovr > threshold) order[j] = -1;
        }
    }
}

// =============================================================================
// INT8 网格解码
// =============================================================================
static int process_int8(int8_t* box_tensor, int32_t box_zp, float box_scale,
                        int8_t* score_tensor, int32_t score_zp, float score_scale,
                        int8_t* score_sum_tensor, int32_t score_sum_zp, float score_sum_scale,
                        int grid_h, int grid_w, int stride, int dfl_len,
                        std::vector<float>& boxes, std::vector<float>& objProbs,
                        std::vector<int>& classId, float threshold) {
    int validCount = 0;
    int grid_len = grid_h * grid_w;
    int8_t score_thres = qnt_f32_to_affine(threshold, score_zp, score_scale);
    int8_t score_sum_thres = qnt_f32_to_affine(threshold, score_sum_zp, score_sum_scale);

    for (int i = 0; i < grid_h; i++) {
        for (int j = 0; j < grid_w; j++) {
            int offset = i * grid_w + j;
            int max_class_id = -1;

            if (score_sum_tensor && score_sum_tensor[offset] < score_sum_thres) continue;

            int8_t max_score = -score_zp;
            for (int c = 0; c < OBJ_CLASS_NUM; c++) {
                if (score_tensor[offset] > score_thres && score_tensor[offset] > max_score) {
                    max_score = score_tensor[offset];
                    max_class_id = c;
                }
                offset += grid_len;
            }

            if (max_score > score_thres) {
                offset = i * grid_w + j;
                float box[4], before_dfl[16*4];
                for (int k = 0; k < dfl_len * 4; k++) {
                    before_dfl[k] = deqnt_affine_to_f32(box_tensor[offset], box_zp, box_scale);
                    offset += grid_len;
                }
                compute_dfl(before_dfl, dfl_len, box);
                boxes.push_back((-box[0] + j + 0.5f) * stride);
                boxes.push_back((-box[1] + i + 0.5f) * stride);
                boxes.push_back((box[2] + box[0]) * stride);
                boxes.push_back((box[3] + box[1]) * stride);
                objProbs.push_back(deqnt_affine_to_f32(max_score, score_zp, score_scale));
                classId.push_back(max_class_id);
                validCount++;
            }
        }
    }
    return validCount;
}

// =============================================================================
// 快排(降序)
// =============================================================================
static void qsort_desc(std::vector<float>& input, int left, int right, std::vector<int>& indices) {
    if (left >= right) return;
    float key = input[left];
    int key_idx = indices[left], low = left, high = right;
    while (low < high) {
        while (low < high && input[high] <= key) high--;
        input[low] = input[high]; indices[low] = indices[high];
        while (low < high && input[low] >= key) low++;
        input[high] = input[low]; indices[high] = indices[low];
    }
    input[low] = key; indices[low] = key_idx;
    qsort_desc(input, left, low - 1, indices);
    qsort_desc(input, low + 1, right, indices);
}

// =============================================================================
// PIMPL 实现
// =============================================================================
class Detector::Impl {
public:
    rknn_context ctx = 0;
    rknn_input_output_num io_num;
    rknn_tensor_attr* input_attrs = nullptr;
    rknn_tensor_attr* output_attrs = nullptr;
    int model_width = 640;
    int model_height = 640;
    int model_channel = 3;
    bool is_quant = false;
    int npu_core = 0;
};

// =============================================================================
// 构造/析构
// =============================================================================
Detector::Detector() : impl_(std::make_unique<Impl>()) {}
Detector::~Detector() { release(); }

// =============================================================================
// 初始化: 加载.rknn → 查询输入输出 → 绑定NPU核心
// =============================================================================
bool Detector::init(const std::string& model_path, int npu_core) {
    impl_->npu_core = npu_core;

    // 1. 加载模型文件
    int model_len = 0;
    unsigned char* model_data = read_file(model_path.c_str(), &model_len);
    if (!model_data) return false;

    // 2. 初始化 RKNN (可以指定核心)
    rknn_context ctx = 0;
    int ret = rknn_init(&ctx, model_data, model_len, 0, nullptr);
    free(model_data);
    if (ret < 0) { std::cerr << "[detector] rknn_init fail: " << ret << std::endl; return false; }
    impl_->ctx = ctx;

    // 3. 查询输入输出数量
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &impl_->io_num, sizeof(impl_->io_num));
    if (ret < 0) { std::cerr << "[detector] query io_num fail" << std::endl; return false; }

    // 4. 查询输入属性
    impl_->input_attrs = (rknn_tensor_attr*)malloc(impl_->io_num.n_input * sizeof(rknn_tensor_attr));
    memset(impl_->input_attrs, 0, impl_->io_num.n_input * sizeof(rknn_tensor_attr));
    for (int i = 0; i < impl_->io_num.n_input; i++) {
        impl_->input_attrs[i].index = i;
        rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &impl_->input_attrs[i], sizeof(rknn_tensor_attr));
    }

    // 5. 查询输出属性
    impl_->output_attrs = (rknn_tensor_attr*)malloc(impl_->io_num.n_output * sizeof(rknn_tensor_attr));
    memset(impl_->output_attrs, 0, impl_->io_num.n_output * sizeof(rknn_tensor_attr));
    for (int i = 0; i < impl_->io_num.n_output; i++) {
        impl_->output_attrs[i].index = i;
        rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &impl_->output_attrs[i], sizeof(rknn_tensor_attr));
    }

    // 6. 判断量化类型 + 获取模型尺寸
    impl_->is_quant = (impl_->output_attrs[0].qnt_type == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC
                    && impl_->output_attrs[0].type == RKNN_TENSOR_INT8);

    if (impl_->input_attrs[0].fmt == RKNN_TENSOR_NCHW) {
        impl_->model_channel = impl_->input_attrs[0].dims[1];
        impl_->model_height  = impl_->input_attrs[0].dims[2];
        impl_->model_width   = impl_->input_attrs[0].dims[3];
    } else {
        impl_->model_height  = impl_->input_attrs[0].dims[1];
        impl_->model_width   = impl_->input_attrs[0].dims[2];
        impl_->model_channel = impl_->input_attrs[0].dims[3];
    }

    std::cout << "[detector] init OK core=" << npu_core
              << " size=" << impl_->model_width << "x" << impl_->model_height
              << " quant=" << (impl_->is_quant ? "INT8" : "FP16")
              << " in=" << impl_->io_num.n_input << " out=" << impl_->io_num.n_output << std::endl;
    return true;
}

// =============================================================================
// 推理一帧
// =============================================================================
int64_t Detector::detect(const cv::Mat& rgb_frame, std::vector<DetectBox>& boxes) {
    boxes.clear();
    if (!impl_->ctx) return -1;

    int mw = impl_->model_width, mh = impl_->model_height, mc = impl_->model_channel;

    // --- 预处理: Resize + LetterBox + BGR→RGB ---
    int img_w = rgb_frame.cols, img_h = rgb_frame.rows;
    float scale = std::min((float)mw / (float)img_w, (float)mh / (float)img_h);
    int new_w = std::max(1, (int)((float)img_w * scale));
    int new_h = std::max(1, (int)((float)img_h * scale));
    int pad_left = (mw - new_w) / 2, pad_top = (mh - new_h) / 2;

    cv::Mat letterbox(mh, mw, CV_8UC3, cv::Scalar(0,0,0));
    // 手写最近邻缩放 (避免 OpenCV resize bug)
    for (int y = 0; y < new_h; y++) {
        int src_y = y * img_h / new_h;
        unsigned char* dst = letterbox.ptr(y + pad_top) + pad_left * 3;
        const unsigned char* src = rgb_frame.ptr(src_y);
        for (int x = 0; x < new_w; x++) {
            int src_x = x * img_w / new_w;
            memcpy(dst + x*3, src + src_x*3, 3);
        }
    }
    if (mc == 3) cv::cvtColor(letterbox, letterbox, cv::COLOR_BGR2RGB);

    // --- 设置输入 ---
    rknn_input inputs[1] = {};
    inputs[0].index = 0;
    inputs[0].type  = RKNN_TENSOR_UINT8;
    inputs[0].fmt   = RKNN_TENSOR_NHWC;
    inputs[0].size  = mw * mh * mc;
    inputs[0].buf   = letterbox.data;

    int ret = rknn_inputs_set(impl_->ctx, 1, inputs);
    if (ret < 0) return -1;

    // --- 推理 ---
    ret = rknn_run(impl_->ctx, nullptr);
    if (ret < 0) return -1;

    // --- 获取输出 ---
    rknn_output outputs[impl_->io_num.n_output];
    memset(outputs, 0, sizeof(outputs));
    for (int i = 0; i < impl_->io_num.n_output; i++) {
        outputs[i].index = i;
        outputs[i].want_float = (!impl_->is_quant);
    }
    ret = rknn_outputs_get(impl_->ctx, impl_->io_num.n_output, outputs, nullptr);
    if (ret < 0) return -1;

    // --- 后处理 ---
    std::vector<float> filterBoxes, objProbs;
    std::vector<int> classId;
    int validCount = 0;
    int dfl_len = impl_->output_attrs[0].dims[1] / 4;
    int output_per_branch = impl_->io_num.n_output / 3;

    for (int i = 0; i < 3; i++) {
        int box_idx = i * output_per_branch;
        int score_idx = i * output_per_branch + 1;
        int8_t* score_sum = nullptr;
        int32_t score_sum_zp = 0;
        float score_sum_scale = 1.0f;
        if (output_per_branch == 3) {
            score_sum = (int8_t*)outputs[i * output_per_branch + 2].buf;
            score_sum_zp = impl_->output_attrs[i * output_per_branch + 2].zp;
            score_sum_scale = impl_->output_attrs[i * output_per_branch + 2].scale;
        }
        int grid_h = impl_->output_attrs[box_idx].dims[2];
        int grid_w = impl_->output_attrs[box_idx].dims[3];
        int stride = mh / grid_h;

        if (impl_->is_quant) {
            validCount += process_int8(
                (int8_t*)outputs[box_idx].buf, impl_->output_attrs[box_idx].zp, impl_->output_attrs[box_idx].scale,
                (int8_t*)outputs[score_idx].buf, impl_->output_attrs[score_idx].zp, impl_->output_attrs[score_idx].scale,
                score_sum, score_sum_zp, score_sum_scale,
                grid_h, grid_w, stride, dfl_len, filterBoxes, objProbs, classId, BOX_THRESH);
        }
    }

    if (validCount > 0) {
        // 排序
        std::vector<int> indexArray(validCount);
        for (int i = 0; i < validCount; i++) indexArray[i] = i;
        qsort_desc(objProbs, 0, validCount - 1, indexArray);

        // 按类NMS
        std::set<int> class_set(classId.begin(), classId.end());
        for (auto c : class_set) nms(validCount, filterBoxes, classId, indexArray, c, NMS_THRESH);

        // 坐标修正(去letterbox) + 输出
        int last_count = 0;
        for (int i = 0; i < validCount; i++) {
            if (indexArray[i] == -1 || last_count >= OBJ_NUMB_MAX_SIZE) continue;
            int n = indexArray[i];
            DetectBox b;
            b.left   = std::max(0.f, (filterBoxes[n*4+0] - pad_left) / scale);
            b.top    = std::max(0.f, (filterBoxes[n*4+1] - pad_top)  / scale);
            b.right  = std::min((float)img_w, b.left + filterBoxes[n*4+2] / scale);
            b.bottom = std::min((float)img_h, b.top  + filterBoxes[n*4+3] / scale);
            b.confidence = objProbs[i];
            b.class_id = classId[n];
            boxes.push_back(b);
            last_count++;
        }
    }

    rknn_outputs_release(impl_->ctx, impl_->io_num.n_output, outputs);
    return 0;
}

// =============================================================================
// 释放
// =============================================================================
void Detector::release() {
    if (impl_->input_attrs)  { free(impl_->input_attrs); impl_->input_attrs = nullptr; }
    if (impl_->output_attrs) { free(impl_->output_attrs); impl_->output_attrs = nullptr; }
    if (impl_->ctx)          { rknn_destroy(impl_->ctx); impl_->ctx = 0; }
}

int Detector::input_width()  const { return impl_->model_width; }
int Detector::input_height() const { return impl_->model_height; }
