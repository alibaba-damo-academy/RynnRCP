// Aero Hand gesture pipeline: MTNN zero-copy NPU inference for Moore Threads E300.
//
// Replicates the MediaPipe hand_landmark_tracking cascade with:
//   - zero-copy I/O: preprocess writes directly into mtnn input buffers,
//     postprocessing reads directly from output buffers (no memcpy);
//   - letterbox + normalize fused into a single pass;
//   - score-first SSD decode (only decode anchors that pass threshold);
//   - tracking ROI from previous landmarks.
#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "mtnn_api.h"

namespace aerohand_mtnn {

struct HandResult {
    std::array<float, 63> image{};  // 21 x (x_norm, y_norm, z)
    std::array<float, 63> world{};  // 21 x (x, y, z) meters
    float score = 0.f;
    bool is_left = false;
};

struct StageStat {
    uint64_t count = 0;
    double total_ms = 0.0;
};

class AeroHandMtnnPipeline {
  public:
    AeroHandMtnnPipeline(const std::string& palm_model, const std::string& hand_model,
                         int num_hands);
    ~AeroHandMtnnPipeline();
    AeroHandMtnnPipeline(const AeroHandMtnnPipeline&) = delete;
    AeroHandMtnnPipeline& operator=(const AeroHandMtnnPipeline&) = delete;

    std::vector<HandResult> process(const uint8_t* data, int width, int height,
                                    int stride_bytes, int format);

    std::map<std::string, StageStat> timings() const { return stage_; }
    void reset_timings() { stage_.clear(); }
    void reset_tracking() { rois_.clear(); }
    void release();

  private:
    struct Roi { float cx, cy, size, rot; };

    struct Model {
        mtnn_mgr mgr = 0;
        std::vector<mtnn_tensor_mem> input_mems;
        std::vector<mtnn_tensor_attr> input_attrs;
        std::vector<mtnn_tensor_mem> output_mems;
        std::vector<mtnn_tensor_attr> output_attrs;
        std::vector<float*> output_ptrs;
    };

    void init_model(Model& m, const std::string& path);
    void destroy_model(Model& m);
    void add_stage(const char* name, double ms, double cpu_ms);
    std::vector<Roi> detect_palms(const uint8_t* data, int width, int height,
                                  int stride_bytes, int format);
    std::vector<HandResult> run_hand(const uint8_t* data, int width, int height,
                                     int stride_bytes, int format, const Roi& roi);

    // Pipeline configuration (MediaPipe hand_landmark_tracking):
    //   palm letterbox 192, hand crop 224, both normalized to [0,1] (div 255).
    static constexpr int PALM_SIZE = 192;
    static constexpr int HAND_SIZE = 224;
    static constexpr float PALM_SCORE_THR = 0.5f;
    static constexpr float PRESENCE_THR = 0.5f;
    static constexpr float NMS_IOU = 0.3f;

    int num_hands_;
    Model palm_;
    Model hand_;
    std::vector<std::array<float, 4>> anchors_;
    std::vector<Roi> rois_;
    std::map<std::string, StageStat> stage_;
    bool released_ = false;
};

}  // namespace aerohand_mtnn
