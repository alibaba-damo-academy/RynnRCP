// Aero Hand gesture pipeline: RGA preprocessing + RKNN zero-copy inference.
//
// Replicates the MediaPipe hand_landmark_tracking cascade (identical to the
// validated Python backend rynnrcp_robot_aero_hand/accelerators/
// mediapipe_lite_rknn.py), with:
//   - palm path: RGA letterbox (+ optional BGR->RGB) written directly into the
//     RKNN zero-copy input buffer (fixed address bound via rknn_set_io_mem);
//   - src frame pointers imported once into RGA and cached (ptr -> handle);
//   - hand path: arbitrary-angle rotated ROI crop on CPU (RGA only rotates in
//     multiples of 90 deg) written directly into the hand zero-copy input.
#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "im2d.hpp"
#include "rga.h"
#include "rknn_api.h"

namespace aerohand {

struct HandResult {
    std::array<float, 63> image{};  // 21 x (x_norm, y_norm, z)
    std::array<float, 63> world{};  // 21 x (x, y, z) meters
    float score = 0.f;              // presence probability
    bool is_left = false;           // MediaPipe raw handedness = P("Left")
};

struct CacheStats {
    uint64_t imports = 0;
    uint64_t hits = 0;
    uint64_t entries = 0;
    uint64_t fallbacks = 0;  // frames processed via CPU letterbox fallback
    uint64_t refusals = 0;   // sources the RGA MMU refused to map (RGA2 4 GB limit)
    uint64_t staged = 0;     // frames copied into the DMA32 buffer to reach RGA
};

struct StageStat {
    uint64_t count = 0;
    double total_ms = 0.0;   // wall clock
    double cpu_ms = 0.0;     // thread CPU time (reveals hidden copies/conversions)
};

class AeroHandPipeline {
  public:
    AeroHandPipeline(const std::string& palm_model, const std::string& hand_model,
                     int num_hands);
    ~AeroHandPipeline();
    AeroHandPipeline(const AeroHandPipeline&) = delete;
    AeroHandPipeline& operator=(const AeroHandPipeline&) = delete;

    // Weight-sharing duplicate via rknn_dup_context (official multi-instance
    // throughput pattern): the clone reuses the loaded model weights but owns
    // its io memories, RGA handles and tracking state, so a pool of clones
    // can pipeline frames across threads safely.
    std::unique_ptr<AeroHandPipeline> clone() const;

    // format: 0 = RGB888, 1 = BGR888. stride_bytes = bytes per row.
    std::vector<HandResult> process(const uint8_t* data, int width, int height,
                                    int stride_bytes, int format);

    CacheStats stats() const;
    std::map<std::string, StageStat> timings() const { return stage_; }
    void reset_timings() { stage_.clear(); }
    void clear_cache();
    void reset_tracking() { rois_.clear(); }
    void release();

  private:
    struct Roi {
        float cx, cy, size, rot;
    };
    struct Model {
        rknn_context ctx = 0;
        rknn_tensor_mem* input = nullptr;
        rknn_tensor_attr input_attr{};  // native (NHWC uint8, w_stride)
        std::vector<rknn_tensor_mem*> outputs;
        std::vector<rknn_tensor_attr> output_attrs;  // float32 or native int8 view
        bool out_int8 = false;          // outputs bound as native int8 (palm)
        std::vector<int32_t> out_zp;
        std::vector<float> out_scale;
    };
    struct SrcEntry {
        rga_buffer_handle_t handle = 0;
        int size = 0;
    };
    struct Det {
        float score;
        std::array<float, 4> box;    // x0,y0,x1,y1 normalized to 192
        std::array<float, 14> kps;   // 7 palm keypoints, normalized
    };

    AeroHandPipeline() = default;  // used by clone()
    void init_model(Model& model, const std::string& path, bool int8_outputs);
    void setup_io(Model& model, bool int8_outputs, const std::string& tag);
    void init_rga_resources();
    void destroy_model(Model& model);
    rga_buffer_t wrap_source(const uint8_t* data, int width, int height,
                             int stride_bytes, int format, bool may_copy);
    // The RGA2 MMU is 32-bit while Linux serves user anonymous memory from high
    // addresses, so on >4 GB boards importing a capture buffer is refused
    // outright ("RGA_MMU unsupported memory larger than 4G"). A dma-buf from the
    // system-dma32 heap is always mappable, so one memcpy buys back the hardware
    // path. Falls back to CPU when the heap is unavailable.
    bool ensure_staging(int size);
    void sync_staging(uint64_t flags);
    rga_buffer_t stage_source(const uint8_t* data, int width, int height,
                              int stride_bytes, int fmt, bool may_copy);
    void release_staging();
    void evict_oldest();
    void add_stage(const char* name, double ms, double cpu);
    void cpu_letterbox(const uint8_t* data, int width, int height,
                       int stride_bytes, int format, int nw, int nh, int padx,
                       int pady);
    std::vector<Roi> detect_palms(const uint8_t* data, rga_buffer_t src,
                                  int width, int height, int stride_bytes,
                                  int format);
    void crop_hand(const uint8_t* data, int width, int height, int stride_bytes,
                   int format, const Roi& roi);
    void warp_bilinear(const uint8_t* src, int sw, int sh, int sstride, int r_off,
                       int b_off, float tlx, float tly, float dux, float duy,
                       float dvx, float dvy);

    int num_hands_;
    Model palm_;
    Model hand_;
    int palm_reg_idx_ = -1;    // output index of the 2016x18 regressors
    int palm_score_idx_ = -1;  // output index of the 2016 scores
    rga_buffer_handle_t palm_dst_handle_ = 0;
    rga_buffer_t palm_dst_img_{};
    uint8_t* tile_buf_ = nullptr;  // small cache-resident tile for 2-pass crop
    rga_buffer_handle_t tile_handle_ = 0;
    rga_buffer_t tile_img_{};
    int last_w_ = -1, last_h_ = -1;  // re-zero letterbox pads on size change

    std::vector<std::array<float, 2>> anchors_;  // 2016 SSD anchor centers
    std::vector<Roi> rois_;                      // tracking state

    std::unordered_map<uintptr_t, SrcEntry> src_cache_;
    std::unordered_set<uintptr_t> src_refused_;  // unmappable: try DMA32 staging
    std::deque<uintptr_t> src_order_;  // FIFO eviction
    std::map<std::string, StageStat> stage_;  // per-stage profiling

    int staging_fd_ = -1;
    uint8_t* staging_virt_ = nullptr;
    int staging_size_ = 0;
    rga_buffer_handle_t staging_handle_ = 0;
    bool staging_available_ = true;         // cleared once the path proves unusable
    const uint8_t* staged_ptr_ = nullptr;   // frame resident in staging this call

    uint64_t cache_imports_ = 0;
    uint64_t cache_hits_ = 0;
    uint64_t cache_refusals_ = 0;
    uint64_t cache_staged_ = 0;
    uint64_t cpu_fallbacks_ = 0;
    bool released_ = false;
};

}  // namespace aerohand
