// Implementation of the RGA + RKNN zero-copy Aero Hand pipeline.
#include "aero_hand_pipeline.hpp"

#include <arm_neon.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <stdexcept>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

// dma-heap / dma-buf UAPI, defined locally rather than pulled from
// <linux/dma-heap.h> / <linux/dma-buf.h> so the build does not require kernel
// headers >= 5.6. This mirrors Rockchip's own librga reference allocator
// (samples/utils/allocator/dma_alloc.cpp).
#ifndef DMA_HEAP_IOCTL_ALLOC
struct dma_heap_allocation_data {
    uint64_t len;
    uint32_t fd;
    uint32_t fd_flags;
    uint64_t heap_flags;
};
#define DMA_HEAP_IOC_MAGIC 'H'
#define DMA_HEAP_IOCTL_ALLOC \
    _IOWR(DMA_HEAP_IOC_MAGIC, 0x0, struct dma_heap_allocation_data)
#endif

#ifndef DMA_BUF_IOCTL_SYNC
struct dma_buf_sync {
    uint64_t flags;
};
#define DMA_BUF_SYNC_READ (1 << 0)
#define DMA_BUF_SYNC_WRITE (2 << 0)
#define DMA_BUF_SYNC_RW (DMA_BUF_SYNC_READ | DMA_BUF_SYNC_WRITE)
#define DMA_BUF_SYNC_START (0 << 2)
#define DMA_BUF_SYNC_END (1 << 2)
#define DMA_BUF_BASE 'b'
#define DMA_BUF_IOCTL_SYNC _IOW(DMA_BUF_BASE, 0, struct dma_buf_sync)
#endif

namespace aerohand {
namespace {

constexpr int kPalmSize = 192;
constexpr int kHandSize = 224;
constexpr float kPalmThr = 0.5f;
constexpr float kPresenceThr = 0.5f;
constexpr float kNmsIou = 0.3f;
constexpr int kTile = 256;  // cache-resident staging tile for the 2-pass crop
// RGA driver refuses further virtualaddr imports after ~50 concurrent 900KB
// buffers (observed on RK3566 / librga 1.9.3): keep the cache well below and
// evict+retry on failure.
constexpr size_t kCacheCap = 48;

inline float sigmoidf(float x) {
    x = std::max(-50.f, std::min(50.f, x));
    return 1.f / (1.f + std::exp(-x));
}

inline float as_prob(float raw) {
    return (raw >= 0.f && raw <= 1.f) ? raw : sigmoidf(raw);
}

// MediaPipe: target 90deg - atan2(-(dy), dx), normalized to (-pi, pi].
inline float rotation_of(float x0, float y0, float x1, float y1) {
    float rot = 0.5f * static_cast<float>(M_PI) - std::atan2(-(y1 - y0), x1 - x0);
    return rot - 2.f * static_cast<float>(M_PI) *
                     std::floor((rot + static_cast<float>(M_PI)) /
                                (2.f * static_cast<float>(M_PI)));
}

std::vector<char> read_file(const std::string& path) {
    std::ifstream stream(path, std::ios::binary | std::ios::ate);
    if (!stream) throw std::runtime_error("cannot open model: " + path);
    std::streamsize size = stream.tellg();
    stream.seekg(0);
    std::vector<char> buffer(static_cast<size_t>(size));
    if (!stream.read(buffer.data(), size))
        throw std::runtime_error("cannot read model: " + path);
    return buffer;
}

inline float box_iou(const std::array<float, 4>& a, const std::array<float, 4>& b) {
    float ix0 = std::max(a[0], b[0]), iy0 = std::max(a[1], b[1]);
    float ix1 = std::min(a[2], b[2]), iy1 = std::min(a[3], b[3]);
    float iw = std::max(0.f, ix1 - ix0), ih = std::max(0.f, iy1 - iy0);
    float inter = iw * ih;
    float uni = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter;
    return uni > 0.f ? inter / uni : 0.f;
}

}  // namespace

namespace {
using Clock = std::chrono::steady_clock;
inline double ms_since(Clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}
inline double cpu_now_ms() {
    timespec ts;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    return ts.tv_sec * 1e3 + ts.tv_nsec * 1e-6;
}
}  // namespace

void AeroHandPipeline::add_stage(const char* name, double ms, double cpu) {
    StageStat& s = stage_[name];
    ++s.count;
    s.total_ms += ms;
    s.cpu_ms += cpu;
}

AeroHandPipeline::AeroHandPipeline(const std::string& palm_model,
                                   const std::string& hand_model, int num_hands)
    : num_hands_(std::max(1, num_hands)) {
    init_model(palm_, palm_model, /*int8_outputs=*/true);
    init_model(hand_, hand_model, /*int8_outputs=*/false);

    // Identify palm outputs by element count: reg = 2016x18, score = 2016.
    for (size_t i = 0; i < palm_.outputs.size(); ++i) {
        if (palm_.output_attrs[i].n_elems == 2016u * 18u) palm_reg_idx_ = static_cast<int>(i);
        else if (palm_.output_attrs[i].n_elems == 2016u) palm_score_idx_ = static_cast<int>(i);
    }
    if (palm_reg_idx_ < 0 || palm_score_idx_ < 0)
        throw std::runtime_error("unexpected palm model outputs (want 2016x18 + 2016)");
    if (hand_.outputs.size() != 4)
        throw std::runtime_error("unexpected hand model outputs (want 4)");

    // SSD anchors, canonical MediaPipe palm 192 generation (2016 anchors).
    const int strides[4] = {8, 16, 16, 16};
    int layer = 0;
    while (layer < 4) {
        int count = 0, last = layer;
        while (last < 4 && strides[last] == strides[layer]) {
            count += 2;  // aspect 1.0 + interpolated scale
            ++last;
        }
        int fm = (kPalmSize + strides[layer] - 1) / strides[layer];
        for (int y = 0; y < fm; ++y)
            for (int x = 0; x < fm; ++x)
                for (int a = 0; a < count; ++a)
                    anchors_.push_back({(x + 0.5f) / fm, (y + 0.5f) / fm});
        layer = last;
    }
    if (anchors_.size() != 2016) throw std::runtime_error("anchor generation mismatch");

    init_rga_resources();
}

std::unique_ptr<AeroHandPipeline> AeroHandPipeline::clone() const {
    // Weight-sharing duplicate: rknn_dup_context reuses the loaded weights,
    // everything per-frame (io mems, RGA handles, caches, tracking) is owned
    // by the clone, so independent threads can run instances concurrently.
    std::unique_ptr<AeroHandPipeline> dup(new AeroHandPipeline());
    dup->num_hands_ = num_hands_;
    rknn_context palm_src = palm_.ctx;
    rknn_context hand_src = hand_.ctx;
    if (rknn_dup_context(&palm_src, &dup->palm_.ctx) != RKNN_SUCC)
        throw std::runtime_error("rknn_dup_context(palm) failed");
    if (rknn_dup_context(&hand_src, &dup->hand_.ctx) != RKNN_SUCC)
        throw std::runtime_error("rknn_dup_context(hand) failed");
    dup->setup_io(dup->palm_, /*int8_outputs=*/true, "palm-clone");
    dup->setup_io(dup->hand_, /*int8_outputs=*/false, "hand-clone");
    dup->palm_reg_idx_ = palm_reg_idx_;
    dup->palm_score_idx_ = palm_score_idx_;
    dup->anchors_ = anchors_;
    dup->init_rga_resources();
    return dup;
}

void AeroHandPipeline::init_rga_resources() {
    // Wrap the palm zero-copy input as the fixed RGA destination.
    int wstride = palm_.input_attr.w_stride > 0 ? palm_.input_attr.w_stride : kPalmSize;
    palm_dst_handle_ = importbuffer_fd(palm_.input->fd, static_cast<int>(palm_.input->size));
    if (palm_dst_handle_ <= 0)
        throw std::runtime_error("RGA importbuffer_fd failed for palm input");
    palm_dst_img_ = wrapbuffer_handle(palm_dst_handle_, kPalmSize, kPalmSize,
                                      RK_FORMAT_RGB_888, wstride, kPalmSize);
    // NOTE: never write this buffer from the CPU. Dirty cache lines would be
    // flushed over the RGA-written pixels at rknn_run time (the runtime syncs
    // bound input mems). Padding is cleared device-side via imfill instead.

    // Staging tile for the 2-pass hand crop: RGA pre-scales the ROI's AABB
    // into this small buffer (DRAM gather done by hardware), the CPU then
    // rotates inside a cache-resident tile instead of the full frame.
    tile_buf_ = static_cast<uint8_t*>(std::aligned_alloc(64, kTile * kTile * 3));
    if (tile_buf_) {
        tile_handle_ = importbuffer_virtualaddr(tile_buf_, kTile * kTile * 3);
        if (tile_handle_ > 0)
            tile_img_ = wrapbuffer_handle(tile_handle_, kTile, kTile, RK_FORMAT_RGB_888);
    }
}

AeroHandPipeline::~AeroHandPipeline() { release(); }

void AeroHandPipeline::init_model(Model& model, const std::string& path,
                                  bool int8_outputs) {
    std::vector<char> blob = read_file(path);
    int ret = rknn_init(&model.ctx, blob.data(), static_cast<uint32_t>(blob.size()), 0, nullptr);
    if (ret != RKNN_SUCC) throw std::runtime_error("rknn_init failed: " + path);
    setup_io(model, int8_outputs, path);
}

void AeroHandPipeline::setup_io(Model& model, bool int8_outputs,
                                const std::string& tag) {
    int ret = 0;
    rknn_input_output_num io_num{};
    rknn_query(model.ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (io_num.n_input != 1) throw std::runtime_error("expected single-input model");

    // Zero-copy input: uint8 NHWC at a fixed address.
    std::memset(&model.input_attr, 0, sizeof(model.input_attr));
    model.input_attr.index = 0;
    rknn_query(model.ctx, RKNN_QUERY_INPUT_ATTR, &model.input_attr, sizeof(model.input_attr));
    model.input_attr.type = RKNN_TENSOR_UINT8;
    model.input_attr.fmt = RKNN_TENSOR_NHWC;
    uint32_t in_size = model.input_attr.size_with_stride > 0
                           ? model.input_attr.size_with_stride
                           : model.input_attr.size;
    model.input = rknn_create_mem(model.ctx, in_size);
    if (!model.input) throw std::runtime_error("rknn_create_mem(input) failed");
    ret = rknn_set_io_mem(model.ctx, model.input, &model.input_attr);
    if (ret != RKNN_SUCC) throw std::runtime_error("rknn_set_io_mem(input) failed");

    if (std::getenv("AERO_HAND_RGA_DEBUG")) {  // reveal hidden layout conversions
        rknn_tensor_attr nat{};
        nat.index = 0;
        rknn_query(model.ctx, RKNN_QUERY_NATIVE_INPUT_ATTR, &nat, sizeof(nat));
        std::fprintf(stderr,
                     "[dbg] %s input: used(type=%d fmt=%d size=%u ws=%d) "
                     "native(type=%d fmt=%d size_ws=%u ws=%d) zp=%d scale=%f\n",
                     tag.c_str(), model.input_attr.type, model.input_attr.fmt,
                     model.input_attr.size, model.input_attr.w_stride, nat.type,
                     nat.fmt, nat.size_with_stride, nat.w_stride, nat.zp, nat.scale);
    }

    // Outputs at fixed addresses. For the palm model we keep the outputs as
    // STANDARD-LAYOUT int8 (attr.type left untouched = INT8) and dequantize
    // lazily, YOLO-demo style: threshold in the int8 domain first, convert
    // only the survivors. The runtime then skips the full int8->float pass
    // (writes 36KB instead of 145KB). Note: NATIVE outputs are NC1HWC2-packed
    // on this model, so the standard-layout int8 view is the practical
    // zero-conversion option.
    model.outputs.resize(io_num.n_output);
    model.output_attrs.resize(io_num.n_output);
    model.out_zp.resize(io_num.n_output, 0);
    model.out_scale.resize(io_num.n_output, 1.f);
    model.out_int8 = int8_outputs;
    if (model.out_int8) {  // all outputs must be int8-quantized, else fall back
        for (uint32_t i = 0; i < io_num.n_output; ++i) {
            rknn_tensor_attr probe{};
            probe.index = i;
            rknn_query(model.ctx, RKNN_QUERY_OUTPUT_ATTR, &probe, sizeof(probe));
            if (probe.type != RKNN_TENSOR_INT8) model.out_int8 = false;
            if (std::getenv("AERO_HAND_RGA_DEBUG"))
                std::fprintf(stderr,
                             "[dbg] %s out[%u]: type=%d fmt=%d n=%u zp=%d scale=%f "
                             "-> int8_path=%d\n",
                             tag.c_str(), i, probe.type, probe.fmt, probe.n_elems,
                             probe.zp, probe.scale, int(model.out_int8));
        }
    }
    for (uint32_t i = 0; i < io_num.n_output; ++i) {
        rknn_tensor_attr& attr = model.output_attrs[i];
        std::memset(&attr, 0, sizeof(attr));
        attr.index = i;
        rknn_query(model.ctx, RKNN_QUERY_OUTPUT_ATTR, &attr, sizeof(attr));
        if (model.out_int8) {
            // keep attr.type == INT8: runtime converts layout only, no dequant
            model.out_zp[i] = attr.zp;
            model.out_scale[i] = attr.scale;
            model.outputs[i] = rknn_create_mem(model.ctx, attr.n_elems);
        } else {
            attr.type = RKNN_TENSOR_FLOAT32;
            model.outputs[i] = rknn_create_mem(model.ctx, attr.n_elems * sizeof(float));
        }
        if (!model.outputs[i]) throw std::runtime_error("rknn_create_mem(output) failed");
        ret = rknn_set_io_mem(model.ctx, model.outputs[i], &attr);
        if (ret != RKNN_SUCC) throw std::runtime_error("rknn_set_io_mem(output) failed");
    }
}

void AeroHandPipeline::destroy_model(Model& model) {
    if (!model.ctx) return;
    if (model.input) rknn_destroy_mem(model.ctx, model.input);
    for (rknn_tensor_mem* mem : model.outputs)
        if (mem) rknn_destroy_mem(model.ctx, mem);
    model.outputs.clear();
    rknn_destroy(model.ctx);
    model.ctx = 0;
    model.input = nullptr;
}

void AeroHandPipeline::evict_oldest() {
    while (!src_order_.empty()) {
        uintptr_t old = src_order_.front();
        src_order_.pop_front();
        auto it = src_cache_.find(old);
        if (it != src_cache_.end()) {
            releasebuffer_handle(it->second.handle);
            src_cache_.erase(it);
            return;
        }
    }
}

namespace {
// A dma-buf from this heap is guaranteed to be backed by pages below 4 GB,
// which is the only kind of memory the 32-bit RGA2 MMU can map.
// Returns -1 if the heap does not exist (permanent), -2 if it exists but the
// allocation failed (transient, e.g. low memory).
int dma32_alloc(size_t len) {
    const int heap = open("/dev/dma_heap/system-dma32", O_RDWR | O_CLOEXEC);
    if (heap < 0) return -1;
    struct dma_heap_allocation_data req{};
    req.len = len;
    req.fd_flags = O_RDWR | O_CLOEXEC;
    const int rc = ioctl(heap, DMA_HEAP_IOCTL_ALLOC, &req);
    close(heap);
    return rc < 0 ? -2 : static_cast<int>(req.fd);
}
}  // namespace

bool AeroHandPipeline::ensure_staging(int size) {
    if (!staging_available_) return false;
    if (staging_handle_ > 0 && staging_size_ >= size) return true;
    const int fd = dma32_alloc(static_cast<size_t>(size));
    if (fd < 0) {
        if (fd == -1) staging_available_ = false;  // no such heap: never retry
        return false;
    }
    void* virt = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (virt == MAP_FAILED) {
        close(fd);
        return false;
    }
    const rga_buffer_handle_t handle = importbuffer_fd(fd, size);
    if (handle <= 0) {
        // RGA refusing a dma32 fd means the hardware path is unusable at all.
        munmap(virt, size);
        close(fd);
        staging_available_ = false;
        return false;
    }
    release_staging();  // only now: a failed grow must not lose a working buffer
    staging_fd_ = fd;
    staging_virt_ = static_cast<uint8_t*>(virt);
    staging_size_ = size;
    staging_handle_ = handle;
    return true;
}

void AeroHandPipeline::release_staging() {
    if (staging_handle_ > 0) releasebuffer_handle(staging_handle_);
    if (staging_virt_) munmap(staging_virt_, staging_size_);
    if (staging_fd_ >= 0) close(staging_fd_);
    staging_handle_ = 0;
    staging_virt_ = nullptr;
    staging_fd_ = -1;
    staging_size_ = 0;
    staged_ptr_ = nullptr;
}

void AeroHandPipeline::sync_staging(uint64_t flags) {
    if (staging_fd_ < 0) return;
    struct dma_buf_sync sync{};
    sync.flags = flags;
    ioctl(staging_fd_, DMA_BUF_IOCTL_SYNC, &sync);
}

rga_buffer_t AeroHandPipeline::stage_source(const uint8_t* data, int width, int height,
                                           int stride_bytes, int fmt, bool may_copy) {
    const int size = stride_bytes * height;
    // Already staged by an earlier stage of this same frame: reuse for free.
    if (staged_ptr_ == data && staging_handle_ > 0 && staging_size_ >= size)
        return wrapbuffer_handle(staging_handle_, width, height, fmt,
                                 stride_bytes / 3, height);
    if (!may_copy || !ensure_staging(size)) return rga_buffer_t{};
    // RGA is not cache-coherent with the CPU, and the handle is imported once
    // and reused, so librga performs no per-frame maintenance. Bracket the write
    // so the exporter flushes it; without this the first use of a freshly
    // faulted mapping leaves dirty lines RGA cannot see.
    sync_staging(DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE);
    std::memcpy(staging_virt_, data, size);
    sync_staging(DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE);
    staged_ptr_ = data;
    ++cache_staged_;
    return wrapbuffer_handle(staging_handle_, width, height, fmt,
                             stride_bytes / 3, height);
}

rga_buffer_t AeroHandPipeline::wrap_source(const uint8_t* data, int width, int height,
                                           int stride_bytes, int format, bool may_copy) {
    const uintptr_t key = reinterpret_cast<uintptr_t>(data);
    const int size = stride_bytes * height;
    const int fmt = (format == 1) ? RK_FORMAT_BGR_888 : RK_FORMAT_RGB_888;
    auto it = src_cache_.find(key);
    if (src_refused_.count(key) && it == src_cache_.end())
        return stage_source(data, width, height, stride_bytes, fmt, may_copy);
    if (it != src_cache_.end() && it->second.size >= size) {
        ++cache_hits_;
    } else {
        if (it != src_cache_.end()) {  // same ptr, larger buffer: re-import
            releasebuffer_handle(it->second.handle);
            src_cache_.erase(it);
        }
        while (src_cache_.size() >= kCacheCap) evict_oldest();
        rga_buffer_handle_t handle =
            importbuffer_virtualaddr(const_cast<uint8_t*>(data), size);
        // The driver has a global import quota; evict our oldest handles and
        // retry before giving up.
        while (handle <= 0 && !src_cache_.empty()) {
            evict_oldest();
            handle = importbuffer_virtualaddr(const_cast<uint8_t*>(data), size);
        }
        if (handle <= 0) {
            // Refused for good: either the quota is exhausted or, far more
            // commonly on >4 GB boards, the pages sit above the RGA2 MMU's 4 GB
            // ceiling. Wrapping the address instead does not help -- RGA_BLIT
            // rejects the same pages. Remember the refusal so the doomed ioctl
            // is not repeated every frame, then try the DMA32 staging buffer.
            if (src_refused_.size() >= kCacheCap) src_refused_.clear();
            src_refused_.insert(key);
            ++cache_refusals_;
            return stage_source(data, width, height, stride_bytes, fmt, may_copy);
        }
        src_cache_[key] = {handle, size};
        src_order_.push_back(key);
        ++cache_imports_;
        it = src_cache_.find(key);
    }
    return wrapbuffer_handle(it->second.handle, width, height, fmt,
                             stride_bytes / 3, height);
}

// CPU bilinear letterbox fallback (used when the RGA import quota is
// exhausted). Writing the whole zero-copy palm input from the CPU is safe:
// rknn_run cleans the CPU cache for bound input mems (same as the hand path).
void AeroHandPipeline::cpu_letterbox(const uint8_t* data, int width, int height,
                                     int stride_bytes, int format, int nw, int nh,
                                     int padx, int pady) {
    ++cpu_fallbacks_;
    const int wstride = palm_.input_attr.w_stride > 0 ? palm_.input_attr.w_stride : kPalmSize;
    uint8_t* base = static_cast<uint8_t*>(palm_.input->virt_addr);
    std::memset(base, 0, palm_.input->size);
    const float sx = static_cast<float>(width) / nw;
    const float sy = static_cast<float>(height) / nh;
    const int r_off = (format == 1) ? 2 : 0;
    const int b_off = (format == 1) ? 0 : 2;
    for (int y = 0; y < nh; ++y) {
        const float syf = std::min((y + 0.5f) * sy - 0.5f, height - 1.001f);
        const int y0 = std::max(0, static_cast<int>(syf));
        const float fy = std::max(0.f, syf - y0);
        const uint8_t* row0 = data + static_cast<size_t>(y0) * stride_bytes;
        const uint8_t* row1 = data + static_cast<size_t>(std::min(y0 + 1, height - 1)) * stride_bytes;
        uint8_t* out_row = base + (static_cast<size_t>(y + pady) * wstride + padx) * 3;
        for (int x = 0; x < nw; ++x) {
            const float sxf = std::min((x + 0.5f) * sx - 0.5f, width - 1.001f);
            const int x0 = std::max(0, static_cast<int>(sxf));
            const float fx = std::max(0.f, sxf - x0);
            const int x1 = std::min(x0 + 1, width - 1);
            uint8_t* out = out_row + x * 3;
            for (int ch = 0; ch < 3; ++ch) {
                const int sc = (ch == 0) ? r_off : (ch == 2 ? b_off : 1);
                const float top = row0[x0 * 3 + sc] * (1 - fx) + row0[x1 * 3 + sc] * fx;
                const float bot = row1[x0 * 3 + sc] * (1 - fx) + row1[x1 * 3 + sc] * fx;
                out[ch] = static_cast<uint8_t>(top * (1 - fy) + bot * fy + 0.5f);
            }
        }
    }
    last_w_ = last_h_ = -1;  // buffer dirtied by CPU: re-imfill before next RGA use
}

std::vector<AeroHandPipeline::Roi> AeroHandPipeline::detect_palms(
    const uint8_t* data, rga_buffer_t src, int width, int height,
    int stride_bytes, int format) {
    const float scale = static_cast<float>(kPalmSize) / std::max(width, height);
    const int nw = static_cast<int>(std::lround(width * scale));
    const int nh = static_cast<int>(std::lround(height * scale));
    const int padx = (kPalmSize - nw) / 2;
    const int pady = (kPalmSize - nh) / 2;
    if (width != last_w_ || height != last_h_) {
        // Device-side clear of the letterbox padding (CPU writes are unsafe
        // here: dirty cache lines would later be flushed over RGA output).
        im_rect full{0, 0, kPalmSize, kPalmSize};
        if (imfill(palm_dst_img_, full, 0x00000000, IM_SYNC) != IM_STATUS_SUCCESS)
            return {};
        last_w_ = width;
        last_h_ = height;
    }
    im_rect srect{0, 0, width, height};
    im_rect drect{padx, pady, nw, nh};
    rga_buffer_t pat{};
    im_rect prect{};
    IM_STATUS status = IM_STATUS_FAILED;
    auto t0 = Clock::now();
    double c0 = cpu_now_ms();
    if (src.handle > 0)
        status = improcess(src, palm_dst_img_, pat, srect, drect, prect, IM_SYNC);
    if (status != IM_STATUS_SUCCESS) {
        cpu_letterbox(data, width, height, stride_bytes, format, nw, nh, padx, pady);
        add_stage("palm.cpu_letterbox", ms_since(t0), cpu_now_ms() - c0);
    } else {
        add_stage("palm.rga_letterbox", ms_since(t0), cpu_now_ms() - c0);
    }

    t0 = Clock::now();
    c0 = cpu_now_ms();
    rknn_run(palm_.ctx, nullptr);
    add_stage("palm.npu_run", ms_since(t0), cpu_now_ms() - c0);
    t0 = Clock::now();
    c0 = cpu_now_ms();

    // Greedy decode: threshold in the int8 domain ((q-zp)*scale > 0  <=>
    // q > zp for the 0.5 sigmoid cut), dequantize survivors only.
    const bool i8 = palm_.out_int8;
    const int8_t* sc_q = static_cast<const int8_t*>(palm_.outputs[palm_score_idx_]->virt_addr);
    const int8_t* rg_q = static_cast<const int8_t*>(palm_.outputs[palm_reg_idx_]->virt_addr);
    const float* sc_f = static_cast<const float*>(palm_.outputs[palm_score_idx_]->virt_addr);
    const float* rg_f = static_cast<const float*>(palm_.outputs[palm_reg_idx_]->virt_addr);
    const int32_t szp = palm_.out_zp[palm_score_idx_];
    const float ssc = palm_.out_scale[palm_score_idx_];
    const int32_t rzp = palm_.out_zp[palm_reg_idx_];
    const float rsc = palm_.out_scale[palm_reg_idx_];
    const auto rval = [&](int i, int k) -> float {
        return i8 ? (rg_q[18 * i + k] - rzp) * rsc : rg_f[18 * i + k];
    };

    std::vector<Det> dets;
    for (int i = 0; i < 2016; ++i) {
        float logit;
        if (i8) {
            if (sc_q[i] <= szp) continue;
            logit = (sc_q[i] - szp) * ssc;
        } else {
            logit = sc_f[i];
            if (logit <= 0.f) continue;  // sigmoid(x) > 0.5  <=>  x > 0
        }
        const float ax = anchors_[i][0], ay = anchors_[i][1];
        const float xc = rval(i, 0) / kPalmSize + ax;
        const float yc = rval(i, 1) / kPalmSize + ay;
        const float bw = rval(i, 2) / kPalmSize;
        const float bh = rval(i, 3) / kPalmSize;
        Det det;
        det.score = sigmoidf(logit);
        det.box = {xc - bw / 2, yc - bh / 2, xc + bw / 2, yc + bh / 2};
        for (int k = 0; k < 7; ++k) {
            det.kps[2 * k] = rval(i, 4 + 2 * k) / kPalmSize + ax;
            det.kps[2 * k + 1] = rval(i, 5 + 2 * k) / kPalmSize + ay;
        }
        dets.push_back(det);
    }

    // MediaPipe-style weighted NMS.
    std::sort(dets.begin(), dets.end(),
              [](const Det& a, const Det& b) { return a.score > b.score; });
    std::vector<Roi> rois;
    std::vector<bool> used(dets.size(), false);
    for (size_t i = 0; i < dets.size(); ++i) {
        if (used[i]) continue;
        std::array<float, 4> box{};
        std::array<float, 14> kps{};
        float wsum = 0.f;
        for (size_t j = i; j < dets.size(); ++j) {
            if (used[j] || box_iou(dets[i].box, dets[j].box) <= kNmsIou) continue;
            used[j] = true;
            const float w = dets[j].score;
            wsum += w;
            for (int k = 0; k < 4; ++k) box[k] += w * dets[j].box[k];
            for (int k = 0; k < 14; ++k) kps[k] += w * dets[j].kps[k];
        }
        for (int k = 0; k < 4; ++k) box[k] /= wsum;
        for (int k = 0; k < 14; ++k) kps[k] /= wsum;

        // letterbox(192) -> frame pixels.
        const float x0 = (box[0] * kPalmSize - padx) / scale;
        const float y0 = (box[1] * kPalmSize - pady) / scale;
        const float x1 = (box[2] * kPalmSize - padx) / scale;
        const float y1 = (box[3] * kPalmSize - pady) / scale;
        const float wristx = (kps[0] * kPalmSize - padx) / scale;
        const float wristy = (kps[1] * kPalmSize - pady) / scale;
        const float middlex = (kps[4] * kPalmSize - padx) / scale;
        const float middley = (kps[5] * kPalmSize - pady) / scale;
        const float rot = rotation_of(wristx, wristy, middlex, middley);

        // RectTransformation: shift (rotated frame) -> scale 2.6 -> square_long.
        const float boxw = x1 - x0, boxh = y1 - y0;
        float cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
        const float shift_y = -0.5f;
        cx += shift_y * boxh * (-std::sin(rot));
        cy += shift_y * boxh * (std::cos(rot));
        rois.push_back({cx, cy, std::max(boxw, boxh) * 2.6f, rot});
    }
    add_stage("palm.decode_nms", ms_since(t0), cpu_now_ms() - c0);
    return rois;
}

void AeroHandPipeline::crop_hand(const uint8_t* data, int width, int height,
                                 int stride_bytes, int format, const Roi& roi) {
    // Rotated square ROI -> 224x224 bilinear (arbitrary angle; RGA only does
    // 90deg steps), written straight into the hand zero-copy input buffer.
    // Fast path: RGA pre-scales the ROI's axis-aligned bounding box into a
    // small tile (hardware does the DRAM-heavy gather + color conversion),
    // then the CPU warp samples a cache-resident buffer.
    const float half = roi.size / 2.f;
    const float c = std::cos(roi.rot), s = std::sin(roi.rot);
    const float ux = c * half, uy = s * half;    // rotated x axis (half length)
    const float vx = -s * half, vy = c * half;   // rotated y axis (half length)
    const float tlx = roi.cx - ux - vx, tly = roi.cy - uy - vy;
    const float dux = 2 * ux / kHandSize, duy = 2 * uy / kHandSize;
    const float dvx = 2 * vx / kHandSize, dvy = 2 * vy / kHandSize;
    const int r_off = (format == 1) ? 2 : 0;  // BGR source: swap R/B on write
    const int b_off = (format == 1) ? 0 : 2;

    if (tile_handle_ > 0) {
        const float half_a = half * (std::fabs(c) + std::fabs(s));
        const int ax0 = static_cast<int>(std::floor(roi.cx - half_a)) - 1;
        const int ay0 = static_cast<int>(std::floor(roi.cy - half_a)) - 1;
        const int ax1 = static_cast<int>(std::ceil(roi.cx + half_a)) + 1;
        const int ay1 = static_cast<int>(std::ceil(roi.cy + half_a)) + 1;
        const int aw = ax1 - ax0, ah = ay1 - ay0;
        const int sx0 = std::max(ax0, 0), sy0 = std::max(ay0, 0);
        const int sx1 = std::min(ax1, width), sy1 = std::min(ay1, height);
        if (aw >= 20 && aw <= 4000 && sx1 > sx0 + 1 && sy1 > sy0 + 1) {
            const float scale_x = static_cast<float>(kTile) / aw;
            const float scale_y = static_cast<float>(kTile) / ah;
            const int dx0 = static_cast<int>(std::lround((sx0 - ax0) * scale_x));
            const int dy0 = static_cast<int>(std::lround((sy0 - ay0) * scale_y));
            int dw = static_cast<int>(std::lround((sx1 - sx0) * scale_x));
            int dh = static_cast<int>(std::lround((sy1 - sy0) * scale_y));
            dw = std::min(dw, kTile - dx0);
            dh = std::min(dh, kTile - dy0);
            const bool clipped = sx0 != ax0 || sy0 != ay0 || sx1 != ax1 || sy1 != ay1;
            rga_buffer_t src = wrap_source(data, width, height, stride_bytes, format,
                                           /*may_copy=*/false);
            if (src.handle > 0 && dw > 1 && dh > 1) {
                if (clipped) std::memset(tile_buf_, 0, kTile * kTile * 3);
                im_rect srect{sx0, sy0, sx1 - sx0, sy1 - sy0};
                im_rect drect{dx0, dy0, dw, dh};
                rga_buffer_t pat{};
                im_rect prect{};
                auto t0 = Clock::now();
                double c0 = cpu_now_ms();
                IM_STATUS st = improcess(src, tile_img_, pat, srect, drect, prect, IM_SYNC);
                add_stage("hand.rga_tile", ms_since(t0), cpu_now_ms() - c0);
                if (st == IM_STATUS_SUCCESS) {
                    // frame -> tile mapping: t = (f - a0) * scale (tile is RGB)
                    warp_bilinear(tile_buf_, kTile, kTile, kTile * 3, 0, 2,
                                  (tlx - ax0) * scale_x, (tly - ay0) * scale_y,
                                  dux * scale_x, duy * scale_y,
                                  dvx * scale_x, dvy * scale_y);
                    return;
                }
            }
        }
    }
    warp_bilinear(data, width, height, stride_bytes, r_off, b_off,
                  tlx, tly, dux, duy, dvx, dvy);
}

void AeroHandPipeline::warp_bilinear(const uint8_t* src, int sw, int sh, int sstride,
                                     int r_off, int b_off, float tlx, float tly,
                                     float dux, float duy, float dvx, float dvy) {
    const int wstride = hand_.input_attr.w_stride > 0 ? hand_.input_attr.w_stride : kHandSize;
    uint8_t* dst_base = static_cast<uint8_t*>(hand_.input->virt_addr);
    const int32_t fdux = static_cast<int32_t>(std::lround(dux * 65536.f));
    const int32_t fduy = static_cast<int32_t>(std::lround(duy * 65536.f));

    for (int y = 0; y < kHandSize; ++y) {
        const float row_x = tlx + y * dvx, row_y = tly + y * dvy;
        const float end_x = row_x + (kHandSize - 1) * dux;
        const float end_y = row_y + (kHandSize - 1) * duy;
        uint8_t* row = dst_base + static_cast<size_t>(y) * wstride * 3;
        const bool interior =
            std::min(row_x, end_x) >= 0.f && std::max(row_x, end_x) < sw - 2.f &&
            std::min(row_y, end_y) >= 0.f && std::max(row_y, end_y) < sh - 2.f;
        if (interior) {
            // 2-pixel interleaved NEON: two independent dependency chains per
            // iteration to exploit the in-order A55 dual-issue pipeline.
            static const uint8x8_t kShuffle = {0, 1, 2, 255, 3, 4, 5, 255};
            int32_t sx = static_cast<int32_t>(std::lround(row_x * 65536.f));
            int32_t sy = static_cast<int32_t>(std::lround(row_y * 65536.f));
            for (int x = 0; x < kHandSize; x += 2, sx += 2 * fdux, sy += 2 * fduy) {
                const int32_t sxb = sx + fdux, syb = sy + fduy;
                const uint8_t* pa = src + static_cast<size_t>(sy >> 16) * sstride + (sx >> 16) * 3;
                const uint8_t* pb = src + static_cast<size_t>(syb >> 16) * sstride + (sxb >> 16) * 3;
                uint16x8_t va0 = vmovl_u8(vtbl1_u8(vld1_u8(pa), kShuffle));
                uint16x8_t vb0 = vmovl_u8(vtbl1_u8(vld1_u8(pb), kShuffle));
                uint16x8_t va1 = vmovl_u8(vtbl1_u8(vld1_u8(pa + sstride), kShuffle));
                uint16x8_t vb1 = vmovl_u8(vtbl1_u8(vld1_u8(pb + sstride), kShuffle));
                const uint32_t fxa = (static_cast<uint32_t>(sx) & 0xFFFFu) >> 9;
                const uint32_t fya = (static_cast<uint32_t>(sy) & 0xFFFFu) >> 9;
                const uint32_t fxb = (static_cast<uint32_t>(sxb) & 0xFFFFu) >> 9;
                const uint32_t fyb = (static_cast<uint32_t>(syb) & 0xFFFFu) >> 9;
                const uint16_t w11a = fxa * fya, w11b = fxb * fyb;
                const uint16_t w10a = (fya << 7) - w11a, w10b = (fyb << 7) - w11b;
                const uint16_t w01a = (fxa << 7) - w11a, w01b = (fxb << 7) - w11b;
                const uint16_t w00a = (1u << 14) - w01a - w10a - w11a;
                const uint16_t w00b = (1u << 14) - w01b - w10b - w11b;
                uint32x4_t la = vmull_u16(vget_low_u16(va0), vdup_n_u16(w00a));
                uint32x4_t lb = vmull_u16(vget_low_u16(vb0), vdup_n_u16(w00b));
                la = vmlal_u16(la, vget_low_u16(va1), vdup_n_u16(w10a));
                lb = vmlal_u16(lb, vget_low_u16(vb1), vdup_n_u16(w10b));
                uint32x4_t ha = vmull_u16(vget_high_u16(va0), vdup_n_u16(w01a));
                uint32x4_t hb = vmull_u16(vget_high_u16(vb0), vdup_n_u16(w01b));
                ha = vmlal_u16(ha, vget_high_u16(va1), vdup_n_u16(w11a));
                hb = vmlal_u16(hb, vget_high_u16(vb1), vdup_n_u16(w11b));
                uint32_t bufa[4], bufb[4];
                vst1q_u32(bufa, vaddq_u32(la, ha));
                vst1q_u32(bufb, vaddq_u32(lb, hb));
                uint8_t* out = row + x * 3;
                out[0] = static_cast<uint8_t>((bufa[r_off] + 8192) >> 14);
                out[1] = static_cast<uint8_t>((bufa[1] + 8192) >> 14);
                out[2] = static_cast<uint8_t>((bufa[b_off] + 8192) >> 14);
                out[3] = static_cast<uint8_t>((bufb[r_off] + 8192) >> 14);
                out[4] = static_cast<uint8_t>((bufb[1] + 8192) >> 14);
                out[5] = static_cast<uint8_t>((bufb[b_off] + 8192) >> 14);
            }
            continue;
        }
        // Border rows: guarded float path (rare).
        float sxf = row_x, syf = row_y;
        for (int x = 0; x < kHandSize; ++x, sxf += dux, syf += duy) {
            uint8_t* out = row + x * 3;
            const int x0 = static_cast<int>(std::floor(sxf));
            const int y0 = static_cast<int>(std::floor(syf));
            if (x0 < -1 || y0 < -1 || x0 >= sw || y0 >= sh) {
                out[0] = out[1] = out[2] = 0;
                continue;
            }
            const float fx = sxf - x0, fy = syf - y0;
            const float w00 = (1 - fx) * (1 - fy), w01 = fx * (1 - fy);
            const float w10 = (1 - fx) * fy, w11 = fx * fy;
            const bool in00 = x0 >= 0 && y0 >= 0;
            const bool in01 = x0 + 1 < sw && y0 >= 0;
            const bool in10 = x0 >= 0 && y0 + 1 < sh;
            const bool in11 = x0 + 1 < sw && y0 + 1 < sh;
            const uint8_t* p00 = src + static_cast<size_t>(y0) * sstride + x0 * 3;
            for (int ch = 0; ch < 3; ++ch) {
                const int sc = (ch == 0) ? r_off : (ch == 2 ? b_off : 1);
                float acc = 0.f;
                if (in00) acc += w00 * p00[sc];
                if (in01) acc += w01 * p00[3 + sc];
                if (in10) acc += w10 * p00[sstride + sc];
                if (in11) acc += w11 * p00[sstride + 3 + sc];
                out[ch] = static_cast<uint8_t>(acc + 0.5f);
            }
        }
    }
}

std::vector<HandResult> AeroHandPipeline::process(const uint8_t* data, int width,
                                                  int height, int stride_bytes,
                                                  int format) {
    auto tf = Clock::now();
    double cf = cpu_now_ms();
    // Capture layers reuse one buffer address across frames, so last frame's
    // staged copy must not be mistaken for this one.
    staged_ptr_ = nullptr;
    std::vector<Roi> rois = rois_;
    if (static_cast<int>(rois.size()) < num_hands_) {
        auto t0 = Clock::now();
        double c0 = cpu_now_ms();
        rga_buffer_t src = wrap_source(data, width, height, stride_bytes, format,
                                       /*may_copy=*/true);
        add_stage("palm.wrap_src", ms_since(t0), cpu_now_ms() - c0);
        for (const Roi& roi :
             detect_palms(data, src, width, height, stride_bytes, format)) {
            bool duplicate = false;
            for (const Roi& tracked : rois) {
                const float dx = roi.cx - tracked.cx, dy = roi.cy - tracked.cy;
                if (std::hypot(dx, dy) < 0.5f * std::min(roi.size, tracked.size)) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) rois.push_back(roi);
            if (static_cast<int>(rois.size()) >= num_hands_) break;
        }
    }

    std::vector<HandResult> results;
    std::vector<Roi> next_rois;
    const int limit = std::min<int>(num_hands_, rois.size());
    for (int r = 0; r < limit; ++r) {
        const Roi& roi = rois[r];
        auto t0 = Clock::now();
        double c0 = cpu_now_ms();
        crop_hand(data, width, height, stride_bytes, format, roi);
        add_stage("hand.crop_affine", ms_since(t0), cpu_now_ms() - c0);
        t0 = Clock::now();
        c0 = cpu_now_ms();
        rknn_run(hand_.ctx, nullptr);
        add_stage("hand.npu_run", ms_since(t0), cpu_now_ms() - c0);
        t0 = Clock::now();
        c0 = cpu_now_ms();
        const float* lm = static_cast<const float*>(hand_.outputs[0]->virt_addr);
        const float* score = static_cast<const float*>(hand_.outputs[1]->virt_addr);
        const float* handed = static_cast<const float*>(hand_.outputs[2]->virt_addr);
        const float* world = static_cast<const float*>(hand_.outputs[3]->virt_addr);
        const float presence = as_prob(score[0]);
        if (presence < kPresenceThr) continue;

        const float half = roi.size / 2.f;
        const float c = std::cos(roi.rot), s = std::sin(roi.rot);
        const float ux = c * half, uy = s * half;
        const float vx = -s * half, vy = c * half;
        const float tlx = roi.cx - ux - vx, tly = roi.cy - uy - vy;

        HandResult hand;
        hand.score = presence;
        hand.is_left = as_prob(handed[0]) > 0.5f;
        float frame_pts[21][2];
        for (int k = 0; k < 21; ++k) {
            const float lx = lm[3 * k], ly = lm[3 * k + 1], lz = lm[3 * k + 2];
            const float fxp = tlx + (lx / kHandSize) * 2 * ux + (ly / kHandSize) * 2 * vx;
            const float fyp = tly + (lx / kHandSize) * 2 * uy + (ly / kHandSize) * 2 * vy;
            frame_pts[k][0] = fxp;
            frame_pts[k][1] = fyp;
            hand.image[3 * k] = fxp / width;
            hand.image[3 * k + 1] = fyp / height;
            hand.image[3 * k + 2] = lz / kHandSize * (roi.size / width);
            const float wx = world[3 * k], wy = world[3 * k + 1];
            hand.world[3 * k] = wx * c - wy * s;
            hand.world[3 * k + 1] = wx * s + wy * c;
            hand.world[3 * k + 2] = world[3 * k + 2];
        }
        results.push_back(hand);

        // Next-frame ROI from stable landmarks: direction = wrist(0) ->
        // weighted index/middle/ring MCP knuckles, and the AABB over
        // palm+knuckle points only (fingertips excluded) so finger flexion
        // does not jitter the crop. A jittery crop drives the handedness
        // head across its decision boundary, which shows up as Left/Right
        // label flapping.
        const float dx = (frame_pts[5][0] + 2.f * frame_pts[9][0] +
                          frame_pts[13][0]) / 4.f;
        const float dy = (frame_pts[5][1] + 2.f * frame_pts[9][1] +
                          frame_pts[13][1]) / 4.f;
        const float rot = rotation_of(frame_pts[0][0], frame_pts[0][1], dx, dy);
        const float rc = std::cos(rot), rs = std::sin(rot);
        static constexpr int kStablePts[12] = {0, 1, 2, 3, 5, 6,
                                               9, 10, 13, 14, 17, 18};
        float amin = 1e30f, amax = -1e30f, bmin = 1e30f, bmax = -1e30f;
        for (int idx : kStablePts) {
            const float* pt = frame_pts[idx];
            const float a = pt[0] * rc + pt[1] * rs;
            const float b = -pt[0] * rs + pt[1] * rc;
            amin = std::min(amin, a); amax = std::max(amax, a);
            bmin = std::min(bmin, b); bmax = std::max(bmax, b);
        }
        const float ca = (amin + amax) / 2, cb = (bmin + bmax) / 2;
        float cx = ca * rc - cb * rs, cy = ca * rs + cb * rc;
        const float bh = bmax - bmin;
        cx += -0.1f * bh * (-rs);
        cy += -0.1f * bh * rc;
        next_rois.push_back({cx, cy, std::max(amax - amin, bmax - bmin) * 2.0f, rot});
        add_stage("hand.postproc", ms_since(t0), cpu_now_ms() - c0);
    }
    rois_ = next_rois;
    add_stage("frame.total", ms_since(tf), cpu_now_ms() - cf);
    return results;
}

CacheStats AeroHandPipeline::stats() const {
    return {cache_imports_, cache_hits_,     src_cache_.size(),
            cpu_fallbacks_, cache_refusals_, cache_staged_};
}

void AeroHandPipeline::clear_cache() {
    for (auto& entry : src_cache_) releasebuffer_handle(entry.second.handle);
    src_cache_.clear();
    src_refused_.clear();
    src_order_.clear();
}

void AeroHandPipeline::release() {
    if (released_) return;
    released_ = true;
    clear_cache();
    release_staging();
    if (palm_dst_handle_ > 0) {
        releasebuffer_handle(palm_dst_handle_);
        palm_dst_handle_ = 0;
    }
    if (tile_handle_ > 0) {
        releasebuffer_handle(tile_handle_);
        tile_handle_ = 0;
    }
    if (tile_buf_) {
        std::free(tile_buf_);
        tile_buf_ = nullptr;
    }
    destroy_model(palm_);
    destroy_model(hand_);
}

}  // namespace aerohand
