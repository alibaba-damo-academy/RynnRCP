#include "aero_hand_mtnn.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace aerohand_mtnn {

namespace {

constexpr int PALM_SZ = 192;
constexpr int HAND_SZ = 224;
constexpr float PALM_THR = 0.5f;
constexpr float PRESENCE_THR = 0.5f;
constexpr float NMS_IOU = 0.3f;
constexpr int ROI_LM[] = {0,1,2,3,5,6,9,10,13,14,17,18};

using Clock = std::chrono::steady_clock;
inline double ms_since(Clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

inline float sigmoidf(float x) {
    x = std::max(-50.f, std::min(50.f, x));
    return 1.f / (1.f + std::exp(-x));
}

// Bilinear sums can overshoot 255 by float epsilon; clamp before the
// uint8 cast so the conversion never goes out of range.
inline float norm8(float v, float inv) {
    return (uint8_t)std::min(v + 0.5f, 255.f) * inv;
}

inline float rotation_of(float x0, float y0, float x1, float y1) {
    float r = 0.5f * (float)M_PI - std::atan2(-(y1-y0), x1-x0);
    return r - 2.f*(float)M_PI * std::floor((r + (float)M_PI) / (2.f*(float)M_PI));
}

inline float box_iou(const float* a, const float* b) {
    float iw = std::max(0.f, std::min(a[2],b[2]) - std::max(a[0],b[0]));
    float ih = std::max(0.f, std::min(a[3],b[3]) - std::max(a[1],b[1]));
    float inter = iw * ih;
    float uni = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter;
    return uni > 0.f ? inter / uni : 0.f;
}

std::vector<char> read_file(const std::string& path) {
    std::ifstream s(path, std::ios::binary | std::ios::ate);
    if (!s) throw std::runtime_error("cannot open: " + path);
    auto sz = s.tellg(); s.seekg(0);
    std::vector<char> buf(sz);
    s.read(buf.data(), sz);
    return buf;
}

// Letterbox resize (cv2.INTER_LINEAR-compatible) + normalize + HWC->NCHW,
// written directly to the NPU buffer. Same mapping as the RKNN backend's
// cpu_letterbox: src = (dst + 0.5) * scale - 0.5, edge-clamped bilinear.
void letterbox_to_npu(const uint8_t* src, int sw, int sh, int sstride,
                      int format, float* npu_buf, int target,
                      int& pad_x, int& pad_y, float& scale) {
    scale = (float)target / std::max(sw, sh);
    int nw = (int)std::lround(sw * scale);
    int nh = (int)std::lround(sh * scale);
    pad_x = (target - nw) / 2;
    pad_y = (target - nh) / 2;

    const int HW = target * target;
    const float inv = 1.f / 255.f;
    // Zero canvas (letterbox padding), then fill the resized inner region.
    std::memset(npu_buf, 0, 3 * (size_t)HW * sizeof(float));
    const float sx = (float)sw / nw;
    const float sy = (float)sh / nh;
    // BGR source: sample R/B swapped so the tensor is always RGB.
    const int r_off = (format == 1) ? 2 : 0;
    const int b_off = (format == 1) ? 0 : 2;
    float* ch0 = npu_buf;
    float* ch1 = npu_buf + HW;
    float* ch2 = npu_buf + 2 * HW;
    for (int y = 0; y < nh; ++y) {
        const float syf = std::min((y + 0.5f) * sy - 0.5f, sh - 1.001f);
        const int y0 = std::max(0, (int)syf);
        const float fy = std::max(0.f, syf - y0);
        const uint8_t* row0 = src + (size_t)y0 * sstride;
        const uint8_t* row1 = src + (size_t)std::min(y0 + 1, sh - 1) * sstride;
        const size_t dst_off = (size_t)(y + pad_y) * target + pad_x;
        float* o0 = ch0 + dst_off;
        float* o1 = ch1 + dst_off;
        float* o2 = ch2 + dst_off;
        for (int x = 0; x < nw; ++x) {
            const float sxf = std::min((x + 0.5f) * sx - 0.5f, sw - 1.001f);
            const int x0 = std::max(0, (int)sxf);
            const float fx = std::max(0.f, sxf - x0);
            const int x1 = std::min(x0 + 1, sw - 1);
            const uint8_t* p00 = row0 + x0 * 3;
            const uint8_t* p01 = row0 + x1 * 3;
            const uint8_t* p10 = row1 + x0 * 3;
            const uint8_t* p11 = row1 + x1 * 3;
            // Round through uint8 like cv2 before normalizing.
            float top = p00[r_off] * (1 - fx) + p01[r_off] * fx;
            float bot = p10[r_off] * (1 - fx) + p11[r_off] * fx;
            o0[x] = norm8(top * (1 - fy) + bot * fy, inv);
            top = p00[1] * (1 - fx) + p01[1] * fx;
            bot = p10[1] * (1 - fx) + p11[1] * fx;
            o1[x] = norm8(top * (1 - fy) + bot * fy, inv);
            top = p00[b_off] * (1 - fx) + p01[b_off] * fx;
            bot = p10[b_off] * (1 - fx) + p11[b_off] * fx;
            o2[x] = norm8(top * (1 - fy) + bot * fy, inv);
        }
    }
}

// Affine crop (cv2.warpAffine-compatible inverse mapping) + normalize +
// HWC->NCHW into the NPU buffer. Same sampling as the RKNN backend's
// warp_bilinear: dst pixel (x, y) maps to tl + x * du / target + y * dv / target,
// bilinear with constant-zero border.
void warp_to_npu(const uint8_t* src, int sw, int sh, int sstride,
                 int format, float tlx, float tly, float dux, float duy,
                 float dvx, float dvy, float* npu_buf, int target) {
    const int HW = target * target;
    const float inv = 1.f / 255.f;
    // Per-pixel steps of the inverse affine (full edge vectors / target).
    const float stx = dux / target, sty = duy / target;
    const float svx = dvx / target, svy = dvy / target;
    // BGR source: sample R/B swapped so the tensor is always RGB.
    const int r_off = (format == 1) ? 2 : 0;
    const int b_off = (format == 1) ? 0 : 2;
    float* ch0 = npu_buf;
    float* ch1 = npu_buf + HW;
    float* ch2 = npu_buf + 2 * HW;
    for (int y = 0; y < target; ++y) {
        float sxf = tlx + y * svx, syf = tly + y * svy;
        float* o0 = ch0 + (size_t)y * target;
        float* o1 = ch1 + (size_t)y * target;
        float* o2 = ch2 + (size_t)y * target;
        for (int x = 0; x < target; ++x, sxf += stx, syf += sty) {
            const int x0 = (int)std::floor(sxf);
            const int y0 = (int)std::floor(syf);
            if (x0 < -1 || y0 < -1 || x0 >= sw || y0 >= sh) {
                o0[x] = o1[x] = o2[x] = 0.f;
                continue;
            }
            const float fx = sxf - x0, fy = syf - y0;
            const float w00 = (1 - fx) * (1 - fy), w01 = fx * (1 - fy);
            const float w10 = (1 - fx) * fy, w11 = fx * fy;
            const bool in00 = x0 >= 0 && y0 >= 0;
            const bool in01 = x0 + 1 < sw && y0 >= 0;
            const bool in10 = x0 >= 0 && y0 + 1 < sh;
            const bool in11 = x0 + 1 < sw && y0 + 1 < sh;
            const uint8_t* p00 = src + (size_t)y0 * sstride + x0 * 3;
            // Round through uint8 like cv2 before normalizing.
            float acc_r = 0.f, acc_g = 0.f, acc_b = 0.f;
            if (in00) { acc_r += w00 * p00[r_off]; acc_g += w00 * p00[1]; acc_b += w00 * p00[b_off]; }
            if (in01) { acc_r += w01 * p00[3 + r_off]; acc_g += w01 * p00[4]; acc_b += w01 * p00[3 + b_off]; }
            if (in10) { acc_r += w10 * p00[sstride + r_off]; acc_g += w10 * p00[sstride + 1]; acc_b += w10 * p00[sstride + b_off]; }
            if (in11) { acc_r += w11 * p00[sstride + 3 + r_off]; acc_g += w11 * p00[sstride + 4]; acc_b += w11 * p00[sstride + 3 + b_off]; }
            o0[x] = norm8(acc_r, inv);
            o1[x] = norm8(acc_g, inv);
            o2[x] = norm8(acc_b, inv);
        }
    }
}

struct Det {
    float score;
    float box[4];
    float kps[14];
};

std::vector<Det> weighted_nms(std::vector<Det>& dets, float thr) {
    std::sort(dets.begin(), dets.end(), [](const Det& a, const Det& b){ return a.score > b.score; });
    std::vector<Det> out;
    while (!dets.empty()) {
        Det top = dets[0];
        std::vector<Det> cluster, rest;
        for (size_t i = 1; i < dets.size(); i++) {
            if (box_iou(top.box, dets[i].box) > thr) cluster.push_back(dets[i]);
            else rest.push_back(dets[i]);
        }
        if (!cluster.empty()) {
            // Average only the cluster (not including top), matching Python
            float total = 0;
            for (auto& d : cluster) total += d.score;
            
            if (total > 0) {
                // Reset top box and keypoints to zero
                for (int j = 0; j < 4; j++) top.box[j] = 0;
                for (int j = 0; j < 14; j++) top.kps[j] = 0;
                
                // Compute weighted average of cluster only
                for (auto& d : cluster) {
                    float w = d.score / total;
                    for (int j = 0; j < 4; j++) top.box[j] += w * d.box[j];
                    for (int j = 0; j < 14; j++) top.kps[j] += w * d.kps[j];
                }
            }
        }
        out.push_back(top);
        dets = std::move(rest);
    }
    return out;
}

}  // namespace

// ----------------------------------------------------------------------- init
AeroHandMtnnPipeline::AeroHandMtnnPipeline(const std::string& palm_model,
                                           const std::string& hand_model,
                                           int num_hands)
    : num_hands_(std::max(1, num_hands)) {
    mtnn_set_log_level(MTNN_LOG_ERR);
    init_model(palm_, palm_model);
    init_model(hand_, hand_model);

    // Generate canonical MediaPipe SSD anchors for palm_detection 192 (2016 x 2).
    // strides [8,16,16,16]; consecutive equal strides each add 2 anchors per cell.
    {
        const int strides[4] = {8, 16, 16, 16};
        int layer = 0;
        while (layer < 4) {
            int count = 0;
            int last = layer;
            while (last < 4 && strides[last] == strides[layer]) { count += 2; last++; }
            int fm = (PALM_SZ + strides[layer] - 1) / strides[layer];  // ceil
            for (int y = 0; y < fm; y++) {
                for (int x = 0; x < fm; x++) {
                    for (int k = 0; k < count; k++) {
                        float cx = (x + 0.5f) / fm;
                        float cy = (y + 0.5f) / fm;
                        anchors_.push_back({cx, cy, 1.0f, 1.0f});
                    }
                }
            }
            layer = last;
        }
    }
}

AeroHandMtnnPipeline::~AeroHandMtnnPipeline() { release(); }

void AeroHandMtnnPipeline::init_model(Model& m, const std::string& path) {
    auto buf = read_file(path);
    mtnn_work_mode_t wm{};
    int ret = mtnn_init(&m.mgr, buf.data(), buf.size(), &wm);
    if (ret != MTNN_SUCC) throw std::runtime_error("mtnn_init failed: " + std::to_string(ret));

    mtnn_input_output_num ionum{};
    mtnn_get(m.mgr, MTNN_GET_IN_OUT_NUM, &ionum, sizeof(ionum));

    m.input_mems.resize(ionum.n_input);
    m.input_attrs.resize(ionum.n_input);
    for (uint32_t i = 0; i < ionum.n_input; i++) {
        m.input_attrs[i].index = i;
        mtnn_get(m.mgr, MTNN_GET_INPUT_ATTR, &m.input_attrs[i], sizeof(mtnn_tensor_attr));
        mtnn_inputs_get(m.mgr, 1, &m.input_mems[i]);
    }
    m.output_mems.resize(ionum.n_output);
    m.output_attrs.resize(ionum.n_output);
    m.output_ptrs.resize(ionum.n_output);
    for (uint32_t i = 0; i < ionum.n_output; i++) {
        m.output_attrs[i].index = i;
        mtnn_get(m.mgr, MTNN_GET_OUTPUT_ATTR, &m.output_attrs[i], sizeof(mtnn_tensor_attr));
        mtnn_outputs_get(m.mgr, 1, &m.output_mems[i]);
        m.output_ptrs[i] = (float*)m.output_mems[i].logical_addr;
    }
}

void AeroHandMtnnPipeline::destroy_model(Model& m) {
    if (m.mgr) { mtnn_destroy(m.mgr); m.mgr = 0; }
}

void AeroHandMtnnPipeline::release() {
    if (released_) return;
    destroy_model(palm_); destroy_model(hand_);
    released_ = true;
}

void AeroHandMtnnPipeline::add_stage(const char* name, double ms, double) {
    auto& s = stage_[name]; s.count++; s.total_ms += ms;
}

// -------------------------------------------------------------- palm detection
std::vector<AeroHandMtnnPipeline::Roi>
AeroHandMtnnPipeline::detect_palms(const uint8_t* data, int width, int height,
                                   int stride_bytes, int format) {
    auto t0 = Clock::now();

    float* npu_in = (float*)palm_.input_mems[0].logical_addr;
    int pad_x, pad_y; float scale;
    letterbox_to_npu(data, width, height, stride_bytes, format,
                     npu_in, PALM_SZ, pad_x, pad_y, scale);

    mtnn_input inp{};
    inp.index = 0; inp.buf = npu_in;
    inp.size = palm_.input_mems[0].size;
    inp.pass_through = 1;
    inp.type = MTNN_TENSOR_FLOAT32; inp.fmt = MTNN_TENSOR_NCHW;
    mtnn_inputs_set(palm_.mgr, 1, &inp);
    mtnn_inference(palm_.mgr, nullptr);
    mtnn_outputs_get(palm_.mgr, palm_.output_mems.size(), palm_.output_mems.data());
    for (size_t i = 0; i < palm_.output_ptrs.size(); i++)
        palm_.output_ptrs[i] = (float*)palm_.output_mems[i].logical_addr;

    add_stage("palm_preproc_npu", ms_since(t0), 0);
    auto t1 = Clock::now();

    float* reg = palm_.output_ptrs[0];
    float* sc = palm_.output_ptrs[1];
    int na = (int)anchors_.size();
    // Compute regressor stride: total elements / num_anchors
    int reg_total = (int)palm_.output_attrs[0].n_elems;
    int reg_stride = reg_total / na;  // should be 18
    if (reg_stride < 18) reg_stride = 18;  // safety
    float inv_p = 1.f / PALM_SZ;

    std::vector<Det> dets;
    for (int i = 0; i < na; i++) {
        float s = sigmoidf(sc[i]);
        if (s <= PALM_THR) continue;
        const float* r = reg + i * reg_stride;
        // Standard MediaPipe decode: offset/192 + anchor_center, size = offset/192
        float cx = r[0]*inv_p + anchors_[i][0];
        float cy = r[1]*inv_p + anchors_[i][1];
        float bw = r[2]*inv_p;
        float bh = r[3]*inv_p;
        Det d; d.score = s;
        d.box[0]=cx-bw/2; d.box[1]=cy-bh/2; d.box[2]=cx+bw/2; d.box[3]=cy+bh/2;
        for (int k=0;k<7;k++) {
            d.kps[2*k]   = r[4+2*k]*inv_p + anchors_[i][0];
            d.kps[2*k+1] = r[5+2*k]*inv_p + anchors_[i][1];
        }
        dets.push_back(d);
    }

    auto nms_out = weighted_nms(dets, NMS_IOU);
    add_stage("palm_decode", ms_since(t1), 0);

    std::vector<Roi> rois;
    for (auto& det : nms_out) {
        float bx0=det.box[0]*PALM_SZ, by0=det.box[1]*PALM_SZ;
        float bx1=det.box[2]*PALM_SZ, by1=det.box[3]*PALM_SZ;
        float kx0=det.kps[0]*PALM_SZ, ky0=det.kps[1]*PALM_SZ;
        float kx2=det.kps[4]*PALM_SZ, ky2=det.kps[5]*PALM_SZ;
        float fx0=(bx0-pad_x)/scale, fy0=(by0-pad_y)/scale;
        float fx1=(bx1-pad_x)/scale, fy1=(by1-pad_y)/scale;
        float wx=(kx0-pad_x)/scale, wy=(ky0-pad_y)/scale;
        float mx=(kx2-pad_x)/scale, my=(ky2-pad_y)/scale;
        float rot = rotation_of(wx, wy, mx, my);
        float vx=-std::sin(rot), vy=std::cos(rot);
        float cx=(fx0+fx1)/2.f + (-0.5f)*(fy1-fy0)*vx;
        float cy=(fy0+fy1)/2.f + (-0.5f)*(fy1-fy0)*vy;
        float sz = std::max(fx1-fx0, fy1-fy0) * 2.6f;
        rois.push_back({cx, cy, sz, rot});
    }
    return rois;
}

// ------------------------------------------------------------- hand landmark
std::vector<HandResult>
AeroHandMtnnPipeline::run_hand(const uint8_t* data, int width, int height,
                               int stride_bytes, int format, const Roi& roi) {
    auto t0 = Clock::now();

    float cos_r=std::cos(roi.rot), sin_r=std::sin(roi.rot);
    float half = roi.size/2.f;
    float ux=cos_r*half, uy=sin_r*half;
    float vx=-sin_r*half, vy=cos_r*half;
    float tlx = roi.cx-ux-vx, tly = roi.cy-uy-vy;
    float dux = (roi.cx+ux-vx) - tlx;
    float duy = (roi.cy+uy-vy) - tly;
    float dvx = (roi.cx-ux+vx) - tlx;
    float dvy = (roi.cy-uy+vy) - tly;
    float* npu_in = (float*)hand_.input_mems[0].logical_addr;
    warp_to_npu(data, width, height, stride_bytes, format,
                tlx, tly, dux, duy, dvx, dvy, npu_in, HAND_SZ);

    mtnn_input inp{};
    inp.index = 0; inp.buf = npu_in;
    inp.size = hand_.input_mems[0].size;
    inp.pass_through = 1;
    inp.type = MTNN_TENSOR_FLOAT32; inp.fmt = MTNN_TENSOR_NCHW;
    mtnn_inputs_set(hand_.mgr, 1, &inp);
    mtnn_inference(hand_.mgr, nullptr);
    mtnn_outputs_get(hand_.mgr, hand_.output_mems.size(), hand_.output_mems.data());
    for (size_t i = 0; i < hand_.output_ptrs.size(); i++)
        hand_.output_ptrs[i] = (float*)hand_.output_mems[i].logical_addr;

    add_stage("hand_preproc_npu", ms_since(t0), 0);

    // MediaPipe hand outputs: [lm(63) pixel-space, score(1), handed(1), world(63)]
    float* lm = hand_.output_ptrs[0];
    float score_val = hand_.output_ptrs[1][0];
    float handed_val = hand_.output_ptrs[2][0];
    float* world = hand_.output_ptrs[3];

    float prob = (score_val>=0.f && score_val<=1.f) ? score_val : sigmoidf(score_val);
    if (prob < PRESENCE_THR) return {};

    HandResult r; r.score = prob;
    // MediaPipe convention (handedness.txt): raw score = P("Left")
    float left_p = (handed_val>=0.f && handed_val<=1.f) ? handed_val : sigmoidf(handed_val);
    r.is_left = left_p > 0.5f;

    // Project landmarks (already in pixel space [0,HAND_SZ]) back to frame
    float sx0=roi.cx-ux-vx, sy0=roi.cy-uy-vy;
    for (int i=0;i<21;i++) {
        float lx=lm[i*3], ly=lm[i*3+1], lz=lm[i*3+2];
        float u=lx/HAND_SZ, v=ly/HAND_SZ;
        float px = sx0 + u*dux + v*dvx;
        float py = sy0 + u*duy + v*dvy;
        r.image[i*3] = px/width;
        r.image[i*3+1] = py/height;
        r.image[i*3+2] = lz/HAND_SZ * (roi.size/width);
        // world landmarks rotated by roi.rot
        float wx=world[i*3], wy=world[i*3+1], wz=world[i*3+2];
        r.world[i*3] = wx*cos_r - wy*sin_r;
        r.world[i*3+1] = wx*sin_r + wy*cos_r;
        r.world[i*3+2] = wz;
    }
    return {r};
}

// ------------------------------------------------------------------- process
std::vector<HandResult>
AeroHandMtnnPipeline::process(const uint8_t* data, int width, int height,
                              int stride_bytes, int format) {
    if (stride_bytes <= 0) stride_bytes = width * 3;

    std::vector<Roi> rois = rois_;
    if ((int)rois.size() < num_hands_) {
        for (auto& roi : detect_palms(data, width, height, stride_bytes, format)) {
            bool dup = false;
            for (auto& t : rois)
                if (std::hypot(roi.cx-t.cx, roi.cy-t.cy) < 0.5f*std::min(roi.size,t.size))
                    { dup=true; break; }
            if (!dup) { rois.push_back(roi); if ((int)rois.size()>=num_hands_) break; }
        }
    }

    std::vector<HandResult> results;
    std::vector<Roi> next_rois;
    for (int i=0; i<std::min((int)rois.size(), num_hands_); i++) {
        auto hands = run_hand(data, width, height, stride_bytes, format, rois[i]);
        if (hands.empty()) continue;
        results.push_back(hands[0]);

        const auto& img = hands[0].image;
        float tip_x = (img[5*3]+img[13*3])/2.f, tip_y = (img[5*3+1]+img[13*3+1])/2.f;
        tip_x = (tip_x+img[9*3])/2.f; tip_y = (tip_y+img[9*3+1])/2.f;
        float wr_x=img[0]*width, wr_y=img[1]*height;
        float rot = rotation_of(wr_x, wr_y, tip_x*width, tip_y*height);
        float cos_r=std::cos(rot), sin_r=std::sin(rot);
        float a_min=1e9f,a_max=-1e9f,b_min=1e9f,b_max=-1e9f;
        for (int j=0;j<12;j++) {
            int idx=ROI_LM[j];
            float px=img[idx*3]*width, py=img[idx*3+1]*height;
            float a=px*cos_r+py*sin_r, b=-px*sin_r+py*cos_r;
            a_min=std::min(a_min,a); a_max=std::max(a_max,a);
            b_min=std::min(b_min,b); b_max=std::max(b_max,b);
        }
        float ca=(a_min+a_max)/2, cb=(b_min+b_max)/2;
        float cx=ca*cos_r-cb*sin_r, cy=ca*sin_r+cb*cos_r;
        float sz=std::max(a_max-a_min,b_max-b_min)*2.f;
        float vr_x=-std::sin(rot), vr_y=std::cos(rot);
        cx += (-0.1f)*(b_max-b_min)*vr_x;
        cy += (-0.1f)*(b_max-b_min)*vr_y;
        next_rois.push_back({cx, cy, sz, rot});
    }
    rois_ = next_rois;
    return results;
}

}  // namespace aerohand_mtnn
