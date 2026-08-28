// pybind11 bindings for the RGA + RKNN zero-copy Aero Hand pipeline.
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "aero_hand_pipeline.hpp"

namespace py = pybind11;
using aerohand::AeroHandPipeline;
using aerohand::HandResult;

namespace {

py::list to_python(const std::vector<HandResult>& hands) {
    py::list out;
    for (const HandResult& hand : hands) {
        py::array_t<float> image({21, 3});
        py::array_t<float> world({21, 3});
        std::memcpy(image.mutable_data(), hand.image.data(), sizeof(hand.image));
        std::memcpy(world.mutable_data(), hand.world.data(), sizeof(hand.world));
        py::dict item;
        item["image"] = image;   // 21 x (x_norm, y_norm, z)
        item["world"] = world;   // 21 x (x, y, z)
        item["score"] = hand.score;
        item["label"] = hand.is_left ? "Left" : "Right";
        item["label_score"] = hand.is_left ? hand.score : hand.score;  // presence
        out.append(item);
    }
    return out;
}

}  // namespace

PYBIND11_MODULE(aero_hand_rga, m) {
    m.doc() = "Aero Hand gesture pipeline: RGA preprocessing + RKNN zero-copy NPU inference";

    py::class_<aerohand::CacheStats>(m, "CacheStats")
        .def_readonly("imports", &aerohand::CacheStats::imports)
        .def_readonly("hits", &aerohand::CacheStats::hits)
        .def_readonly("entries", &aerohand::CacheStats::entries)
        .def_readonly("fallbacks", &aerohand::CacheStats::fallbacks)
        .def_readonly("refusals", &aerohand::CacheStats::refusals)
        .def_readonly("staged", &aerohand::CacheStats::staged);

    py::class_<AeroHandPipeline>(m, "AeroHandPipeline")
        .def(py::init<const std::string&, const std::string&, int>(),
             py::arg("palm_model"), py::arg("hand_model"), py::arg("num_hands") = 1)
        .def(
            "clone",
            [](const AeroHandPipeline& self) { return self.clone().release(); },
            py::return_value_policy::take_ownership,
            "Weight-sharing duplicate (rknn_dup_context) with its own io/RGA/"
            "tracking state; use one instance per worker thread.")
        .def(
            "process",
            [](AeroHandPipeline& self, py::array_t<uint8_t, py::array::c_style> frame,
               int format) {
                py::buffer_info info = frame.request();
                if (info.ndim != 3 || info.shape[2] != 3)
                    throw std::invalid_argument("frame must be HxWx3 uint8");
                const int height = static_cast<int>(info.shape[0]);
                const int width = static_cast<int>(info.shape[1]);
                const int stride = static_cast<int>(info.strides[0]);
                const uint8_t* data = static_cast<const uint8_t*>(info.ptr);
                std::vector<HandResult> hands;
                {
                    py::gil_scoped_release release;
                    hands = self.process(data, width, height, stride, format);
                }
                return to_python(hands);
            },
            py::arg("frame"), py::arg("format") = 0,
            "Process one frame (HxWx3 uint8, zero-copy via the array's data "
            "pointer). format: 0=RGB888, 1=BGR888.")
        .def(
            "process_ptr",
            [](AeroHandPipeline& self, uintptr_t addr, int width, int height,
               int stride_bytes, int format) {
                std::vector<HandResult> hands;
                {
                    py::gil_scoped_release release;
                    hands = self.process(reinterpret_cast<const uint8_t*>(addr),
                                         width, height, stride_bytes, format);
                }
                return to_python(hands);
            },
            py::arg("addr"), py::arg("width"), py::arg("height"),
            py::arg("stride_bytes"), py::arg("format") = 0,
            "Process one frame given a raw pointer (for upstream memory pools).")
        .def("stats", &AeroHandPipeline::stats)
        .def("timings",
             [](const AeroHandPipeline& self) {
                 py::dict out;
                 for (const auto& [name, stat] : self.timings()) {
                     py::dict item;
                     item["count"] = stat.count;
                     item["total_ms"] = stat.total_ms;
                     item["avg_ms"] = stat.count ? stat.total_ms / stat.count : 0.0;
                     item["avg_cpu_ms"] = stat.count ? stat.cpu_ms / stat.count : 0.0;
                     out[py::str(name)] = item;
                 }
                 return out;
             },
             "Per-stage accumulated timings (count/total_ms/avg_ms).")
        .def("reset_timings", &AeroHandPipeline::reset_timings)
        .def("clear_cache", &AeroHandPipeline::clear_cache)
        .def("reset_tracking", &AeroHandPipeline::reset_tracking)
        .def("release", &AeroHandPipeline::release);
}
