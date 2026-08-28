#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <cstring>

#include "aero_hand_mtnn.hpp"

namespace py = pybind11;
using aerohand_mtnn::AeroHandMtnnPipeline;
using aerohand_mtnn::HandResult;

namespace {

py::list to_python(const std::vector<HandResult>& hands) {
    py::list out;
    for (const HandResult& hand : hands) {
        py::array_t<float> image({21, 3});
        py::array_t<float> world({21, 3});
        std::memcpy(image.mutable_data(), hand.image.data(), sizeof(hand.image));
        std::memcpy(world.mutable_data(), hand.world.data(), sizeof(hand.world));
        py::dict item;
        item["image"] = image;
        item["world"] = world;
        item["score"] = hand.score;
        item["label"] = hand.is_left ? "Left" : "Right";
        out.append(item);
    }
    return out;
}

}  // namespace

PYBIND11_MODULE(aero_hand_mtnn, m) {
    m.doc() = "Aero Hand gesture pipeline: MTNN zero-copy NPU inference for Moore Threads E300";

    py::class_<AeroHandMtnnPipeline>(m, "AeroHandPipeline")
        .def(py::init<const std::string&, const std::string&, int>(),
             py::arg("palm_model"), py::arg("hand_model"), py::arg("num_hands") = 1)
        .def(
            "process",
            [](AeroHandMtnnPipeline& self, py::array_t<uint8_t, py::array::c_style> frame,
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
            "Process one frame (HxWx3 uint8). format: 0=RGB, 1=BGR.")
        .def(
            "process_ptr",
            [](AeroHandMtnnPipeline& self, uintptr_t addr, int width, int height,
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
            "Process one frame from raw pointer (for memory pool integration).")
        .def("timings",
             [](const AeroHandMtnnPipeline& self) {
                 py::dict out;
                 for (const auto& [name, stat] : self.timings()) {
                     py::dict item;
                     item["count"] = stat.count;
                     item["total_ms"] = stat.total_ms;
                     item["avg_ms"] = stat.count ? stat.total_ms / stat.count : 0.0;
                     out[py::str(name)] = item;
                 }
                 return out;
             })
        .def("reset_timings", &AeroHandMtnnPipeline::reset_timings)
        .def("reset_tracking", &AeroHandMtnnPipeline::reset_tracking)
        .def("release", &AeroHandMtnnPipeline::release);
}
