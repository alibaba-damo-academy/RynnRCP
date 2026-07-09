// Copyright 2026 RynnRCP Authors. All rights reserved.
// pybind11 bindings for rynnrcp_core native acceleration.
// Exposes: RingBuffer, Transport, ChannelManager, TFTree to Python.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include <cstdint>
#include <limits>

#include "ring_buffer.h"
#include "transport.h"
#include "channel_manager.h"
#include "tf.h"

// Logging support
#include "log.h"

namespace py = pybind11;
using namespace rynnrcp::core;
using namespace rynnrcp::native::log;

static py::object index_or_minus_one(uint64_t index) {
    if (index == std::numeric_limits<uint64_t>::max()) {
        return py::int_(-1);
    }
    return py::int_(index);
}

struct PyBufferGuard {
    std::vector<Py_buffer> views;

    ~PyBufferGuard() {
        for (auto& view : views) {
            PyBuffer_Release(&view);
        }
    }
};

// ---------------------------------------------------------------------------
// PythonLogSink - routes C++ log records through Python logging
// ---------------------------------------------------------------------------
class PythonLogSink : public LogSink {
public:
    explicit PythonLogSink(py::object logger) : py_logger_(std::move(logger)) {}

    void Write(const LogRecord& record) override {
        py::gil_scoped_acquire acquire;
        try {
            // Map C++ LogLevel to Python logging level
            int py_level;
            switch (record.level) {
                case LogLevel::kDebug: py_level = 10; break;  // logging.DEBUG
                case LogLevel::kInfo:  py_level = 20; break;  // logging.INFO
                case LogLevel::kWarn:  py_level = 30; break;  // logging.WARNING
                case LogLevel::kError: py_level = 40; break;  // logging.ERROR
                case LogLevel::kFatal: py_level = 50; break;  // logging.CRITICAL
                default:               py_level = 20; break;
            }

            // Format: [MODULE] FILE:LINE MESSAGE
            char buf[640];
            snprintf(buf, sizeof(buf), "[%s] %s:%d %s",
                     record.module, record.file, record.line, record.message);

            py_logger_.attr("log")(py_level, buf);
        } catch (py::error_already_set& e) {
            // Swallow Python exceptions to avoid crashing C++ code
            e.restore();
            PyErr_Clear();
        }
    }

private:
    py::object py_logger_;
};

static std::shared_ptr<PythonLogSink> g_python_log_sink;

PYBIND11_MODULE(rynnrcp_core_native, m) {
    m.doc() = "RynnRCP C++ native acceleration module";

    // ======================================================================
    // RingBuffer
    // ======================================================================
    py::class_<RingBuffer>(m, "RingBuffer")
        .def_static("create", [](const std::string& name, size_t slot_size, size_t slot_count) {
            return RingBuffer::Create(name, slot_size, slot_count);
        }, py::arg("name"), py::arg("slot_size"), py::arg("slot_count") = 16)
        .def_static("open", [](const std::string& name, size_t slot_size, size_t slot_count) {
            return RingBuffer::Open(name, slot_size, slot_count);
        }, py::arg("name"), py::arg("slot_size"), py::arg("slot_count") = 16)
        .def("write", [](RingBuffer& self, py::bytes data) {
            std::string s = data;
            return self.Write(s.data(), s.size());
        })
        .def("write_envelope_parts", [](RingBuffer& self, size_t payload_size, py::sequence parts) {
            std::vector<std::pair<const uint8_t*, size_t>> native_parts;
            std::vector<py::object> keep_alive;
            PyBufferGuard buffer_guard;
            native_parts.reserve(py::len(parts));
            keep_alive.reserve(py::len(parts));

            py::object bytes_ctor = py::module_::import("builtins").attr("bytes");
            for (py::handle item : parts) {
                py::object obj = py::reinterpret_borrow<py::object>(item);

                char* data = nullptr;
                Py_ssize_t size = 0;
                if (PyBytes_Check(obj.ptr()) && PyBytes_AsStringAndSize(obj.ptr(), &data, &size) == 0) {
                    keep_alive.emplace_back(obj);
                    native_parts.emplace_back(reinterpret_cast<const uint8_t*>(data), static_cast<size_t>(size));
                    continue;
                }

                if (PyByteArray_Check(obj.ptr())) {
                    keep_alive.emplace_back(obj);
                    native_parts.emplace_back(
                        reinterpret_cast<const uint8_t*>(PyByteArray_AS_STRING(obj.ptr())),
                        static_cast<size_t>(PyByteArray_GET_SIZE(obj.ptr())));
                    continue;
                }

                Py_buffer view;
                if (PyObject_GetBuffer(obj.ptr(), &view, PyBUF_CONTIG_RO) == 0) {
                    keep_alive.emplace_back(obj);
                    native_parts.emplace_back(
                        reinterpret_cast<const uint8_t*>(view.buf),
                        static_cast<size_t>(view.len));
                    buffer_guard.views.emplace_back(view);
                    continue;
                }
                PyErr_Clear();

                py::bytes bytes_obj(bytes_ctor(obj));
                if (PyBytes_AsStringAndSize(bytes_obj.ptr(), &data, &size) != 0) {
                    throw py::error_already_set();
                }
                keep_alive.emplace_back(bytes_obj);
                native_parts.emplace_back(reinterpret_cast<const uint8_t*>(data), static_cast<size_t>(size));
            }
            return self.WriteEnvelopeRawParts(payload_size, native_parts);
        }, py::arg("payload_size"), py::arg("parts"))
        .def("read", [](const RingBuffer& self, uint64_t index) -> py::object {
            std::vector<uint8_t> buf(self.slot_size());
            size_t len = buf.size();
            if (self.Read(index, buf.data(), &len)) {
                return py::bytes(reinterpret_cast<char*>(buf.data()), len);
            }
            return py::none();
        })
        .def("try_read", [](const RingBuffer& self, uint64_t index) -> py::object {
            std::vector<uint8_t> buf(self.slot_size());
            size_t len = buf.size();
            if (self.TryRead(index, buf.data(), &len)) {
                return py::bytes(reinterpret_cast<char*>(buf.data()), len);
            }
            return py::none();
        })
        .def("read_at", [](const RingBuffer& self, uint64_t index, size_t offset, size_t length) -> py::object {
            std::vector<uint8_t> buf(length);
            size_t len = buf.size();
            if (self.ReadAt(index, offset, length, buf.data(), &len)) {
                return py::bytes(reinterpret_cast<char*>(buf.data()), len);
            }
            return py::none();
        }, py::arg("index"), py::arg("offset"), py::arg("length"))
        .def("latest_index", [](const RingBuffer& self) {
            return index_or_minus_one(self.LatestIndex());
        })
        .def("oldest_index", [](const RingBuffer& self) {
            return index_or_minus_one(self.OldestIndex());
        })
        .def("write_count", &RingBuffer::WriteCount)
        .def("close", &RingBuffer::Close)
        .def("unlink", &RingBuffer::Unlink)
        .def("is_valid", &RingBuffer::IsValid)
        .def_property_readonly("slot_size", &RingBuffer::slot_size)
        .def_property_readonly("slot_count", &RingBuffer::slot_count);

    // ======================================================================
    // TransportLevel enum
    // ======================================================================
    py::enum_<TransportLevel>(m, "TransportLevel")
        .value("INTRA_THREAD", TransportLevel::kIntraThread)
        .value("INTRA_PROCESS", TransportLevel::kIntraProcess)
        .value("SHM", TransportLevel::kShm)
        .export_values();

    // ======================================================================
    // IntraProcessTransport
    // ======================================================================
    py::class_<IntraProcessTransport, std::shared_ptr<IntraProcessTransport>>(m, "IntraProcessTransport")
        .def(py::init<const std::string&, size_t, size_t>(),
             py::arg("name"), py::arg("msg_size"), py::arg("buffer_size") = 64)
        .def("publish", [](IntraProcessTransport& self, py::bytes data) {
            std::string s = data;
            self.Publish(s.data(), s.size());
        })
        .def("poll", [](IntraProcessTransport& self, int timeout_ms) -> py::object {
            std::vector<uint8_t> buf(4096);
            size_t len = buf.size();
            if (self.Poll(buf.data(), &len, timeout_ms)) {
                return py::bytes(reinterpret_cast<char*>(buf.data()), len);
            }
            return py::none();
        }, py::arg("timeout_ms") = 100)
        .def("close", &IntraProcessTransport::Close)
        .def("is_open", &IntraProcessTransport::IsOpen);

    // ======================================================================
    // ShmTransport
    // ======================================================================
    py::class_<ShmTransport, std::shared_ptr<ShmTransport>>(m, "ShmTransport")
        .def(py::init<const std::string&, size_t, size_t, bool>(),
             py::arg("name"), py::arg("msg_size"),
             py::arg("slot_count") = 16, py::arg("create") = true)
        .def("publish", [](ShmTransport& self, py::bytes data) {
            std::string s = data;
            self.Publish(s.data(), s.size());
        })
        .def("poll", [](ShmTransport& self, int timeout_ms) -> py::object {
            std::vector<uint8_t> buf(4096);
            size_t len = buf.size();
            if (self.Poll(buf.data(), &len, timeout_ms)) {
                return py::bytes(reinterpret_cast<char*>(buf.data()), len);
            }
            return py::none();
        }, py::arg("timeout_ms") = 100)
        .def("read_latest", [](ShmTransport& self) -> py::object {
            std::vector<uint8_t> buf(4096);
            size_t len = buf.size();
            if (self.ReadLatest(buf.data(), &len)) {
                return py::bytes(reinterpret_cast<char*>(buf.data()), len);
            }
            return py::none();
        })
        .def("close", &ShmTransport::Close)
        .def("unlink", &ShmTransport::Unlink)
        .def("is_open", &ShmTransport::IsOpen);

    // ======================================================================
    // ChannelManager
    // ======================================================================
    py::class_<Publisher>(m, "Publisher")
        .def("publish", [](Publisher& self, py::bytes data) {
            std::string s = data;
            self.Publish(s.data(), s.size());
        })
        .def_property_readonly("channel_name", &Publisher::channel_name);

    py::class_<Subscriber>(m, "Subscriber")
        .def("poll", [](Subscriber& self, int timeout_ms) -> py::object {
            std::vector<uint8_t> buf(4096);
            size_t len = buf.size();
            if (self.Poll(buf.data(), &len, timeout_ms)) {
                return py::bytes(reinterpret_cast<char*>(buf.data()), len);
            }
            return py::none();
        }, py::arg("timeout_ms") = 100)
        .def("read_latest", [](Subscriber& self) -> py::object {
            std::vector<uint8_t> buf(4096);
            size_t len = buf.size();
            if (self.ReadLatest(buf.data(), &len)) {
                return py::bytes(reinterpret_cast<char*>(buf.data()), len);
            }
            return py::none();
        })
        .def_property_readonly("channel_name", &Subscriber::channel_name);

    py::class_<ChannelManager>(m, "ChannelManager")
        .def_static("instance", &ChannelManager::Instance, py::return_value_policy::reference)
        .def_static("reset", &ChannelManager::Reset)
        .def("create_publisher", &ChannelManager::CreatePublisher,
             py::arg("name"), py::arg("msg_size"),
             py::arg("level") = TransportLevel::kIntraProcess)
        .def("create_subscriber", [](ChannelManager& self, const std::string& name,
                                      size_t msg_size, TransportLevel level) {
            return self.CreateSubscriber(name, msg_size, nullptr, level);
        }, py::arg("name"), py::arg("msg_size"),
           py::arg("level") = TransportLevel::kIntraProcess)
        .def("list_channels", &ChannelManager::ListChannels)
        .def("close_all", &ChannelManager::CloseAll);

    // ======================================================================
    // TF
    // ======================================================================
    py::class_<Transform>(m, "Transform")
        .def(py::init<>())
        .def(py::init([](const std::string& parent, const std::string& child,
                         std::array<double, 3> translation,
                         std::array<double, 4> rotation, double timestamp) {
            Transform t;
            t.parent = parent;
            t.child = child;
            t.translation = translation;
            t.rotation = rotation;
            t.timestamp = timestamp;
            return t;
        }), py::arg("parent"), py::arg("child"),
           py::arg("translation"), py::arg("rotation"), py::arg("timestamp"))
        .def_readwrite("parent", &Transform::parent)
        .def_readwrite("child", &Transform::child)
        .def_readwrite("translation", &Transform::translation)
        .def_readwrite("rotation", &Transform::rotation)
        .def_readwrite("timestamp", &Transform::timestamp);

    py::class_<TFTree>(m, "TFTree")
        .def(py::init<double>(), py::arg("history_duration") = 10.0)
        .def("set_transform", &TFTree::SetTransform)
        .def("lookup", &TFTree::Lookup,
             py::arg("target"), py::arg("source"), py::arg("time") = 0.0)
        .def("can_transform", &TFTree::CanTransform)
        .def("get_chain", &TFTree::GetChain)
        .def("all_frames", &TFTree::AllFrames);

    // Quaternion utility functions
    m.def("quat_normalize", &QuatNormalize);
    m.def("quat_multiply", &QuatMultiply);
    m.def("quat_inverse", &QuatInverse);
    m.def("quat_rotate_vec", &QuatRotateVec);
    m.def("quat_slerp", &QuatSlerp);
    m.def("compose_transforms", &ComposeTransforms);
    m.def("invert_transform", &InvertTransform);

    // ======================================================================
    // Logging bridge
    // ======================================================================
    m.def("set_python_log_sink", [](py::object logger) {
        // Remove previous Python sink if any
        if (g_python_log_sink) {
            RemoveLogSink(g_python_log_sink);
        }
        g_python_log_sink = std::make_shared<PythonLogSink>(logger);
        AddLogSink(g_python_log_sink);
    }, py::arg("logger"),
       "Route all C++ log output through a Python logging.Logger instance.");

    m.def("set_native_log_level", [](int level) {
        SetGlobalLogLevel(static_cast<LogLevel>(level));
    }, py::arg("level"),
       "Set C++ global log level (0=DEBUG, 1=INFO, 2=WARN, 3=ERROR, 4=FATAL).");
}
