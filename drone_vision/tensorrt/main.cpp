// AstroDrone Jetson camera -> TensorRT YOLO -> VisionReceiver UDP bridge.
// This executable never opens Pixhawk ports and cannot transmit MAVLink.
#include "frame_source.hpp"
#include "tensorrt_engine.hpp"
#include "udp_detection.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
volatile std::sig_atomic_t running = 1;
void stop_handler(int) { running = 0; }

struct Options {
    std::string onnx;
    std::string engine;
    std::string host = "127.0.0.1";
    int port = 14560;
    int target_class = 0;
    float confidence = 0.6f;
    float iou = 0.45f;
    bool fp16 = false;
};

void usage(const char* executable) {
    std::cerr << "Usage: " << executable << " --onnx MODEL.onnx --engine MODEL.engine\n"
              << "  [--host 127.0.0.1] [--port 14560] [--target-class 0]\n"
              << "  [--conf 0.6] [--iou 0.45] [--fp16]\n";
}

Options parse_args(int argc, char** argv) {
    Options out;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if (key == "--fp16") { out.fp16 = true; continue; }
        if (i + 1 >= argc) throw std::runtime_error("Missing argument after " + key);
        const std::string value = argv[++i];
        if (key == "--onnx") out.onnx = value;
        else if (key == "--engine") out.engine = value;
        else if (key == "--host") out.host = value;
        else if (key == "--port") out.port = std::stoi(value);
        else if (key == "--target-class") out.target_class = std::stoi(value);
        else if (key == "--conf") out.confidence = std::stof(value);
        else if (key == "--iou") out.iou = std::stof(value);
        else throw std::runtime_error("Unknown option " + key);
    }
    if (out.onnx.empty() || out.engine.empty()) throw std::runtime_error("--onnx and --engine required");
    if (out.port < 1 || out.port > 65535 || out.target_class < 0 ||
        !std::isfinite(out.confidence) || out.confidence < 0.0f || out.confidence > 1.0f ||
        !std::isfinite(out.iou) || out.iou < 0.0f || out.iou > 1.0f)
        throw std::runtime_error("Invalid port/class/conf/iou value");
    return out;
}
}

int main(int argc, char** argv) {
    std::signal(SIGINT, stop_handler);
    std::signal(SIGTERM, stop_handler);
    try {
        const Options opt = parse_args(argc, argv);
        std::cout << "[VISION] REAL camera TensorRT bridge; MAVLink/flight TX not present\n"
                  << "[VISION] UDP " << opt.host << ':' << opt.port << " target class " << opt.target_class << '\n';
        UdpDetectionSender udp(opt.host, opt.port);
        // From supplied reference project: GStreamer nvarguscamerasrc sensor-id=0 wbmode=3
        std::unique_ptr<FrameSource> camera = make_imx219_frame_source();
        YoloTrt model(opt.onnx, opt.engine, opt.fp16);

        unsigned long long frames = 0;
        while (running) {
            cv::Mat frame;
            FrameMetadata meta;
            std::string error;
            if (!camera->read(frame, meta, error)) {
                // A camera failure must not leave a stale detection as target.
                (void)udp.send(false, -1, 0, 0, 0, 640, 480);
                std::cerr << "[CAMERA] " << error << '\n';
                std::this_thread::sleep_for(std::chrono::milliseconds(250));
                continue;
            }

            const auto detections = model.infer(frame, opt.confidence, opt.iou);
            const Detection* best = nullptr;
            for (const auto& d : detections) {
                if (d.cls != opt.target_class || !std::isfinite(d.conf) || d.conf < opt.confidence)
                    continue;
                if (!best || d.conf > best->conf) best = &d;
            }

            bool ok = false;
            if (best) {
                const int cx = std::clamp(static_cast<int>(std::lround((best->x1 + best->x2) / 2.0f)), 0, frame.cols - 1);
                const int cy = std::clamp(static_cast<int>(std::lround((best->y1 + best->y2) / 2.0f)), 0, frame.rows - 1);
                ok = udp.send(true, best->cls, best->conf, cx, cy, frame.cols, frame.rows);
            } else {
                ok = udp.send(false, -1, 0, 0, 0, frame.cols, frame.rows);
            }
            if (!ok) std::cerr << "[UDP] failed to publish detection\n";
            if (++frames % 30 == 0)
                std::cout << "[VISION] frames=" << frames << " target=" << (best ? "YES" : "NO") << '\n';
        }
        // Notify receiver of target loss upon clean shutdown.
        (void)udp.send(false, -1, 0, 0, 0, 640, 480);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << '\n';
        usage(argv[0]);
        return 1;
    }
}
