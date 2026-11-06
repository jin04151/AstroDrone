# Jetson Nano IMX219 -> C++ TensorRT YOLO -> UDP 14560

Independent real-camera vision component for AstroDrone. **No MAVLink commands are implemented here.**

## Contents
- `frame_source.*`, `tensorrt_engine.*`, `common.hpp`: copied from the user-supplied `astrodrone_YOLO` reference (`feature/control-test-HJ`) with no algorithm changes. The frame source uses `nvarguscamerasrc sensor-id=0 wbmode=3`, 640x480 @ 30 fps.
- `main.cpp`, `udp_detection.hpp`: new minimal adapter to publish best class-0 detection to existing `VisionReceiver`.
- `CMakeLists.txt`: standalone Jetson build, independent of `~/control` CMake.

## Build on Jetson (not on Windows WSL)
```bash
cd ~/drone_vision/tensorrt
cmake -S . -B build
cmake --build build -j2
```

## Run (examples; replace model paths)
```bash
./build/jetson_yolo_trt \
  --onnx ../models/best_v5.onnx \
  --engine ../models/best_v5_jetson.engine \
  --target-class 0 --conf 0.6
```

The provided reference also contains `YOLO_MODEL/prototype.onnx`; it may have different class or output conventions than the separate SITL `best_v5.onnx`. Do **not** reuse `0812best.engine` across a different GPU/TensorRT version. Use an ONNX file that is compatible with `YoloTrt`'s supported bindings and validate decoded boxes before relying on it. Engine is built/cached by the reference backend when absent.

## UDP payload

Send every inference frame to `127.0.0.1:14560`:

- Detected: `1,0,0.9231,320,245,640,480`
- Not detected: `0,-1,0.0000,0,0,640,480`

Image coordinates are original frame pixel centers (origin upper-left), not the reference project's center-offset `target.x_px/y_px`. This is a crucial compatibility distinction.

## Caveats
- Not hardware-tested yet. Camera OpenCV GStreamer, CUDA/TensorRT and model format must be validated on the actual Jetson Nano.
- Original TensorRT backend uses TensorRT 8-style `getNbBindings()` and `enqueueV2()` APIs; these are not compatible with newer TRT 10 without porting.
- Nano performance/latency may be below 30 FPS; measure inference latency. Do not use these outputs for flight without independent safety checks.
- REAL `control_real` remains telemetry-only. It currently does **not** attach `VisionReceiver` or use autonomous flight commands.
- `127.0.0.1` assumes the vision process and receiving C++ process run on the same Jetson.
