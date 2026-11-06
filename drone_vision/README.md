
# Drone Vision

AstroDrone의 YOLO 기반 객체 검출 모듈입니다.

Gazebo SITL 환경과 실제 Jetson Nano 환경의
영상 입력 및 추론 방식을 분리하여 관리합니다.

## Architecture

### SITL

Gazebo Camera
→ Python YOLO (ONNX)
→ UDP 14560
→ C++ VisionReceiver

### REAL

Jetson IMX219 Camera
→ C++ TensorRT YOLO
→ UDP 14560
→ C++ VisionReceiver

## Files

| File | Description |
|---|---|
| yolo_bridge.py | Gazebo 카메라 영상 및 YOLO 검출 |
| jetson_yolo.py | Jetson CSI 카메라 Python 테스트 |
| tensorrt/ | Jetson용 C++ TensorRT 추론 |

## Detection Protocol

UDP destination: 127.0.0.1:14560

Message format:

detected,class_id,confidence,cx,cy,width,height

Example:

1,0,0.9231,320,245,640,480

- detected: 1 = detected, 0 = not detected
- class_id: object class
- confidence: detection confidence
- cx, cy: bounding box center in pixels
- width, height: original image resolution

## Model

Current model: best_v5.onnx

Input: [1, 3, 640, 640]
Output: [1, 25200, 6]

Target class: 0 (white_box)

Model weights and TensorRT engines are not
included in this repository.

## Status

- [x] Gazebo YOLO integration
- [x] SITL UDP detection interface
- [x] Jetson camera input code
- [x] TensorRT inference code
- [ ] Jetson TensorRT build verification
- [ ] Real IMX219 camera verification
- [ ] Real-time inference performance testing
- [ ] Pixhawk hardware integration testing

The real-hardware components are not yet
validated on a physical drone.
