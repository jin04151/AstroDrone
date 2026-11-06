# AstroDrone

**YOLO 객체 인식과 MAVLink 통신을 이용한 드론 자율 접근 및 하강 시스템**

AstroDrone은 카메라로 목표물을 인식하고, 인식 결과를 바탕으로 드론이 목표물에 접근하고 하강하도록 제어하는 프로젝트입니다.

현재 Gazebo와 ArduPilot SITL을 이용하여 자율 접근 및 하강 알고리즘을 개발하고 있으며, 추후 Jetson Nano와 Pixhawk 4 Mini를 실제 드론에 연결하는 것을 목표로 합니다.

시뮬레이션과 실제 하드웨어 환경을 분리하면서도, MAVLink 통신 및 제어 로직을 가능한 한 공통으로 사용할 수 있도록 구성하였습니다.

## 1. 시스템 구성

### 1.1 시뮬레이션 환경 (SITL)

```text
Gazebo Camera
      ↓
Python YOLO (ONNX)
      ↓
UDP 14560
      ↓
VisionReceiver
      ↓
FlightPhaseManager
      ↓
MavlinkClient
      ↓
MAVLink UDP 14552
      ↓
ArduPilot SITL
      ↓
Gazebo Drone
```

Gazebo에서 생성된 카메라 영상을 YOLO 모델로 분석하고, 검출된 목표물의 중심 좌표를 C++ 제어 프로그램으로 전달합니다.

제어 프로그램은 목표물의 위치와 드론의 고도 및 속도 정보를 이용해 비행 상태를 관리하고, MAVLink를 통해 ArduPilot SITL에 제어 명령을 전송합니다.

### 1.2 실제 하드웨어 환경 (REAL)

```text
IMX219 CSI Camera
      ↓
Jetson Nano
      ↓
C++ TensorRT YOLO
      ↓
UDP 14560
      ↓
VisionReceiver
      ↓
C++ Control System
      ↕
MAVLink UART
      ↕
Pixhawk 4 Mini
```

실제 환경에서는 Jetson Nano의 IMX219 카메라를 이용하여 영상을 수집하고, TensorRT로 YOLO 추론을 수행하도록 설계하였습니다.

Pixhawk와 Jetson은 유선 UART 통신으로 연결할 계획입니다.

**현재 REAL 프로그램은 MAVLink 상태 정보 수신 전용이며, 실제 기체에 비행 명령을 전송하는 기능은 구현하지 않았습니다.**

## 2. 주요 기능

### 2.1 YOLO 기반 목표물 검출

YOLO를 이용하여 카메라 영상에서 목표물을 검출합니다.

- 검출 대상: `white_box` (Class 0)
- 최소 Confidence: 0.6
- Bounding Box 중심 좌표 계산
- 연속 검출 결과 및 데이터 유효시간 확인
- UDP를 이용한 C++ 제어 프로그램 연동

검출 결과는 다음 형식으로 전송합니다.

```text
detected,class_id,confidence,cx,cy,width,height
```

예시:

```text
1,0,0.9231,320,245,640,480
```

### 2.2 수직 접근 및 하강 (VERTICAL)

목표물을 인식한 후 수평 이동을 통해 목표물 중심에 정렬하고, 정렬 상태를 유지하면서 수직 하강하는 방식입니다.

동작 순서:

1. AUTO 모드에서 목표물 탐색
2. 목표물 검출 확인
3. GUIDED 모드 전환 및 초기 호버링
4. 목표물 중심으로 수평 접근
5. 중심 정렬 및 안정화
6. 수직 하강하면서 위치 오차 보정
7. 상대고도 1m 이하에서 정지 속도 명령

### 2.3 사선 접근 및 하강 (DIAGONAL)

목표물을 인식한 후 수평 이동과 하강을 동시에 수행하는 방식입니다.

- 수평 명령 속도: 최대 0.2m/s
- 하강 명령 속도: 최대 0.2m/s
- 수평·하강 명령 속도를 이용한 45도 사선 접근
- YOLO 검출 결과에 따라 수평 이동 방향 갱신
- 상대고도 1m 이하에서 정지 속도 명령

45도는 명령 속도 기준이며, 실제 비행 궤적이나 목표물 중심 도달을 보장하지는 않습니다.

### 2.4 비행 상태 관리

비행 단계에 따라 상태를 전환하고, 목표물 검출 여부와 비행제어기 상태를 확인합니다.

주요 상태:

```text
IDLE
AUTO_SEARCH
TARGET_CONFIRMED
GUIDED_HOVER
GUIDED_APPROACH
GUIDED_DIAGONAL
GUIDED_STABILIZE
GUIDED_DESCEND
TARGET_LOST_HOVER
MISSION_ABORTED_HOVER
GUIDED_COMPLETE
MANUAL_LOCKOUT
```

### 2.5 안전 및 상태 모니터링

- 프로그램 시작 시 Manual Lockout 활성화
- 명시적인 RESET 및 FLY 명령을 통한 미션 활성화
- 수동 조종 개입 감지
- HEARTBEAT 및 텔레메트리 상태 확인
- 목표물 검출 유효시간 검사
- 수신 데이터에 따른 비행 명령 제한
- SITL / REAL 실행 프로그램 분리
- REAL 프로그램의 비행 명령 송신 차단

현재 안전 기능은 시뮬레이션 개발 단계의 구현이며, 실제 비행 안전성을 보장하지 않습니다.

## 3. 프로젝트 구조

```text
AstroDrone/
├── apps/
│   ├── sitl/                  # SITL 실행 프로그램
│   └── real/                  # 실제 Pixhawk 수신 프로그램
│
├── include/                   # C++ 클래스 헤더
├── src/                       # MAVLink 및 비행 제어 구현
│
├── transport/
│   ├── IMavlinkTransport.hpp
│   ├── udp/                   # SITL용 UDP 통신
│   └── serial/                # 실제 Pixhawk용 UART 통신
│
├── drone_vision/
│   ├── yolo_bridge.py         # Gazebo YOLO
│   ├── jetson_yolo.py         # Jetson Python 카메라 테스트
│   ├── README.md
│   └── tensorrt/              # Jetson C++ TensorRT YOLO
│
├── config/
├── docs/
├── CMakeLists.txt
└── README.md
```

`drone_vision/`은 GitHub 저장소에 복사한 버전이며, WSL의 기존 개발 폴더 `~/drone_vision`과는 별도로 존재합니다.

## 4. 주요 C++ 모듈

| 모듈 | 역할 |
|---|---|
| `MavlinkClient` | MAVLink 메시지 해석, 비행 모드·고도·속도 관리 |
| `VisionReceiver` | YOLO 검출 결과 UDP 수신 |
| `FlightPhaseManager` | 수직·사선 접근 및 하강 상태 머신 |
| `MissionManager` | 미션 활성화 및 진행 상태 관리 |
| `SafetyManager` | Manual Lockout 및 수동 개입 관리 |
| `HealthMonitor` | 배터리·GPS·RC·텔레메트리 상태 확인 |
| `UdpMavlinkTransport` | SITL UDP MAVLink 통신 |
| `SerialMavlinkTransport` | 실제 Pixhawk UART 수신 |

## 5. 개발 환경

### SITL

- Windows 11 / WSL2 Ubuntu 22.04
- Gazebo Harmonic
- ArduPilot SITL (ArduCopter)
- Mission Planner
- C++17 / CMake
- Python YOLO / ONNX Runtime
- MAVLink

### REAL (예정)

- NVIDIA Jetson Nano
- Pixhawk 4 Mini
- IMX219 CSI 카메라
- C++ TensorRT
- UART MAVLink
- ArduCopter

## 6. 빌드 및 실행

### 6.1 C++ 제어 프로그램 빌드

```bash
cmake -S . -B build
cmake --build build -j4
```

생성되는 실행 파일:

```text
build/control_sitl
build/control_real
```

### 6.2 SITL 실행

명령 송신 비활성화:

```bash
./build/control_sitl --sitl
```

SITL 비행 명령 활성화:

```bash
./build/control_sitl --sitl --enable-flight-tx
```

비행 명령 활성화 옵션은 Gazebo와 ArduPilot SITL이 연결된 것을 확인한 시뮬레이션 환경에서만 사용해야 합니다.

### 6.3 콘솔 명령어

```text
RESET
APPROACH VERTICAL
APPROACH DIAGONAL
FLY
MANUAL
STATUS
EXIT
```

### 6.4 REAL 텔레메트리 수신

```bash
./build/control_real <serial-device> <baud-rate>
```

예시:

```bash
./build/control_real /dev/ttyTHS1 57600
```

장치 경로와 통신 속도는 실제 Jetson 및 Pixhawk 설정에 따라 변경해야 합니다.

REAL 프로그램은 현재 비행 명령을 전송하지 않는 수신 전용 프로그램입니다.

### 6.5 Jetson TensorRT YOLO

Jetson용 TensorRT 프로그램은 `drone_vision/tensorrt/`에서 별도로 빌드합니다.

모델 입력 및 출력:

```text
Input  : [1, 3, 640, 640]
Output : [1, 25200, 6]
```

TensorRT 코드에는 ONNX 엔진 생성·로드, 이미지 전처리, 객체 검출 후처리, UDP 결과 전송 기능이 구현되어 있습니다.

실제 Jetson Nano에서의 빌드 및 추론은 아직 검증되지 않았습니다.

## 7. 개발 진행 상황

### 완료 및 부분 검증

- [x] Gazebo + ArduPilot SITL 연동
- [x] MAVLink 상태 수신
- [x] Python YOLO 기반 목표물 검출
- [x] YOLO 검출 결과 C++ 연동
- [x] AUTO → GUIDED 전환
- [x] 수직 접근 및 하강 알고리즘 구현
- [x] 45도 사선 접근 및 하강 SITL 동작 확인
- [x] SITL / REAL 실행 프로그램 분리
- [x] UDP 및 UART 통신 클래스 구현
- [x] 공통 MAVLink 파서 구성
- [x] Jetson 카메라 입력 코드 작성
- [x] TensorRT YOLO 추론 코드 작성

### 추가 검증 및 구현 예정

- [ ] 구조 변경 이후 수직·사선 하강 전체 회귀 테스트
- [ ] Jetson Nano에서 TensorRT 빌드 및 엔진 생성
- [ ] 실제 IMX219 카메라 영상 추론
- [ ] Pixhawk–Jetson UART 통신 실물 검증
- [ ] REAL 환경의 YOLO·MAVLink 수신 통합 테스트
- [ ] 실제 제어 명령 없이 Shadow Mode 검증
- [ ] 실제 비행을 위한 별도 안전 설계 및 검증

## 8. 개발 방향

현재 목표는 시뮬레이션에서 개발한 비행 제어 로직을 Jetson Nano와 Pixhawk 기반의 실제 드론 시스템으로 확장하는 것입니다.

시뮬레이션 환경에서 제어 알고리즘을 먼저 검증하고, 실제 환경에서는 카메라 인식과 비행제어기 텔레메트리 수신부터 단계적으로 연동할 계획입니다.

실제 기체의 비행 명령 송신은 현재 비활성화되어 있으며, 하드웨어 통신 및 안전 검증 이후 별도로 개발할 예정입니다.
