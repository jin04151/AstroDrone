
import queue
import socket

import cv2
import numpy as np
import onnxruntime as ort

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image


# ========================================
# Configuration
# ========================================

MODEL_PATH = "/home/hj/drone_vision/models/best_v5.onnx"

CAMERA_TOPIC = (
    "/world/iris_runway/model/iris_with_gimbal/"
    "model/gimbal/link/pitch_link/sensor/camera/image"
)

OUTPUT_TOPIC = "/yolo/annotated_image"

UDP_IP = "127.0.0.1"
UDP_PORT = 14560

INPUT_SIZE = 640

# Python detection threshold
CONF_THRESHOLD = 0.35
IOU_THRESHOLD = 0.45

TARGET_CLASS_ID = 0
MAX_DETECTIONS = 30

DEBUG_INTERVAL = 30


# ========================================
# ONNX Runtime
# ========================================

session = ort.InferenceSession(
    MODEL_PATH,
    providers=["CPUExecutionProvider"]
)

input_info = session.get_inputs()[0]
output_info = session.get_outputs()[0]

input_name = input_info.name
output_name = output_info.name

print("[ONNX] Input :", input_info.shape)
print("[ONNX] Output:", output_info.shape)
print("[ONNX] Provider:", session.get_providers())
print("[ONNX] Class: 0 = white_box")

# This postprocessor is designed for
# YOLOv5 raw output [1, 25200, 6].
if (len(output_info.shape) != 3 or
        output_info.shape[2] != 6):
    raise RuntimeError(
        "Unexpected ONNX output format. "
        "Expected [1, 25200, 6]."
    )


# ========================================
# Gazebo / UDP
# ========================================

node = Node()
publisher = node.advertise(OUTPUT_TOPIC, Image)

udp_socket = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM
)

frames = queue.Queue(maxsize=1)

print(f"[UDP] Sending to {UDP_IP}:{UDP_PORT}")


def camera_callback(msg):
    # Keep only the latest camera frame.
    if frames.full():
        try:
            frames.get_nowait()
        except queue.Empty:
            pass

    try:
        frames.put_nowait(msg)
    except queue.Full:
        pass


if not node.subscribe(
    Image,
    CAMERA_TOPIC,
    camera_callback
):
    raise RuntimeError("Gazebo camera subscription failed")


# ========================================
# Preprocessing
# ========================================

def preprocess(frame):
    """
    BGR image -> letterbox 640x640
    -> RGB -> CHW -> float32 [0,1]
    """

    h, w = frame.shape[:2]

    ratio = min(
        INPUT_SIZE / w,
        INPUT_SIZE / h
    )

    new_w = int(round(w * ratio))
    new_h = int(round(h * ratio))

    resized = cv2.resize(
        frame,
        (new_w, new_h),
        interpolation=cv2.INTER_LINEAR
    )

    dw = INPUT_SIZE - new_w
    dh = INPUT_SIZE - new_h

    left = int(round(dw / 2 - 0.1))
    right = int(round(dw / 2 + 0.1))
    top = int(round(dh / 2 - 0.1))
    bottom = int(round(dh / 2 + 0.1))

    padded = cv2.copyMakeBorder(
        resized,
        top,
        bottom,
        left,
        right,
        cv2.BORDER_CONSTANT,
        value=(114, 114, 114)
    )

    rgb = cv2.cvtColor(
        padded,
        cv2.COLOR_BGR2RGB
    )

    blob = rgb.transpose(2, 0, 1)
    blob = np.ascontiguousarray(
        blob,
        dtype=np.float32
    )

    blob /= 255.0
    blob = blob[np.newaxis, ...]

    return blob, ratio, left, top


# ========================================
# YOLOv5 Postprocessing
# ========================================

def postprocess(
    prediction,
    original_w,
    original_h,
    ratio,
    pad_left,
    pad_top
):
    """
    YOLOv5 raw predictions:
    [cx, cy, width, height, obj_conf, class_conf]

    Returns:
    [(x1, y1, x2, y2, confidence), ...]
    """

    predictions = prediction[0]

    if predictions.ndim != 2 or predictions.shape[1] != 6:
        raise ValueError("Invalid YOLOv5 output shape")

    # Combined confidence
    scores = (
        predictions[:, 4] *
        predictions[:, 5]
    )

    valid = (
        np.isfinite(predictions).all(axis=1) &
        (scores >= CONF_THRESHOLD) &
        (predictions[:, 2] > 0) &
        (predictions[:, 3] > 0)
    )

    predictions = predictions[valid]
    scores = scores[valid]

    if len(predictions) == 0:
        return []

    # YOLOv5 xywh(center) -> xyxy
    cx = predictions[:, 0]
    cy = predictions[:, 1]
    bw = predictions[:, 2]
    bh = predictions[:, 3]

    x1 = cx - bw / 2
    y1 = cy - bh / 2
    x2 = cx + bw / 2
    y2 = cy + bh / 2

    # Remove letterbox padding
    # and map back to original image size.
    x1 = (x1 - pad_left) / ratio
    x2 = (x2 - pad_left) / ratio

    y1 = (y1 - pad_top) / ratio
    y2 = (y2 - pad_top) / ratio

    x1 = np.clip(x1, 0, original_w - 1)
    x2 = np.clip(x2, 0, original_w - 1)

    y1 = np.clip(y1, 0, original_h - 1)
    y2 = np.clip(y2, 0, original_h - 1)

    # Reject invalid or tiny boxes.
    box_valid = (
        (x2 - x1 >= 2) &
        (y2 - y1 >= 2)
    )

    x1 = x1[box_valid]
    y1 = y1[box_valid]
    x2 = x2[box_valid]
    y2 = y2[box_valid]
    scores = scores[box_valid]

    if len(scores) == 0:
        return []

    # OpenCV NMS expects xywh where
    # x,y are top-left coordinates.
    nms_boxes = []
    nms_scores = []

    for i in range(len(scores)):
        nms_boxes.append([
            float(x1[i]),
            float(y1[i]),
            float(x2[i] - x1[i]),
            float(y2[i] - y1[i])
        ])

        nms_scores.append(float(scores[i]))

    indices = cv2.dnn.NMSBoxes(
        nms_boxes,
        nms_scores,
        CONF_THRESHOLD,
        IOU_THRESHOLD
    )

    if len(indices) == 0:
        return []

    indices = np.array(indices).reshape(-1)

    indices = sorted(
        indices,
        key=lambda i: nms_scores[int(i)],
        reverse=True
    )[:MAX_DETECTIONS]

    detections = []

    for idx in indices:
        idx = int(idx)

        detections.append((
            int(round(x1[idx])),
            int(round(y1[idx])),
            int(round(x2[idx])),
            int(round(y2[idx])),
            nms_scores[idx]
        ))

    return detections


# ========================================
# UDP
# ========================================

def send_detection(
    detected,
    confidence,
    cx,
    cy,
    width,
    height
):
    class_id = TARGET_CLASS_ID if detected else -1

    message = (
        f"{int(detected)},"
        f"{class_id},"
        f"{confidence:.4f},"
        f"{cx},"
        f"{cy},"
        f"{width},"
        f"{height}"
    )

    udp_socket.sendto(
        message.encode("utf-8"),
        (UDP_IP, UDP_PORT)
    )


# ========================================
# Main Loop
# ========================================

print("[YOLO] Processing started")

count = 0

try:
    while True:

        try:
            msg = frames.get(timeout=1)
        except queue.Empty:
            continue

        w = msg.width
        h = msg.height

        if w <= 0 or h <= 0:
            continue

        if msg.pixel_format_type not in (3, 8):
            print(
                "[CAMERA] Unsupported pixel format:",
                msg.pixel_format_type
            )
            continue

        step = msg.step if msg.step else w * 3

        if step < w * 3:
            continue

        raw = np.frombuffer(
            msg.data,
            dtype=np.uint8
        )

        if len(raw) < h * step:
            continue

        frame = raw[:h * step].reshape(
            h, step
        )[:, :w * 3].reshape(
            h, w, 3
        ).copy()

        # Gazebo RGB -> OpenCV BGR
        if msg.pixel_format_type == 3:
            frame = cv2.cvtColor(
                frame,
                cv2.COLOR_RGB2BGR
            )

        # --------------------------------
        # Preprocess
        # --------------------------------

        blob, ratio, pad_left, pad_top = (
            preprocess(frame)
        )

        # --------------------------------
        # ONNX inference
        # --------------------------------

        prediction = session.run(
            [output_name],
            {input_name: blob}
        )[0]

        # --------------------------------
        # YOLOv5 postprocess + NMS
        # --------------------------------

        detections = postprocess(
            prediction,
            w,
            h,
            ratio,
            pad_left,
            pad_top
        )

        # --------------------------------
        # Select best detection
        # --------------------------------

        target_detected = len(detections) > 0
        target_conf = 0.0
        target_cx = 0
        target_cy = 0

        if target_detected:
            x1, y1, x2, y2, conf = detections[0]

            target_conf = conf
            target_cx = (x1 + x2) // 2
            target_cy = (y1 + y2) // 2

        # Send one result per frame.
        send_detection(
            target_detected,
            target_conf,
            target_cx,
            target_cy,
            w,
            h
        )

        # --------------------------------
        # Draw validated-coordinate boxes
        # --------------------------------

        annotated = frame.copy()

        for x1, y1, x2, y2, conf in detections:

            cv2.rectangle(
                annotated,
                (x1, y1),
                (x2, y2),
                (0, 0, 255),
                2
            )

            label = f"white_box {conf:.2f}"

            cv2.putText(
                annotated,
                label,
                (x1, max(15, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                1,
                cv2.LINE_AA
            )

        # --------------------------------
        # Gazebo Image Display
        # --------------------------------

        annotated_rgb = cv2.cvtColor(
            annotated,
            cv2.COLOR_BGR2RGB
        )

        output = Image()
        output.CopyFrom(msg)

        output.pixel_format_type = 3
        output.step = w * 3
        output.data = annotated_rgb.tobytes()

        publisher.publish(output)

        # --------------------------------
        # Debug
        # --------------------------------

        count += 1

        if count % DEBUG_INTERVAL == 0:

            print(
                f"\n[DEBUG] Frame {count} "
                f"| Image {w}x{h} "
                f"| Detections {len(detections)}"
            )

            for x1, y1, x2, y2, conf in detections:

                area = (
                    (x2 - x1) * (y2 - y1)
                    / (w * h) * 100.0
                )

                print(
                    f"[BOX] Conf={conf:.3f} "
                    f"XYXY=({x1},{y1},{x2},{y2}) "
                    f"Size={x2-x1}x{y2-y1} "
                    f"Area={area:.1f}%"
                )

            if not detections:
                print("[BOX] No detections")

except KeyboardInterrupt:
    print("\n[YOLO] Stopped")

finally:
    udp_socket.close()
