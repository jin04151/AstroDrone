
#!/usr/bin/env python3

import argparse
import socket
import time
from pathlib import Path

import cv2

UDP_HOST = "127.0.0.1"
UDP_PORT = 14560
TARGET_CLASS = 0
MIN_CONF = 0.6


def camera_pipeline(sensor_id=0, flip=0):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} wbmode=3 ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)640, height=(int)480, "
        "format=(string)NV12, "
        "framerate=(fraction)30/1 ! "
        f"nvvidconv flip-method={flip} ! "
        "video/x-raw, "
        "width=(int)640, height=(int)480, "
        "format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )


def send_detection(sock, detected, class_id,
                   confidence, cx, cy, width, height):
    message = (
        f"{int(detected)},{class_id},"
        f"{confidence:.4f},{cx},{cy},"
        f"{width},{height}"
    )
    sock.sendto(
        message.encode("utf-8"),
        (UDP_HOST, UDP_PORT)
    )
    return message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model",
        default="models/best_v5.onnx"
    )
    parser.add_argument(
        "--sensor",
        type=int,
        default=0
    )
    parser.add_argument(
        "--flip",
        type=int,
        default=0
    )
    parser.add_argument(
        "--camera-only",
        action="store_true"
    )
    parser.add_argument(
        "--show",
        action="store_true"
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=MIN_CONF
    )
    args = parser.parse_args()

    pipeline = camera_pipeline(
        args.sensor,
        args.flip
    )

    print("[CAMERA] Opening Jetson CSI camera")

    cap = cv2.VideoCapture(
        pipeline,
        cv2.CAP_GSTREAMER
    )

    if not cap.isOpened():
        raise RuntimeError(
            "Cannot open CSI camera. "
            "Check camera, GStreamer and sensor ID."
        )

    model = None

    if not args.camera_only:
        model_path = Path(args.model)

        if not model_path.is_file():
            cap.release()
            raise FileNotFoundError(model_path)

        from ultralytics import YOLO

        print("[YOLO] Loading:", model_path)
        model = YOLO(str(model_path))

    sock = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM
    )

    frames = 0
    start = time.monotonic()

    print("[CAMERA] Running")
    print("[CONFIG] Flight command TX: NONE")

    try:
        while True:
            ok, frame = cap.read()

            if not ok or frame is None:
                print("[CAMERA] Frame read failed")
                break

            height, width = frame.shape[:2]
            frames += 1

            if model is None:
                if frames % 30 == 0:
                    print(
                        f"[CAMERA] Frame {frames} "
                        f"{width}x{height}"
                    )

                if args.show:
                    cv2.imshow(
                        "Jetson Camera",
                        frame
                    )
                    if cv2.waitKey(1) & 0xFF == 27:
                        break
                continue

            # Run inference on the actual camera frame.
            results = model.predict(
                source=frame,
                conf=args.conf,
                classes=[TARGET_CLASS],
                verbose=False,
                imgsz=640
            )

            best = None

            for result in results:
                for box in result.boxes:
                    class_id = int(box.cls[0])
                    confidence = float(box.conf[0])

                    if class_id != TARGET_CLASS:
                        continue

                    if best is None or confidence > best[0]:
                        xyxy = box.xyxy[0].tolist()
                        best = (confidence, xyxy)

            detected = best is not None

            if detected:
                confidence, (x1, y1, x2, y2) = best

                cx = max(
                    0,
                    min(width - 1, int((x1 + x2) / 2))
                )
                cy = max(
                    0,
                    min(height - 1, int((y1 + y2) / 2))
                )

                message = send_detection(
                    sock, True, TARGET_CLASS,
                    confidence, cx, cy,
                    width, height
                )

                if args.show:
                    cv2.rectangle(
                        frame,
                        (int(x1), int(y1)),
                        (int(x2), int(y2)),
                        (0, 0, 255),
                        2
                    )
            else:
                message = send_detection(
                    sock, False, -1, 0.0,
                    0, 0, width, height
                )

            if frames % 10 == 0:
                elapsed = max(
                    time.monotonic() - start,
                    0.001
                )
                print(
                    f"[YOLO] {message} "
                    f"| FPS(avg)={frames / elapsed:.2f}"
                )

            if args.show:
                cv2.imshow(
                    "Jetson YOLO",
                    frame
                )
                if cv2.waitKey(1) & 0xFF == 27:
                    break

    except KeyboardInterrupt:
        print("\n[STOP] Ctrl+C")

    finally:
        cap.release()
        sock.close()
        cv2.destroyAllWindows()

        print("[CAMERA] Closed")


if __name__ == "__main__":
    main()
