#!/home/minhquang/venvs/ros_env/bin/python3

import json
import time
from typing import Any, Dict, List
import os

import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class YoloDetectNode(Node):
    def __init__(self):
        super().__init__("yolo_detect_node")

        # =========================
        # Declare parameters
        # =========================
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("annotated_image_topic", "/vision/yolo/image_annotated")
        self.declare_parameter("detections_topic", "/vision/yolo/detections_json")

        self.declare_parameter(
            "model_path",
            os.path.expanduser("~/ros2/src/robot_vision_pipeline/model/best.pt"),
        )
        self.declare_parameter("device", "cpu")

        self.declare_parameter("conf_threshold", 0.35)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("imgsz", 640)

        # Detect chu kỳ, mặc định 1 giây/lần
        self.declare_parameter("detect_period_sec", 1.0)

        self.declare_parameter("show_gui", True)
        self.declare_parameter("class_filter", "")
        self.declare_parameter("verbose_log", False)

        # Override from launch / command line
        self.declare_parameter("model_path_override", "")
        self.declare_parameter("image_topic_override", "")

        # =========================
        # Get parameters
        # =========================
        image_topic = self.get_parameter("image_topic").value
        annotated_image_topic = self.get_parameter("annotated_image_topic").value
        detections_topic = self.get_parameter("detections_topic").value

        model_path = self.get_parameter("model_path").value
        model_path_override = self.get_parameter("model_path_override").value
        image_topic_override = self.get_parameter("image_topic_override").value

        if isinstance(model_path_override, str) and model_path_override.strip():
            model_path = model_path_override.strip()

        if isinstance(image_topic_override, str) and image_topic_override.strip():
            image_topic = image_topic_override.strip()

        self.device = str(self.get_parameter("device").value)
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.detect_period_sec = float(self.get_parameter("detect_period_sec").value)

        self.show_gui = bool(self.get_parameter("show_gui").value)
        self.verbose_log = bool(self.get_parameter("verbose_log").value)

        class_filter_param = str(self.get_parameter("class_filter").value).strip()
        self.class_filter = set()
        if class_filter_param:
            self.class_filter = set(
                x.strip() for x in class_filter_param.split(",") if x.strip()
            )

        self.bridge = CvBridge()
        self.model = None
        self.names = {}

        # =========================
        # Runtime state
        # =========================
        self.window_name = "YOLO Detect - 1Hz"
        self.latest_frame = None
        self.latest_header = None
        self.last_result_image = None
        self.last_payload = None

        self.last_detect_time = 0.0
        self.is_detecting = False

        self.frame_count = 0
        self.last_fps_time = time.time()
        self.preview_fps = 0.0

        self.detect_count = 0
        self.detect_fps = 0.0
        self.detect_fps_count = 0
        self.last_detect_fps_time = time.time()

        if self.show_gui:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 960, 720)

        # =========================
        # Load YOLO only once
        # =========================
        try:
            from ultralytics import YOLO  # pyright: ignore[reportMissingImports]

            self.get_logger().info(f"Loading YOLO model: {model_path}")
            self.model = YOLO(model_path)

            if hasattr(self.model, "names"):
                self.names = self.model.names
            else:
                self.names = {}

            self.get_logger().info("YOLO model loaded successfully.")
            self.get_logger().info(f"Class names: {self.names}")

        except Exception as e:
            self.get_logger().error("Failed to load YOLO model.")
            self.get_logger().error(str(e))
            self.get_logger().error(
                "Kiểm tra ultralytics:\n"
                "  python3 -c \"from ultralytics import YOLO; print('YOLO OK')\""
            )
            raise e

        # =========================
        # ROS publishers / subscribers
        # =========================
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            sensor_qos,
        )

        self.annotated_pub = self.create_publisher(
            Image,
            annotated_image_topic,
            10,
        )

        self.detections_pub = self.create_publisher(
            String,
            detections_topic,
            10,
        )

        self.get_logger().info("========================================")
        self.get_logger().info("YOLO Detect Node started")
        self.get_logger().info("Mode                  : PERIODIC 1Hz")
        self.get_logger().info("YOLO does NOT run realtime on every frame.")
        self.get_logger().info(f"Detect period         : {self.detect_period_sec:.3f} s")
        self.get_logger().info(f"Input image topic     : {image_topic}")
        self.get_logger().info(f"Annotated topic       : {annotated_image_topic}")
        self.get_logger().info(f"Detections JSON topic : {detections_topic}")
        self.get_logger().info(f"Device                : {self.device}")
        self.get_logger().info(f"Confidence threshold  : {self.conf_threshold}")
        self.get_logger().info(f"IoU threshold         : {self.iou_threshold}")
        self.get_logger().info(f"Image size            : {self.imgsz}")
        self.get_logger().info(f"Show GUI              : {self.show_gui}")
        self.get_logger().info(f"Class filter          : {list(self.class_filter)}")
        self.get_logger().info("========================================")

    # ============================================================
    # Camera callback:
    # - camera frame vẫn nhận liên tục
    # - YOLO chỉ chạy theo chu kỳ detect_period_sec, mặc định 1 giây/lần
    # ============================================================
    def image_callback(self, msg: Image):
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge convert error: {e}")
            return

        if frame_bgr is None:
            self.get_logger().warn("Received empty image.")
            return

        self.latest_frame = frame_bgr.copy()
        self.latest_header = msg.header
        self.update_preview_fps()

        now = time.time()

        # Chỉ detect khi đã đủ thời gian 1 giây từ lần detect trước.
        # Đặt last_detect_time trước khi inference để tránh detect dồn nếu inference bị chậm.
        if (not self.is_detecting) and (now - self.last_detect_time >= self.detect_period_sec):
            self.last_detect_time = now
            self.run_detect_once(frame_bgr.copy(), msg.header)

        if self.show_gui:
            # GUI hiển thị kết quả detect gần nhất. Nếu chưa có kết quả thì hiển thị frame live.
            if self.last_result_image is not None:
                display = self.last_result_image.copy()
            else:
                display = frame_bgr.copy()

            self.draw_gui_overlay(display)
            cv2.imshow(self.window_name, display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                cv2.destroyWindow(self.window_name)
                self.show_gui = False
                self.get_logger().info("GUI closed. Press Ctrl+C in terminal to stop node.")

    # ============================================================
    # Run YOLO once
    # ============================================================
    def run_detect_once(self, frame_bgr, header):
        self.is_detecting = True
        start = time.time()

        try:
            result = self.model.predict(
                source=frame_bgr,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                imgsz=self.imgsz,
                device=self.device,
                verbose=False,
            )[0]

        except Exception as e:
            self.get_logger().error(f"YOLO inference error: {e}")
            self.is_detecting = False
            return

        inference_ms = (time.time() - start) * 1000.0
        self.update_detect_fps()

        detections = self.parse_yolo_result(result)

        # Ảnh đã vẽ bbox từ YOLO
        annotated = result.plot()

        # Vẽ thông tin detection
        cv2.putText(
            annotated,
            (
                f"YOLO 1Hz | Objects: {len(detections)} | "
                f"infer: {inference_ms:.1f} ms | count: {self.detect_count + 1}"
            ),
            (20, 95),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        # Publish annotated image theo chu kỳ 1 giây/lần
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            annotated_msg.header = header
            self.annotated_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f"Publish annotated image error: {e}")

        # Publish JSON theo chu kỳ 1 giây/lần
        payload = {
            "stamp": {
                "sec": int(header.stamp.sec),
                "nanosec": int(header.stamp.nanosec),
            },
            "frame_id": header.frame_id,
            "image_width": int(frame_bgr.shape[1]),
            "image_height": int(frame_bgr.shape[0]),
            "mode": "periodic_1hz",
            "detect_period_sec": float(self.detect_period_sec),
            "preview_fps": float(self.preview_fps),
            "detect_fps": float(self.detect_fps),
            "inference_time_ms": float(inference_ms),
            "num_detections": len(detections),
            "detections": detections,
        }

        json_msg = String()
        json_msg.data = json.dumps(payload, ensure_ascii=False)
        self.detections_pub.publish(json_msg)

        if self.verbose_log:
            self.get_logger().info(json_msg.data)

        self.detect_count += 1
        self.last_result_image = annotated.copy()
        self.last_payload = payload
        self.is_detecting = False

        self.get_logger().info(
            f"Detect #{self.detect_count}: {len(detections)} object(s), inference={inference_ms:.1f} ms"
        )

    # ============================================================
    # Parse YOLO result
    # ============================================================
    def parse_yolo_result(self, result: Any) -> List[Dict[str, Any]]:
        detections: List[Dict[str, Any]] = []

        if result.boxes is None:
            return detections

        boxes = result.boxes

        for i in range(len(boxes)):
            try:
                xyxy = boxes.xyxy[i].detach().cpu().numpy().astype(float)
                conf = float(boxes.conf[i].detach().cpu().numpy())
                cls_id = int(boxes.cls[i].detach().cpu().numpy())

                class_name = self.get_class_name(cls_id)

                # Nếu có class_filter thì chỉ lấy class nằm trong filter
                if len(self.class_filter) > 0:
                    if class_name not in self.class_filter:
                        continue

                x1, y1, x2, y2 = xyxy.tolist()

                cx = 0.5 * (x1 + x2)
                cy = 0.5 * (y1 + y2)
                w = x2 - x1
                h = y2 - y1

                det = {
                    "id": len(detections),
                    "class_id": cls_id,
                    "class_name": class_name,
                    "confidence": conf,
                    "bbox_xyxy": {
                        "x1": x1,
                        "y1": y1,
                        "x2": x2,
                        "y2": y2,
                    },
                    "bbox_xywh": {
                        "cx": cx,
                        "cy": cy,
                        "w": w,
                        "h": h,
                    },
                }

                detections.append(det)

            except Exception as e:
                self.get_logger().warn(f"Parse detection error: {e}")
                continue

        return detections

    def get_class_name(self, cls_id: int) -> str:
        if isinstance(self.names, dict):
            return str(self.names.get(cls_id, cls_id))

        if isinstance(self.names, list):
            if 0 <= cls_id < len(self.names):
                return str(self.names[cls_id])

        return str(cls_id)

    # ============================================================
    # GUI overlay
    # ============================================================
    def draw_gui_overlay(self, img):
        # Nền mờ phía trên để đọc chữ dễ hơn
        cv2.rectangle(img, (0, 0), (img.shape[1], 80), (30, 30, 30), -1)

        cv2.putText(
            img,
            (
                f"YOLO PERIODIC DETECT | every {self.detect_period_sec:.1f}s | "
                f"Preview FPS: {self.preview_fps:.1f} | press q to close GUI"
            ),
            (20, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    def update_preview_fps(self):
        self.frame_count += 1
        now = time.time()
        dt = now - self.last_fps_time

        if dt >= 1.0:
            self.preview_fps = self.frame_count / dt
            self.frame_count = 0
            self.last_fps_time = now

    def update_detect_fps(self):
        self.detect_fps_count += 1
        now = time.time()
        dt = now - self.last_detect_fps_time

        if dt >= 1.0:
            self.detect_fps = self.detect_fps_count / dt
            self.detect_fps_count = 0
            self.last_detect_fps_time = now

    def destroy_node(self):
        if self.show_gui:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = YoloDetectNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(f"[YoloDetectNode] Fatal error: {e}")

    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()