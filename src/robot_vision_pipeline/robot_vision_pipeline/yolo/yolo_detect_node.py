#!/usr/bin/env python3

import gc
import json
import os
import threading
import time
from typing import Any, Dict, List

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

        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("annotated_image_topic", "/vision/yolo/image_annotated")
        self.declare_parameter("detections_topic", "/vision/yolo/detections_json")

        self.declare_parameter(
            "model_path",
            os.path.expanduser("~/ros2/src/robot_vision_pipeline/model/best.pt"),
        )
        self.declare_parameter("model_path_override", "")
        self.declare_parameter("image_topic_override", "")

        self.declare_parameter("device", "cpu")
        self.declare_parameter("conf_threshold", 0.35)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("imgsz", 256)
        self.declare_parameter("max_det", 20)

        self.declare_parameter("detect_period_sec", 0.2)
        self.declare_parameter("torch_num_threads", 2)

        self.declare_parameter("show_gui", False)
        self.declare_parameter("publish_annotated", True)
        self.declare_parameter("class_filter", "")
        self.declare_parameter("verbose_log", False)

        image_topic = str(self.get_parameter("image_topic").value)
        annotated_image_topic = str(self.get_parameter("annotated_image_topic").value)
        detections_topic = str(self.get_parameter("detections_topic").value)

        model_path = str(self.get_parameter("model_path").value)
        model_path_override = str(self.get_parameter("model_path_override").value)
        image_topic_override = str(self.get_parameter("image_topic_override").value)

        if model_path_override.strip():
            model_path = model_path_override.strip()

        if image_topic_override.strip():
            image_topic = image_topic_override.strip()

        self.device = str(self.get_parameter("device").value)
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.max_det = int(self.get_parameter("max_det").value)

        self.detect_period_sec = float(self.get_parameter("detect_period_sec").value)
        self.torch_num_threads = int(self.get_parameter("torch_num_threads").value)

        self.show_gui = bool(self.get_parameter("show_gui").value)
        self.publish_annotated = bool(self.get_parameter("publish_annotated").value)
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

        self.frame_lock = threading.Lock()
        self.latest_msg = None

        self.rx_frames = 0
        self.detect_count = 0
        self.publish_count = 0

        self.last_processed_stamp_ns = -1
        self.is_detecting = False
        self.last_detect_duration_ms = 0.0

        try:
            try:
                import torch  # type: ignore

                if self.torch_num_threads > 0:
                    torch.set_num_threads(self.torch_num_threads)
                    torch.set_num_interop_threads(max(1, min(2, self.torch_num_threads)))
                    self.get_logger().info(
                        f"Torch threads set to {torch.get_num_threads()}"
                    )
            except Exception as e:
                self.get_logger().warn(f"Could not set torch threads: {e}")

            from ultralytics import YOLO # type: ignore

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
            raise e

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
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
            sensor_qos,
        )

        self.detections_pub = self.create_publisher(
            String,
            detections_topic,
            10,
        )

        self.detect_timer = self.create_timer(
            self.detect_period_sec,
            self.detect_timer_callback,
        )

        self.status_timer = self.create_timer(
            5.0,
            self.status_timer_callback,
        )

        self.get_logger().info("========================================")
        self.get_logger().info("YOLO Detect Node started - TIMER SAFE MODE")
        self.get_logger().info(f"Input image topic      : {image_topic}")
        self.get_logger().info(f"Annotated image topic  : {annotated_image_topic}")
        self.get_logger().info(f"Detections JSON topic  : {detections_topic}")
        self.get_logger().info(f"Device                 : {self.device}")
        self.get_logger().info(f"Confidence threshold   : {self.conf_threshold}")
        self.get_logger().info(f"IoU threshold          : {self.iou_threshold}")
        self.get_logger().info(f"Image size             : {self.imgsz}")
        self.get_logger().info(f"Max detections         : {self.max_det}")
        self.get_logger().info(f"Detect period          : {self.detect_period_sec} s")
        self.get_logger().info(f"Publish annotated      : {self.publish_annotated}")
        self.get_logger().info(f"Show GUI               : {self.show_gui}")
        self.get_logger().info(f"Class filter           : {list(self.class_filter)}")
        self.get_logger().info("========================================")

    def image_callback(self, msg: Image):
        # Không chạy YOLO ở đây.
        # Callback chỉ giữ lại frame mới nhất để tránh nghẽn callback camera.
        with self.frame_lock:
            self.latest_msg = msg
            self.rx_frames += 1

    def detect_timer_callback(self):
        if self.is_detecting:
            self.get_logger().warn("Previous YOLO inference still running, skip this tick.")
            return

        with self.frame_lock:
            msg = self.latest_msg

        if msg is None:
            self.get_logger().warn("No image received yet.")
            return

        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

        if stamp_ns == self.last_processed_stamp_ns:
            self.get_logger().warn(
                "No new image frame since last detection. Check Gazebo camera/bridge rate."
            )
            return

        self.is_detecting = True
        t0 = time.time()

        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            if frame_bgr is None:
                self.get_logger().warn("Received empty image.")
                return

            result = self.model.predict(
                source=frame_bgr,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                imgsz=self.imgsz,
                device=self.device,
                max_det=self.max_det,
                verbose=False,
                stream=False,
            )[0]

            detections = self.parse_yolo_result(result)

            if self.publish_annotated:
                annotated = self.draw_detections(frame_bgr, detections)
                self.draw_status(annotated, detections)

                annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                annotated_msg.header = msg.header
                self.annotated_pub.publish(annotated_msg)

                if self.show_gui:
                    cv2.imshow("YOLO Detect - timer safe", annotated)
                    cv2.waitKey(1)

            payload = {
                "stamp": {
                    "sec": int(msg.header.stamp.sec),
                    "nanosec": int(msg.header.stamp.nanosec),
                },
                "frame_id": msg.header.frame_id,
                "image_width": int(frame_bgr.shape[1]),
                "image_height": int(frame_bgr.shape[0]),
                "detect_period_sec": float(self.detect_period_sec),
                "detect_duration_ms": float((time.time() - t0) * 1000.0),
                "rx_frames": int(self.rx_frames),
                "detect_count": int(self.detect_count + 1),
                "num_detections": len(detections),
                "detections": detections,
            }

            json_msg = String()
            json_msg.data = json.dumps(payload, ensure_ascii=False)
            self.detections_pub.publish(json_msg)

            self.detect_count += 1
            self.publish_count += 1
            self.last_processed_stamp_ns = stamp_ns

            if self.verbose_log:
                self.get_logger().info(json_msg.data)

            del result
            del frame_bgr

            if self.detect_count % 20 == 0:
                gc.collect()

        except Exception as e:
            self.get_logger().error(f"Detect timer error: {repr(e)}")

        finally:
            self.last_detect_duration_ms = (time.time() - t0) * 1000.0
            self.is_detecting = False

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

                if len(self.class_filter) > 0 and class_name not in self.class_filter:
                    continue

                x1, y1, x2, y2 = xyxy.tolist()
                cx = 0.5 * (x1 + x2)
                cy = 0.5 * (y1 + y2)
                w = x2 - x1
                h = y2 - y1

                detections.append(
                    {
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
                )

            except Exception as e:
                self.get_logger().warn(f"Parse detection error: {e}")
                continue

        return detections

    def draw_detections(self, frame_bgr, detections: List[Dict[str, Any]]):
        annotated = frame_bgr.copy()

        for det in detections:
            bbox = det["bbox_xyxy"]
            x1 = int(bbox["x1"])
            y1 = int(bbox["y1"])
            x2 = int(bbox["x2"])
            y2 = int(bbox["y2"])

            class_name = str(det["class_name"])
            conf = float(det["confidence"])

            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)

            label = f"{class_name} {conf:.2f}"
            y_text = max(20, y1 - 8)
            cv2.putText(
                annotated,
                label,
                (x1, y_text),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        return annotated

    def draw_status(self, annotated, detections: List[Dict[str, Any]]):
        text = (
            f"period:{self.detect_period_sec:.1f}s | "
            f"det:{len(detections)} | "
            f"rx:{self.rx_frames} | "
            f"pub:{self.publish_count} | "
            f"{self.last_detect_duration_ms:.0f}ms"
        )

        cv2.putText(
            annotated,
            text,
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

    def get_class_name(self, cls_id: int) -> str:
        if isinstance(self.names, dict):
            return str(self.names.get(cls_id, cls_id))

        if isinstance(self.names, list):
            if 0 <= cls_id < len(self.names):
                return str(self.names[cls_id])

        return str(cls_id)

    def status_timer_callback(self):
        self.get_logger().info(
            f"Status | rx_frames={self.rx_frames}, "
            f"detect_count={self.detect_count}, "
            f"publish_count={self.publish_count}, "
            f"last_detect={self.last_detect_duration_ms:.1f} ms, "
            f"is_detecting={self.is_detecting}"
        )

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
