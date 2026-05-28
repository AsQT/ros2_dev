#!/usr/bin/env python3

import json
import math

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge


ARUCO_DICTS = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
}

def rvec_to_yaw_deg(rvec):
    R, _ = cv2.Rodrigues(rvec)
    yaw = math.atan2(R[1, 0], R[0, 0])
    return math.degrees(yaw)


class ArucoDetectNode(Node):
    def __init__(self):
        super().__init__("aruco_detect_node")

        self.declare_parameter("image_topic", "/astra/rgb/image_raw")
        self.declare_parameter("camera_info_topic", "/astra/rgb/camera_info")
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("marker_size", 0.04)

        # Mặc định tắt pose để tránh crash OpenCV trước
        self.declare_parameter("enable_pose", False)
        self.declare_parameter("draw_debug", True)

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.dictionary_name = self.get_parameter("dictionary").value
        self.marker_size = float(self.get_parameter("marker_size").value)
        self.enable_pose = bool(self.get_parameter("enable_pose").value)
        self.draw_debug = bool(self.get_parameter("draw_debug").value)

        if self.dictionary_name not in ARUCO_DICTS:
            self.get_logger().warn(
                f"Unknown dictionary {self.dictionary_name}, fallback DICT_4X4_50"
            )
            self.dictionary_name = "DICT_4X4_50"

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            ARUCO_DICTS[self.dictionary_name]
        )

        # Dùng API cũ cho ổn định với OpenCV 4.6 trên Ubuntu/ROS
        try:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters()

        self.bridge = CvBridge()

        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame = ""

        self.sub_info = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10,
        )

        self.sub_img = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
        )

        self.pub_debug = self.create_publisher(
            Image,
            "/aruco/image_annotated",
            10,
        )

        self.pub_json = self.create_publisher(
            String,
            "/aruco/detections_json",
            10,
        )

        self.get_logger().info("ArucoDetectNode SAFE started")
        self.get_logger().info(f"Image topic      : {self.image_topic}")
        self.get_logger().info(f"CameraInfo topic : {self.camera_info_topic}")
        self.get_logger().info(f"Dictionary       : {self.dictionary_name}")
        self.get_logger().info(f"Marker size      : {self.marker_size} m")
        self.get_logger().info(f"Enable pose      : {self.enable_pose}")

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)

        if len(msg.d) > 0:
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        else:
            self.dist_coeffs = np.zeros((5,), dtype=np.float64)

        self.camera_frame = msg.header.frame_id

    def estimate_pose_solvepnp(self, corners):
        """
        Tính pose marker bằng solvePnP.
        Hệ tọa độ marker đặt tại tâm marker.
        """
        s = self.marker_size / 2.0

        obj_points = np.array(
            [
                [-s,  s, 0.0],
                [ s,  s, 0.0],
                [ s, -s, 0.0],
                [-s, -s, 0.0],
            ],
            dtype=np.float32,
        )

        img_points = corners.reshape(4, 2).astype(np.float32)

        ok, rvec, tvec = cv2.solvePnP(
            obj_points,
            img_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if not ok:
            return None, None

        return rvec, tvec

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        if frame is None:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        annotated = frame.copy()

        try:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray,
                self.aruco_dict,
                parameters=self.aruco_params,
            )
        except Exception as e:
            self.get_logger().error(f"aruco detect error: {e}")
            return

        detections = []

        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten()

            if self.draw_debug:
                cv2.aruco.drawDetectedMarkers(annotated, corners, ids)

            for i, marker_id in enumerate(ids_flat):
                pts = corners[i].reshape(4, 2)

                cx = float(np.mean(pts[:, 0]))
                cy = float(np.mean(pts[:, 1]))

                det = {
                    "id": int(marker_id),
                    "center_px": {
                        "x": cx,
                        "y": cy,
                    },
                    "corners_px": pts.astype(float).tolist(),
                }

                if self.enable_pose:
                    if self.camera_matrix is not None and self.dist_coeffs is not None:
                        try:
                            rvec, tvec = self.estimate_pose_solvepnp(corners[i])

                            if rvec is not None and tvec is not None:
                                x = float(tvec[0][0])
                                y = float(tvec[1][0])
                                z = float(tvec[2][0])
                                yaw_deg = float(rvec_to_yaw_deg(rvec))

                                det["position_m"] = {
                                    "x": x,
                                    "y": y,
                                    "z": z,
                                }

                                det["yaw_deg"] = yaw_deg

                                cv2.drawFrameAxes(
                                    annotated,
                                    self.camera_matrix,
                                    self.dist_coeffs,
                                    rvec,
                                    tvec,
                                    self.marker_size * 0.5,
                                )
                        except Exception as e:
                            self.get_logger().warn(f"pose error marker {marker_id}: {e}")
                    else:
                        det["pose_status"] = "missing_camera_info"

                detections.append(det)

        out = {
            "stamp": {
                "sec": int(msg.header.stamp.sec),
                "nanosec": int(msg.header.stamp.nanosec),
            },
            "frame_id": msg.header.frame_id,
            "count": len(detections),
            "detections": detections,
        }

        json_msg = String()
        json_msg.data = json.dumps(out, ensure_ascii=False)
        self.pub_json.publish(json_msg)

        if self.draw_debug:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                debug_msg.header = msg.header
                self.pub_debug.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f"publish debug image error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()