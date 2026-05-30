#!/usr/bin/env python3

import json
import math

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from robot_vision_pipeline_msgs.msg import ArucoPose, ArucoPoseArray # type: ignore msg custom
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from cv_bridge import CvBridge



def rvec_to_yaw_deg(rvec):
    R, _ = cv2.Rodrigues(rvec) # chuyển đổi vector xoay rvec thành ma trận xoay R
    yaw = math.atan2(R[1, 0], R[0, 0])
    return math.degrees(yaw)


class ArucoDetectNode(Node):
    def __init__(self):
        super().__init__("aruco_detect_node")

        self.declare_parameter("image_topic", "/astra/rgb/image_raw")
        self.declare_parameter("camera_info_topic", "/astra/rgb/camera_info")
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("marker_size", 0.03) # kích thước cạnh marker ArUco tính bằng mét
        self.declare_parameter("enable_pose", True)
        self.declare_parameter("draw_debug", True)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "astra_link_optical")

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.dictionary_name = self.get_parameter("dictionary").value
        self.marker_size = float(self.get_parameter("marker_size").value)
        self.enable_pose = bool(self.get_parameter("enable_pose").value) # nếu False sẽ không tính pose
        self.draw_debug = bool(self.get_parameter("draw_debug").value) # nếu True sẽ vẽ debug trên ảnh và publish ra topic /aruco/image_annotated
        self.base_frame = self.get_parameter("base_frame").value
        self.camera_frame_override = self.get_parameter("camera_frame").value


        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        '''
        cv2.aruco.DICT_4X4_50
        cv2.aruco.DICT_5X5_100
        cv2.aruco.DICT_6X6_250
        cv2.aruco.DICT_7X7_1000
        '''

        try:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            # tạo bộ tham số cho quán trình detect
            self.aruco_params = cv2.aruco.DetectorParameters()

        self.bridge = CvBridge()

        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame = ""

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub_info = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10,
        )

        self.sub_img = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback, # callback xử lý ảnh khi có ảnh mới từ topic
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

        self.pub_pose = self.create_publisher(
            ArucoPoseArray,
            "/aruco_pose",
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

        if self.camera_frame_override:
            self.camera_frame = self.normalize_camera_frame(self.camera_frame_override)
        else:
            self.camera_frame = self.normalize_camera_frame(msg.header.frame_id)

        self.get_logger().info(f"Resolved camera TF frame: {self.camera_frame}")

    def normalize_camera_frame(self, frame_id):
        if not frame_id:
            return frame_id

        if "astra_rgb" in frame_id or "astra_depth" in frame_id:
            return "astra_link_optical"

        if frame_id.endswith("_link") and not frame_id.endswith("_optical"):
            return f"{frame_id}_optical"

        return frame_id

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

        return rvec, tvec # rvec hướng marker so với camera, tvec vị trí marker so với camera

    @staticmethod
    def rotation_matrix_to_quaternion(R):
        q = np.empty((4,), dtype=np.float64)
        trace = np.trace(R)
        if trace > 0.0:
            s = 0.5 / np.sqrt(trace + 1.0)
            q[3] = 0.25 / s
            q[0] = (R[2, 1] - R[1, 2]) * s
            q[1] = (R[0, 2] - R[2, 0]) * s
            q[2] = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                q[3] = (R[2, 1] - R[1, 2]) / s
                q[0] = 0.25 * s
                q[1] = (R[0, 1] + R[1, 0]) / s
                q[2] = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                q[3] = (R[0, 2] - R[2, 0]) / s
                q[0] = (R[0, 1] + R[1, 0]) / s
                q[1] = 0.25 * s
                q[2] = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                q[3] = (R[1, 0] - R[0, 1]) / s
                q[0] = (R[0, 2] + R[2, 0]) / s
                q[1] = (R[1, 2] + R[2, 1]) / s
                q[2] = 0.25 * s
        return q

    def pose_stamped_from_rvec_tvec(self, rvec, tvec, frame_id, stamp=None):
        R, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = self.rotation_matrix_to_quaternion(R)

        pose = PoseStamped()
        pose.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(tvec[0][0])
        pose.pose.position.y = float(tvec[1][0])
        pose.pose.position.z = float(tvec[2][0])
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        return pose

    def get_camera_frame(self, msg):
        if self.camera_frame_override:
            return self.normalize_camera_frame(self.camera_frame_override)
        if self.camera_frame:
            return self.normalize_camera_frame(self.camera_frame)
        return self.normalize_camera_frame(msg.header.frame_id)

    def image_callback(self, msg):
        # step 1: chuyển ROS Image message thành OpenCV image
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        if frame is None:
            return
        # step 2: chuyển ảnh sang grayscale 
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        annotated = frame.copy()

        # step 3: phát hiện marker ArUco trong ảnh
        try:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                                            gray,
                                            self.aruco_dict,
                                            parameters=self.aruco_params,      )
        except Exception as e:
            self.get_logger().error(f"aruco detect error: {e}")
            return

        detections = []
        pose_array = ArucoPoseArray()
        pose_array.header = msg.header
        # step 4: nếu phát hiện marker, xử lý từng marker để tính toán pose 
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

                pose_msg = ArucoPose()
                pose_msg.header = msg.header
                pose_msg.id = int(marker_id)
                pose_msg.frame_cam = msg.header.frame_id or self.camera_frame or ""
                pose_msg.frame_base = self.base_frame
                pose_msg.has_pose_base = False
                pose_msg.yaw_deg = 0.0

                if self.enable_pose: # chỉ tính pose nếu enable_pose = True, tránh lỗi khi thiếu camera info
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

                                camera_frame = self.get_camera_frame(msg)
                                pose_msg.frame_cam = camera_frame
                                pose_cam = self.pose_stamped_from_rvec_tvec(
                                    rvec,
                                    tvec,
                                    camera_frame,
                                    stamp=msg.header.stamp if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0 else self.get_clock().now().to_msg(),
                                )
                                pose_msg.yaw_deg = yaw_deg

                                transform = None
                                exact_time = rclpy.time.Time.from_msg(msg.header.stamp) if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0 else rclpy.time.Time()
                                try:
                                    transform = self.tf_buffer.lookup_transform(
                                        self.base_frame,
                                        camera_frame,
                                        exact_time,
                                        timeout=rclpy.duration.Duration(seconds=0.5),
                                    )
                                except Exception as e:
                                    if msg.header.frame_id and msg.header.frame_id != camera_frame:
                                        try:
                                            transform = self.tf_buffer.lookup_transform(
                                                self.base_frame,
                                                msg.header.frame_id,
                                                exact_time,
                                                timeout=rclpy.duration.Duration(seconds=0.5),
                                            )
                                            pose_msg.frame_cam = msg.header.frame_id
                                            pose_cam.header.frame_id = msg.header.frame_id
                                        except Exception:
                                            self.get_logger().warn(
                                                f"TF transform to {self.base_frame} failed for exact stamp from both camera_frame={camera_frame} and header.frame_id={msg.header.frame_id}. Trying latest transform."
                                            )
                                    else:
                                        self.get_logger().warn(
                                            f"TF transform to {self.base_frame} failed for exact stamp from source frame {camera_frame}. Trying latest transform."
                                        )

                                    try:
                                        transform = self.tf_buffer.lookup_transform(
                                            self.base_frame,
                                            camera_frame,
                                            rclpy.time.Time(),
                                            timeout=rclpy.duration.Duration(seconds=0.5),
                                        )
                                        self.get_logger().warn(
                                            f"Using latest TF transform for {camera_frame} -> {self.base_frame} as exact timestamp lookup failed."
                                        )
                                    except Exception as e2:
                                        if msg.header.frame_id and msg.header.frame_id != camera_frame:
                                            try:
                                                transform = self.tf_buffer.lookup_transform(
                                                    self.base_frame,
                                                    msg.header.frame_id,
                                                    rclpy.time.Time(),
                                                    timeout=rclpy.duration.Duration(seconds=0.5),
                                                )
                                                pose_msg.frame_cam = msg.header.frame_id
                                                pose_cam.header.frame_id = msg.header.frame_id
                                                self.get_logger().warn(
                                                    f"Using latest TF transform for {msg.header.frame_id} -> {self.base_frame} as exact timestamp lookup failed."
                                                )
                                            except Exception as e3:
                                                self.get_logger().warn(
                                                    f"TF transform to {self.base_frame} failed for both latest and exact frames: {e3}"
                                                )
                                        else:
                                            self.get_logger().warn(
                                                f"TF transform to {self.base_frame} failed for latest and exact source frame {camera_frame}: {e2}"
                                            )

                                if transform is not None:
                                    pose_base = do_transform_pose(pose_cam.pose, transform)
                                    pose_msg.has_pose_base = True
                                    pose_msg.pose_base = pose_base

                                cv2.drawFrameAxes(
                                    annotated,
                                    self.camera_matrix,
                                    self.dist_coeffs,
                                    rvec,
                                    tvec,
                                    self.marker_size * 0.5,
                                )
                                '''

                                label = f"pix=({int(cx)},{int(cy)})"
                                text_pos = (int(cx) + 10, int(cy) - 10)
                                cv2.putText(
                                    annotated,
                                    label,
                                    text_pos,
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5,
                                    (255, 255, 0),
                                    2,
                                    cv2.LINE_AA,
                                )
                                '''
                        except Exception as e:
                            self.get_logger().warn(f"pose error marker {marker_id}: {e}")
                    else:
                        det["pose_status"] = "missing_camera_info"

                detections.append(det)
                pose_array.poses.append(pose_msg)

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
        self.pub_pose.publish(pose_array)

        if self.draw_debug:
            try:
                # Draw image origin axes at the top-left corner for reference.
                origin_size = 50
                cv2.arrowedLine(annotated, (0, 0), (origin_size, 0), (0, 0, 255), 2, tipLength=0.1)
                cv2.arrowedLine(annotated, (0, 0), (0, origin_size), (255, 0, 0), 2, tipLength=0.1)
                cv2.putText(
                    annotated,
                    "(0,0)",
                    (5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    annotated,
                    "x",
                    (origin_size + 5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    annotated,
                    "y",
                    (5, origin_size + 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 0, 0),
                    1,
                    cv2.LINE_AA,
                )

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