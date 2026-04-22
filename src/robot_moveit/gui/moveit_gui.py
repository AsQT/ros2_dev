#!/usr/bin/env python3
import math
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor

import tf2_ros
from tf2_ros import TransformException

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    RobotState,
    JointConstraint,
)
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import QueryPlannerInterfaces

from PyQt5 import QtCore, QtWidgets, QtGui

# Optional camera deps
HAS_CAMERA_DEPS = True
try:
    import numpy as np
    import cv2
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image, CompressedImage
except Exception:
    HAS_CAMERA_DEPS = False


# ---------------------- Math helpers ----------------------
def rpy_to_quat(roll: float, pitch: float, yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


def quat_to_rpy(qx: float, qy: float, qz: float, qw: float):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


# ---------------------- Data types ----------------------
@dataclass
class PlanResult:
    ok: bool
    message: str


# ---------------------- ROS Bridge ----------------------
class MoveItBridge(Node):
    def __init__(self):
        super().__init__("moveit_pyqt_gui_node")

        # ---- Params (đổi theo robot bạn)
        self.declare_parameter("group_name", "arm")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_link", "link_6")  # tool0 / ee_link của bạn

        # If your move_group namespace is /move_group, set move_action_name="/move_group/move_action"
        self.declare_parameter("move_action_name", "/move_action")
        self.declare_parameter("execute_action_name", "/execute_trajectory")

        # Tolerances
        self.declare_parameter("pos_tol_m", 0.005)      # 5mm
        self.declare_parameter("ori_tol_rad", 0.10)     # ~5.7deg
        self.declare_parameter("joint_tol_rad", 0.01)   # for joint-goal constraints (~0.57deg)

        # planning settings default
        self.declare_parameter("allowed_planning_time", 5.0)
        self.declare_parameter("num_planning_attempts", 5)
        self.declare_parameter("vel_scaling", 1.0)
        self.declare_parameter("acc_scaling", 1.0)

        self.group_name = self.get_parameter("group_name").value
        self.base_frame = self.get_parameter("base_frame").value
        self.ee_link = self.get_parameter("ee_link").value

        self.pos_tol_m = float(self.get_parameter("pos_tol_m").value)
        self.ori_tol_rad = float(self.get_parameter("ori_tol_rad").value)
        self.joint_tol_rad = float(self.get_parameter("joint_tol_rad").value)

        self.allowed_planning_time = float(self.get_parameter("allowed_planning_time").value)
        self.num_planning_attempts = int(self.get_parameter("num_planning_attempts").value)
        self.vel_scaling = float(self.get_parameter("vel_scaling").value)
        self.acc_scaling = float(self.get_parameter("acc_scaling").value)

        move_action_name = self.get_parameter("move_action_name").value
        execute_action_name = self.get_parameter("execute_action_name").value

        # ---- Joint state
        self.joint_state: Optional[JointState] = None
        self.joint_map: Dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 50)

        # ---- TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- Action clients
        self.move_client = ActionClient(self, MoveGroup, move_action_name)
        self.exec_client = ActionClient(self, ExecuteTrajectory, execute_action_name)
        self.last_trajectory = None  # moveit_msgs/RobotTrajectory

        # ---- Planner discovery (service)
        self._query_planners_client: Optional[rclpy.client.Client] = None

        # ---- Robot model data for slider limits
        self.group_joints: List[str] = []
        self.joint_limits_deg: Dict[str, Tuple[float, float]] = {}  # deg
        self._robot_desc_cached: Optional[str] = None
        self._srdf_cached: Optional[str] = None

        # ---- Camera
        self._cam_sub = None
        self._cam_topic = ""
        self._cam_type = ""
        self._last_qimage: Optional[QtGui.QImage] = None
        self._cam_fps = 0.0
        self._cam_last_ts = 0.0
        self._cam_frames = 0
        self._cam_fps_ts0 = time.time()
        self._cv_bridge = CvBridge() if HAS_CAMERA_DEPS else None

    # ---------- Joint / Pose ----------
    def _on_joint_state(self, msg: JointState):
        self.joint_state = msg
        for n, p in zip(msg.name, msg.position):
            self.joint_map[n] = p

    def get_current_joints(self) -> Dict[str, float]:
        return dict(self.joint_map)

    def get_current_pose(self) -> Optional[Pose]:
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.ee_link, rclpy.time.Time())
        except TransformException:
            return None
        p = Pose()
        p.position.x = tf.transform.translation.x
        p.position.y = tf.transform.translation.y
        p.position.z = tf.transform.translation.z
        p.orientation = tf.transform.rotation
        return p

    # ---------- Constraints ----------
    def _make_pose_goal_constraints(self, target_pose: Pose) -> Constraints:
        pc = PositionConstraint()
        pc.header.frame_id = self.base_frame
        pc.link_name = self.ee_link
        pc.weight = 1.0

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        d = self.pos_tol_m * 2.0
        box.dimensions = [d, d, d]

        region = BoundingVolume()
        region.primitives.append(box)
        region.primitive_poses.append(Pose(position=target_pose.position, orientation=target_pose.orientation))
        pc.constraint_region = region

        oc = OrientationConstraint()
        oc.header.frame_id = self.base_frame
        oc.link_name = self.ee_link
        oc.orientation = target_pose.orientation
        oc.absolute_x_axis_tolerance = self.ori_tol_rad
        oc.absolute_y_axis_tolerance = self.ori_tol_rad
        oc.absolute_z_axis_tolerance = self.ori_tol_rad
        oc.weight = 1.0

        c = Constraints()
        c.name = "goal_pose"
        c.position_constraints = [pc]
        c.orientation_constraints = [oc]
        return c

    def _make_joint_goal_constraints(self, joint_targets_rad: Dict[str, float]) -> Constraints:
        c = Constraints()
        c.name = "goal_joints"
        jcs: List[JointConstraint] = []
        for jname, jpos in joint_targets_rad.items():
            jc = JointConstraint()
            jc.joint_name = jname
            jc.position = float(jpos)
            jc.tolerance_above = self.joint_tol_rad
            jc.tolerance_below = self.joint_tol_rad
            jc.weight = 1.0
            jcs.append(jc)
        c.joint_constraints = jcs
        return c

    # ---------- Planner discovery ----------
    def list_planner_service_candidates(self) -> List[str]:
        """
        Find services with type moveit_msgs/srv/QueryPlannerInterfaces.
        This avoids guessing namespace (/query_planner_interface vs /move_group/query_planner_interface).
        """
        out = []
        for name, types in self.get_service_names_and_types():
            if any("moveit_msgs/srv/QueryPlannerInterfaces" in t for t in types):
                out.append(name)
        return sorted(out)

    def query_planners_async(self, done_cb):
        """
        Returns mapping: pipeline_id -> planner_ids
        """
        cands = self.list_planner_service_candidates()
        if not cands:
            done_cb({}, "No QueryPlannerInterfaces service found. Is move_group running?")
            return

        srv_name = cands[0]  # pick first
        if self._query_planners_client is None or self._query_planners_client.srv_name != srv_name:
            self._query_planners_client = self.create_client(QueryPlannerInterfaces, srv_name)

        if not self._query_planners_client.wait_for_service(timeout_sec=1.5):
            done_cb({}, f"Planner service not available: {srv_name}")
            return

        req = QueryPlannerInterfaces.Request()
        fut = self._query_planners_client.call_async(req)

        def _on_done(f):
            try:
                res = f.result()
            except Exception as e:
                done_cb({}, f"Query planners failed: {e}")
                return
            mapping: Dict[str, List[str]] = {}
            for pi in res.planner_interfaces:
                mapping[pi.pipeline_id] = list(pi.planner_ids)
            done_cb(mapping, f"Loaded planners from: {srv_name}")

        fut.add_done_callback(_on_done)

    # ---------- Robot model load for joint sliders ----------
    def _parse_urdf_limits_deg(self, urdf_xml: str) -> Tuple[Dict[str, Tuple[float, float]], Dict[str, Tuple[str, str, str]]]:
        """
        Returns:
          limits_deg: joint -> (min_deg, max_deg)
          kinematic_map: joint -> (type, parent_link, child_link)
        """
        limits_deg: Dict[str, Tuple[float, float]] = {}
        kin: Dict[str, Tuple[str, str, str]] = {}

        root = ET.fromstring(urdf_xml)
        for j in root.findall("joint"):
            jname = j.get("name", "")
            jtype = j.get("type", "")
            parent = j.find("parent")
            child = j.find("child")
            parent_link = parent.get("link") if parent is not None else ""
            child_link = child.get("link") if child is not None else ""
            kin[jname] = (jtype, parent_link, child_link)

            if jtype in ("revolute", "continuous"):
                if jtype == "continuous":
                    limits_deg[jname] = (-180.0, 180.0)
                    continue
                lim = j.find("limit")
                if lim is not None and lim.get("lower") is not None and lim.get("upper") is not None:
                    lo = float(lim.get("lower"))
                    hi = float(lim.get("upper"))
                    limits_deg[jname] = (math.degrees(lo), math.degrees(hi))
                else:
                    limits_deg[jname] = (-180.0, 180.0)
            elif jtype == "prismatic":
                lim = j.find("limit")
                if lim is not None and lim.get("lower") is not None and lim.get("upper") is not None:
                    lo = float(lim.get("lower"))
                    hi = float(lim.get("upper"))
                    # show prismatic in mm in UI? for now: deg-like fallback
                    limits_deg[jname] = (lo * 1000.0, hi * 1000.0)
        return limits_deg, kin

    def _extract_group_joints_from_srdf(self, srdf_xml: str, group_name: str) -> Tuple[List[str], Optional[Tuple[str, str]]]:
        """
        Return (joint_list, chain(base_link, tip_link) or None)
        """
        root = ET.fromstring(srdf_xml)
        group = None
        for g in root.findall("group"):
            if g.get("name") == group_name:
                group = g
                break
        if group is None:
            return [], None

        joints = [j.get("name") for j in group.findall("joint") if j.get("name")]
        chain = group.find("chain")
        if chain is not None:
            base = chain.get("base_link")
            tip = chain.get("tip_link")
            if base and tip:
                return joints, (base, tip)
        return joints, None

    def _resolve_chain_joints(self, chain_base: str, chain_tip: str, kin_map: Dict[str, Tuple[str, str, str]]) -> List[str]:
        """
        Resolve joints along chain tip->base using URDF kin map.
        """
        # Build child_link -> (joint_name, parent_link)
        child_to_parent: Dict[str, Tuple[str, str]] = {}
        for jname, (_typ, parent, child) in kin_map.items():
            if child:
                child_to_parent[child] = (jname, parent)

        joints_rev: List[str] = []
        cur = chain_tip
        guard = 0
        while cur != chain_base and guard < 200:
            guard += 1
            if cur not in child_to_parent:
                return []
            jname, parent = child_to_parent[cur]
            joints_rev.append(jname)
            cur = parent
        joints_rev.reverse()
        return joints_rev

    def load_robot_model_async(self, node_fullname: str, done_cb):
        """
        Query robot_description + robot_description_semantic (SRDF) from a selected node, then:
          - compute group joints (prefer SRDF group)
          - compute joint limits from URDF (deg)
        """
        # Remote parameter access
        from rclpy.parameter_client import AsyncParameterClient

        pc = AsyncParameterClient(self, node_fullname)
        # NOTE: some nodes might not provide srdf; we handle gracefully
        fut = pc.get_parameters(["robot_description", "robot_description_semantic"])

        def _on_done(f):
            try:
                params = f.result()
            except Exception as e:
                done_cb(False, f"GetParameters failed from {node_fullname}: {e}")
                return

            urdf = ""
            srdf = ""
            if len(params) >= 1 and params[0].type_ != 0:
                urdf = params[0].value
            if len(params) >= 2 and params[1].type_ != 0:
                srdf = params[1].value

            if not urdf:
                done_cb(False, f"No robot_description on {node_fullname}")
                return

            self._robot_desc_cached = urdf
            self._srdf_cached = srdf

            limits_deg, kin_map = self._parse_urdf_limits_deg(urdf)
            self.joint_limits_deg = limits_deg

            gj: List[str] = []
            if srdf:
                direct_joints, chain = self._extract_group_joints_from_srdf(srdf, self.group_name)
                gj.extend([j for j in direct_joints if j])

                if chain is not None:
                    chain_j = self._resolve_chain_joints(chain[0], chain[1], kin_map)
                    if chain_j:
                        gj = chain_j  # chain usually defines full group joints

            # fallback: take joints from /joint_states
            if not gj and self.joint_state and self.joint_state.name:
                gj = list(self.joint_state.name)

            # keep only joints that exist in urdf limits or appear in joint_states
            if self.joint_state and self.joint_state.name:
                js_set = set(self.joint_state.name)
                gj = [j for j in gj if j in js_set or j in limits_deg]

            self.group_joints = gj
            done_cb(True, f"Loaded model from {node_fullname}. Group joints: {len(self.group_joints)}")

        fut.add_done_callback(_on_done)

    # ---------- Planning ----------
    def _fill_common_request(self, goal: MoveGroup.Goal, pipeline_id: str, planner_id: str):
        goal.request.group_name = self.group_name
        goal.request.num_planning_attempts = self.num_planning_attempts
        goal.request.allowed_planning_time = self.allowed_planning_time
        goal.request.max_velocity_scaling_factor = float(self.vel_scaling)
        goal.request.max_acceleration_scaling_factor = float(self.acc_scaling)

        if pipeline_id:
            goal.request.pipeline_id = pipeline_id
        if planner_id:
            goal.request.planner_id = planner_id

        if self.joint_state is not None:
            rs = RobotState()
            rs.joint_state = self.joint_state
            goal.request.start_state = rs

        goal.planning_options.plan_only = True  # Plan/Execute tách
        goal.planning_options.replan = False

    def plan_to_pose_async(self, target_pose: Pose, pipeline_id: str, planner_id: str, done_cb):
        if not self.move_client.wait_for_server(timeout_sec=2.0):
            done_cb(PlanResult(False, "MoveGroup action server not available"))
            return

        goal = MoveGroup.Goal()
        self._fill_common_request(goal, pipeline_id, planner_id)
        goal.request.goal_constraints = [self._make_pose_goal_constraints(target_pose)]

        send_future = self.move_client.send_goal_async(goal)

        def _on_goal_sent(fut):
            gh = fut.result()
            if not gh.accepted:
                done_cb(PlanResult(False, "MoveGroup goal rejected"))
                return
            result_future = gh.get_result_async()

            def _on_result(rf):
                res = rf.result().result
                if res.error_code.val == 1:
                    self.last_trajectory = res.planned_trajectory
                    done_cb(PlanResult(True, "Plan SUCCESS"))
                else:
                    self.last_trajectory = None
                    done_cb(PlanResult(False, f"Plan FAILED (error_code={res.error_code.val})"))

            result_future.add_done_callback(_on_result)

        send_future.add_done_callback(_on_goal_sent)

    def plan_to_joints_async(self, joint_targets_rad: Dict[str, float], pipeline_id: str, planner_id: str, done_cb):
        if not self.move_client.wait_for_server(timeout_sec=2.0):
            done_cb(PlanResult(False, "MoveGroup action server not available"))
            return

        goal = MoveGroup.Goal()
        self._fill_common_request(goal, pipeline_id, planner_id)
        goal.request.goal_constraints = [self._make_joint_goal_constraints(joint_targets_rad)]

        send_future = self.move_client.send_goal_async(goal)

        def _on_goal_sent(fut):
            gh = fut.result()
            if not gh.accepted:
                done_cb(PlanResult(False, "MoveGroup goal rejected"))
                return
            result_future = gh.get_result_async()

            def _on_result(rf):
                res = rf.result().result
                if res.error_code.val == 1:
                    self.last_trajectory = res.planned_trajectory
                    done_cb(PlanResult(True, "Plan SUCCESS"))
                else:
                    self.last_trajectory = None
                    done_cb(PlanResult(False, f"Plan FAILED (error_code={res.error_code.val})"))

            result_future.add_done_callback(_on_result)

        send_future.add_done_callback(_on_goal_sent)

    def execute_last_async(self, done_cb):
        if self.last_trajectory is None:
            done_cb("No trajectory. Click Plan first.")
            return
        if not self.exec_client.wait_for_server(timeout_sec=2.0):
            done_cb("ExecuteTrajectory action server not available")
            return

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = self.last_trajectory

        send_future = self.exec_client.send_goal_async(goal)

        def _on_goal_sent(fut):
            gh = fut.result()
            if not gh.accepted:
                done_cb("Execute goal rejected")
                return
            result_future = gh.get_result_async()

            def _on_result(rf):
                res = rf.result().result
                done_cb("Execute SUCCESS" if res.error_code.val == 1 else f"Execute FAILED (error_code={res.error_code.val})")

            result_future.add_done_callback(_on_result)

        send_future.add_done_callback(_on_goal_sent)

    # ---------- Camera ----------
    def list_image_topics(self) -> List[Tuple[str, str]]:
        """
        Returns list of (topic, type) where type in [sensor_msgs/msg/Image, sensor_msgs/msg/CompressedImage]
        """
        out: List[Tuple[str, str]] = []
        for name, types in self.get_topic_names_and_types():
            for t in types:
                if t == "sensor_msgs/msg/Image" or t == "sensor_msgs/msg/CompressedImage":
                    out.append((name, t))
        out.sort(key=lambda x: x[0])
        return out

    def set_camera_topic(self, topic: str, msg_type: str) -> str:
        if not HAS_CAMERA_DEPS:
            return "Camera deps missing (cv_bridge/cv2)."

        # destroy old
        if self._cam_sub is not None:
            try:
                self.destroy_subscription(self._cam_sub)
            except Exception:
                pass
            self._cam_sub = None

        self._cam_topic = topic
        self._cam_type = msg_type
        self._last_qimage = None

        self._cam_frames = 0
        self._cam_fps = 0.0
        self._cam_fps_ts0 = time.time()

        if msg_type == "sensor_msgs/msg/Image":
            self._cam_sub = self.create_subscription(Image, topic, self._on_image, 10)
        else:
            self._cam_sub = self.create_subscription(CompressedImage, topic, self._on_cimage, 10)
        return f"Subscribed: {topic} ({msg_type})"

    def _update_fps(self):
        self._cam_frames += 1
        now = time.time()
        dt = now - self._cam_fps_ts0
        if dt >= 1.0:
            self._cam_fps = float(self._cam_frames) / dt
            self._cam_frames = 0
            self._cam_fps_ts0 = now

    def _on_image(self, msg):
        try:
            cv = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            h, w = cv.shape[:2]
            rgb = cv2.cvtColor(cv, cv2.COLOR_BGR2RGB)
            qimg = QtGui.QImage(rgb.data, w, h, 3 * w, QtGui.QImage.Format_RGB888).copy()
            self._last_qimage = qimg
            self._update_fps()
        except Exception:
            pass

    def _on_cimage(self, msg):
        try:
            data = np.frombuffer(msg.data, dtype=np.uint8)
            cv = cv2.imdecode(data, cv2.IMREAD_COLOR)
            h, w = cv.shape[:2]
            rgb = cv2.cvtColor(cv, cv2.COLOR_BGR2RGB)
            qimg = QtGui.QImage(rgb.data, w, h, 3 * w, QtGui.QImage.Format_RGB888).copy()
            self._last_qimage = qimg
            self._update_fps()
        except Exception:
            pass

    def get_last_camera_qimage(self) -> Optional[QtGui.QImage]:
        return self._last_qimage

    def get_camera_fps(self) -> float:
        return float(self._cam_fps)


# ---------------------- GUI ----------------------
OCEAN_QSS = """
QMainWindow { background-color: #062A3A; }
QWidget { color: #EAF6FF; font-size: 12px; }

QGroupBox {
  border: 1px solid #1B6C8E;
  border-radius: 10px;
  margin-top: 10px;
  padding: 8px;
}
QGroupBox:title {
  subcontrol-origin: margin;
  left: 10px;
  padding: 0 6px;
  color: #AEEBFF;
  font-weight: bold;
}

QLabel { color: #EAF6FF; }
QTableWidget { background-color: #063246; border: 1px solid #1B6C8E; border-radius: 8px; gridline-color: #1B6C8E; }
QHeaderView::section { background-color: #0A3C55; color: #AEEBFF; border: 0px; padding: 6px; }

QComboBox, QDoubleSpinBox, QSpinBox, QLineEdit {
  background-color: #063246;
  border: 1px solid #1B6C8E;
  border-radius: 8px;
  padding: 4px 8px;
  color: #EAF6FF;
}
QComboBox::drop-down { border: 0px; }

QPushButton {
  background-color: #0A84FF;
  border: none;
  border-radius: 10px;
  padding: 8px 14px;
  color: white;
  font-weight: bold;
}
QPushButton:hover { background-color: #1B93FF; }
QPushButton:pressed { background-color: #0666C2; }

QTabWidget::pane {
  border: 1px solid #1B6C8E;
  border-radius: 10px;
  padding: 4px;
}
QTabBar::tab {
  background: #063246;
  border: 1px solid #1B6C8E;
  padding: 8px 12px;
  border-top-left-radius: 10px;
  border-top-right-radius: 10px;
}
QTabBar::tab:selected {
  background: #0A84FF;
  color: white;
}
QSlider::groove:horizontal { height: 8px; background: #0A3C55; border-radius: 4px; }
QSlider::handle:horizontal { width: 16px; margin: -6px 0; border-radius: 8px; background: #AEEBFF; }
"""


class JointRow(QtCore.QObject):
    """
    One row: [Label] [Slider] [SpinBox]
    Slider in 0.1 units (deg or mm depending).
    """
    value_changed = QtCore.pyqtSignal(str, float)  # joint_name, value (deg or mm)

    def __init__(self, joint_name: str, minv: float, maxv: float, unit: str = "deg", step: float = 0.1):
        super().__init__()
        self.joint_name = joint_name
        self.unit = unit
        self.step = step

        self.label = QtWidgets.QLabel(joint_name)
        self.label.setMinimumWidth(120)

        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.spin = QtWidgets.QDoubleSpinBox()
        self.spin.setDecimals(1 if step <= 0.1 else 0)
        self.spin.setSingleStep(step)
        self.spin.setRange(minv, maxv)

        # slider works in int
        self.slider_min = int(round(minv / step))
        self.slider_max = int(round(maxv / step))
        self.slider.setRange(self.slider_min, self.slider_max)

        self._lock = False
        self.slider.valueChanged.connect(self._on_slider)
        self.spin.valueChanged.connect(self._on_spin)

    def _on_slider(self, v: int):
        if self._lock:
            return
        self._lock = True
        val = float(v) * self.step
        self.spin.setValue(val)
        self._lock = False
        self.value_changed.emit(self.joint_name, val)

    def _on_spin(self, val: float):
        if self._lock:
            return
        self._lock = True
        self.slider.setValue(int(round(val / self.step)))
        self._lock = False
        self.value_changed.emit(self.joint_name, float(val))

    def set_value(self, val: float):
        self.spin.setValue(float(val))


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, bridge: MoveItBridge):
        super().__init__()
        self.bridge = bridge
        self.setWindowTitle("robot_moveit: Ocean GUI (Plan / Execute / Sliders / Astra)")

        self._planners_map: Dict[str, List[str]] = {}
        self._joint_rows: Dict[str, JointRow] = {}

        # Central layout
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        # Top status
        self.lbl_status = QtWidgets.QLabel("Status: Ready")
        self.lbl_status.setStyleSheet("font-size: 13px; font-weight: bold; color: #AEEBFF;")
        root.addWidget(self.lbl_status)

        # Split: Left current-state / Right tabs
        main = QtWidgets.QHBoxLayout()
        root.addLayout(main, 1)

        # ---- LEFT: Current State
        gb_state = QtWidgets.QGroupBox("Current State")
        main.addWidget(gb_state, 2)
        v_state = QtWidgets.QVBoxLayout(gb_state)

        # Pose box
        gb_pose = QtWidgets.QGroupBox("End-effector Pose")
        v_state.addWidget(gb_pose)
        g_pose = QtWidgets.QGridLayout(gb_pose)

        self.pose_xyz = QtWidgets.QLabel("XYZ: ---")
        self.pose_rpy = QtWidgets.QLabel("RPY: ---")
        self.pose_quat = QtWidgets.QLabel("Quat: ---")
        self.pose_frame = QtWidgets.QLabel(f"TF: {self.bridge.base_frame} -> {self.bridge.ee_link}")

        self.pose_xyz.setStyleSheet("font-size: 13px;")
        self.pose_rpy.setStyleSheet("font-size: 13px;")
        self.pose_quat.setStyleSheet("font-size: 12px; color: #D0F5FF;")
        self.pose_frame.setStyleSheet("font-size: 11px; color: #D0F5FF;")

        g_pose.addWidget(self.pose_frame, 0, 0, 1, 2)
        g_pose.addWidget(self.pose_xyz, 1, 0, 1, 2)
        g_pose.addWidget(self.pose_rpy, 2, 0, 1, 2)
        g_pose.addWidget(self.pose_quat, 3, 0, 1, 2)

        # Joints table
        gb_j = QtWidgets.QGroupBox("Joints (deg)")
        v_state.addWidget(gb_j, 1)
        vj = QtWidgets.QVBoxLayout(gb_j)

        self.tbl_joints = QtWidgets.QTableWidget(0, 2)
        self.tbl_joints.setHorizontalHeaderLabels(["Joint", "Value"])
        self.tbl_joints.horizontalHeader().setStretchLastSection(True)
        self.tbl_joints.verticalHeader().setVisible(False)
        self.tbl_joints.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.tbl_joints.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        vj.addWidget(self.tbl_joints)

        # ---- RIGHT: Tabs
        tabs = QtWidgets.QTabWidget()
        main.addWidget(tabs, 3)

        # Pose tab
        self.tab_pose = QtWidgets.QWidget()
        tabs.addTab(self.tab_pose, "Pose Goal")
        self._build_pose_tab(self.tab_pose)

        # Joint tab
        self.tab_joints = QtWidgets.QWidget()
        tabs.addTab(self.tab_joints, "Joint Sliders")
        self._build_joint_tab(self.tab_joints)

        # Camera tab
        self.tab_cam = QtWidgets.QWidget()
        tabs.addTab(self.tab_cam, "Astra Camera")
        self._build_camera_tab(self.tab_cam)

        # Bottom: planning settings + buttons
        gb_plan = QtWidgets.QGroupBox("Planning Settings")
        root.addWidget(gb_plan)
        grid = QtWidgets.QGridLayout(gb_plan)

        # MoveIt planners
        self.cmb_pipeline = QtWidgets.QComboBox()
        self.cmb_planner = QtWidgets.QComboBox()
        self.btn_refresh_planners = QtWidgets.QPushButton("Refresh Planners")
        self.btn_refresh_planners.clicked.connect(self.refresh_planners)

        self.cmb_pipeline.currentTextChanged.connect(self._on_pipeline_changed)

        grid.addWidget(QtWidgets.QLabel("Pipeline"), 0, 0)
        grid.addWidget(self.cmb_pipeline, 0, 1)
        grid.addWidget(QtWidgets.QLabel("Planner"), 0, 2)
        grid.addWidget(self.cmb_planner, 0, 3)
        grid.addWidget(self.btn_refresh_planners, 0, 4)

        # Planning numeric settings
        self.sp_time = QtWidgets.QDoubleSpinBox()
        self.sp_time.setRange(0.1, 60.0)
        self.sp_time.setValue(self.bridge.allowed_planning_time)
        self.sp_time.setSingleStep(0.5)

        self.sp_attempts = QtWidgets.QSpinBox()
        self.sp_attempts.setRange(1, 50)
        self.sp_attempts.setValue(self.bridge.num_planning_attempts)

        self.sp_vel = QtWidgets.QDoubleSpinBox()
        self.sp_vel.setRange(0.01, 1.0)
        self.sp_vel.setValue(self.bridge.vel_scaling)
        self.sp_vel.setSingleStep(0.05)

        self.sp_acc = QtWidgets.QDoubleSpinBox()
        self.sp_acc.setRange(0.01, 1.0)
        self.sp_acc.setValue(self.bridge.acc_scaling)
        self.sp_acc.setSingleStep(0.05)

        grid.addWidget(QtWidgets.QLabel("Time (s)"), 1, 0)
        grid.addWidget(self.sp_time, 1, 1)
        grid.addWidget(QtWidgets.QLabel("Attempts"), 1, 2)
        grid.addWidget(self.sp_attempts, 1, 3)

        grid.addWidget(QtWidgets.QLabel("Vel scaling"), 2, 0)
        grid.addWidget(self.sp_vel, 2, 1)
        grid.addWidget(QtWidgets.QLabel("Acc scaling"), 2, 2)
        grid.addWidget(self.sp_acc, 2, 3)

        # Global buttons
        hbtn = QtWidgets.QHBoxLayout()
        root.addLayout(hbtn)
        self.btn_plan = QtWidgets.QPushButton("PLAN (current tab)")
        self.btn_exec = QtWidgets.QPushButton("EXECUTE (last plan)")
        hbtn.addWidget(self.btn_plan)
        hbtn.addWidget(self.btn_exec)

        self.btn_plan.clicked.connect(lambda: self.on_plan(tabs.currentIndex()))
        self.btn_exec.clicked.connect(self.on_exec)

        # Timers
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh_ui)
        self.timer.start(100)  # 10 Hz

        self.cam_timer = QtCore.QTimer(self)
        self.cam_timer.timeout.connect(self.refresh_camera_view)
        self.cam_timer.start(50)  # 20 Hz

        # Theme
        self.setStyleSheet(OCEAN_QSS)

        # Kick-off auto load
        QtCore.QTimer.singleShot(300, self.refresh_planners)
        QtCore.QTimer.singleShot(500, self.refresh_camera_topics)
        QtCore.QTimer.singleShot(800, self.try_load_robot_model_for_sliders)

    # ---------- Tabs ----------
    def _build_pose_tab(self, w: QtWidgets.QWidget):
        v = QtWidgets.QVBoxLayout(w)

        gb = QtWidgets.QGroupBox("Target Pose (base frame)")
        v.addWidget(gb)
        g = QtWidgets.QGridLayout(gb)

        def add_spin(row, col, label, minv, maxv, step, decimals=1):
            lab = QtWidgets.QLabel(label)
            sp = QtWidgets.QDoubleSpinBox()
            sp.setDecimals(decimals)
            sp.setRange(minv, maxv)
            sp.setSingleStep(step)
            g.addWidget(lab, row, col)
            g.addWidget(sp, row, col + 1)
            return sp

        self.in_x = add_spin(0, 0, "X (mm)", -2000, 2000, 1.0)
        self.in_y = add_spin(0, 2, "Y (mm)", -2000, 2000, 1.0)
        self.in_z = add_spin(0, 4, "Z (mm)", -2000, 2000, 1.0)

        self.in_r = add_spin(1, 0, "Roll (deg)", -180, 180, 1.0)
        self.in_p = add_spin(1, 2, "Pitch (deg)", -180, 180, 1.0)
        self.in_yaw = add_spin(1, 4, "Yaw (deg)", -180, 180, 1.0)

        # convenience buttons
        hb = QtWidgets.QHBoxLayout()
        v.addLayout(hb)
        btn_from_current = QtWidgets.QPushButton("Fill from current pose")
        hb.addWidget(btn_from_current)
        hb.addStretch(1)

        def _fill():
            pose = self.bridge.get_current_pose()
            if pose is None:
                self.set_status("No TF pose yet.")
                return
            r, p, y = quat_to_rpy(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            self.in_x.setValue(pose.position.x * 1000.0)
            self.in_y.setValue(pose.position.y * 1000.0)
            self.in_z.setValue(pose.position.z * 1000.0)
            self.in_r.setValue(math.degrees(r))
            self.in_p.setValue(math.degrees(p))
            self.in_yaw.setValue(math.degrees(y))

        btn_from_current.clicked.connect(_fill)

    def _build_joint_tab(self, w: QtWidgets.QWidget):
        v = QtWidgets.QVBoxLayout(w)

        top = QtWidgets.QHBoxLayout()
        v.addLayout(top)

        self.btn_j_from_current = QtWidgets.QPushButton("Set sliders = current joints")
        self.btn_j_rebuild = QtWidgets.QPushButton("Rebuild sliders")
        top.addWidget(self.btn_j_from_current)
        top.addWidget(self.btn_j_rebuild)
        top.addStretch(1)

        self.btn_j_from_current.clicked.connect(self.set_sliders_from_current)
        self.btn_j_rebuild.clicked.connect(self.try_load_robot_model_for_sliders)

        gb = QtWidgets.QGroupBox("Joint Targets")
        v.addWidget(gb, 1)

        self.scroll = QtWidgets.QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setFrameShape(QtWidgets.QFrame.NoFrame)

        gbv = QtWidgets.QVBoxLayout(gb)
        gbv.addWidget(self.scroll)

        self.joint_container = QtWidgets.QWidget()
        self.joint_layout = QtWidgets.QGridLayout(self.joint_container)
        self.joint_layout.setColumnStretch(1, 1)
        self.scroll.setWidget(self.joint_container)

        hint = QtWidgets.QLabel("Tip: PLAN on this tab will send JointConstraints goal.")
        hint.setStyleSheet("color:#D0F5FF;")
        v.addWidget(hint)

    def _build_camera_tab(self, w: QtWidgets.QWidget):
        v = QtWidgets.QVBoxLayout(w)

        hb = QtWidgets.QHBoxLayout()
        v.addLayout(hb)

        self.cmb_cam = QtWidgets.QComboBox()
        self.btn_cam_refresh = QtWidgets.QPushButton("Refresh Topics")
        self.lbl_cam_info = QtWidgets.QLabel("Camera: (not selected)")
        self.lbl_cam_info.setStyleSheet("color:#D0F5FF;")

        hb.addWidget(QtWidgets.QLabel("Topic"))
        hb.addWidget(self.cmb_cam, 1)
        hb.addWidget(self.btn_cam_refresh)
        hb.addWidget(self.lbl_cam_info)

        self.btn_cam_refresh.clicked.connect(self.refresh_camera_topics)
        self.cmb_cam.currentIndexChanged.connect(self.on_cam_selected)

        self.lbl_cam_view = QtWidgets.QLabel("No camera")
        self.lbl_cam_view.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_cam_view.setMinimumHeight(320)
        self.lbl_cam_view.setStyleSheet("background-color:#031E2B; border:1px solid #1B6C8E; border-radius:10px;")
        v.addWidget(self.lbl_cam_view, 1)

        if not HAS_CAMERA_DEPS:
            warn = QtWidgets.QLabel("Camera disabled: missing cv_bridge/cv2")
            warn.setStyleSheet("color:#FFB3B3; font-weight:bold;")
            v.addWidget(warn)

    # ---------- Helpers ----------
    def set_status(self, msg: str):
        self.lbl_status.setText(f"Status: {msg}")

    def _selected_pipeline_planner(self) -> Tuple[str, str]:
        pipeline = self.cmb_pipeline.currentText().strip()
        planner = self.cmb_planner.currentText().strip()
        return pipeline, planner

    def _apply_planning_settings_to_bridge(self):
        self.bridge.allowed_planning_time = float(self.sp_time.value())
        self.bridge.num_planning_attempts = int(self.sp_attempts.value())
        self.bridge.vel_scaling = float(self.sp_vel.value())
        self.bridge.acc_scaling = float(self.sp_acc.value())

    # ---------- Refresh UI ----------
    def refresh_ui(self):
        # pose
        pose = self.bridge.get_current_pose()
        if pose is not None:
            r, p, y = quat_to_rpy(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            self.pose_xyz.setText(
                f"XYZ:  x={pose.position.x*1000:.1f} mm   y={pose.position.y*1000:.1f} mm   z={pose.position.z*1000:.1f} mm"
            )
            self.pose_rpy.setText(
                f"RPY:  r={math.degrees(r):.1f}°   p={math.degrees(p):.1f}°   y={math.degrees(y):.1f}°"
            )
            self.pose_quat.setText(
                f"Quat: x={pose.orientation.x:.4f}  y={pose.orientation.y:.4f}  z={pose.orientation.z:.4f}  w={pose.orientation.w:.4f}"
            )
        else:
            self.pose_xyz.setText("XYZ: ---")
            self.pose_rpy.setText("RPY: ---")
            self.pose_quat.setText("Quat: ---")

        # joints table
        joints = self.bridge.get_current_joints()
        if joints:
            names = list(joints.keys())
            if self.tbl_joints.rowCount() != len(names):
                self.tbl_joints.setRowCount(len(names))
                for i, n in enumerate(names):
                    self.tbl_joints.setItem(i, 0, QtWidgets.QTableWidgetItem(n))
                    self.tbl_joints.setItem(i, 1, QtWidgets.QTableWidgetItem(""))
            for i, n in enumerate(names):
                v = joints[n]
                deg = math.degrees(v)
                self.tbl_joints.item(i, 1).setText(f"{deg:.2f}")
        else:
            if self.tbl_joints.rowCount() != 0:
                self.tbl_joints.setRowCount(0)

    # ---------- Planner loading ----------
    def refresh_planners(self):
        self.set_status("Loading planners...")
        self.bridge.query_planners_async(self._on_planners_loaded)

    def _on_planners_loaded(self, mapping: Dict[str, List[str]], msg: str):
        QtCore.QTimer.singleShot(0, lambda: self._update_planners_ui(mapping, msg))

    def _update_planners_ui(self, mapping: Dict[str, List[str]], msg: str):
        self._planners_map = mapping
        self.cmb_pipeline.blockSignals(True)
        self.cmb_pipeline.clear()
        self.cmb_pipeline.addItem("")  # allow default
        for k in sorted(mapping.keys()):
            self.cmb_pipeline.addItem(k)
        self.cmb_pipeline.blockSignals(False)

        # pick first pipeline if exists
        if mapping:
            self.cmb_pipeline.setCurrentIndex(1)
        self._on_pipeline_changed(self.cmb_pipeline.currentText())

        self.set_status(msg)

    def _on_pipeline_changed(self, pipeline: str):
        pipeline = pipeline.strip()
        self.cmb_planner.clear()
        self.cmb_planner.addItem("")  # default planner
        if pipeline and pipeline in self._planners_map:
            for pid in sorted(self._planners_map[pipeline]):
                self.cmb_planner.addItem(pid)

    # ---------- Robot model -> sliders ----------
    def try_load_robot_model_for_sliders(self):
        # choose a likely node that has robot_description
        nodes = []
        for name, ns in self.bridge.get_node_names_and_namespaces():
            full = (ns.rstrip("/") + "/" + name).replace("//", "/")
            nodes.append(full)

        # prefer nodes containing "move_group" then "robot_state_publisher"
        preferred = None
        for cand in nodes:
            if "move_group" in cand:
                preferred = cand
                break
        if preferred is None:
            for cand in nodes:
                if "robot_state_publisher" in cand:
                    preferred = cand
                    break
        if preferred is None and nodes:
            preferred = nodes[0]

        if preferred is None:
            self.set_status("No nodes discovered for robot_description.")
            return

        self.set_status(f"Loading robot model from {preferred} ...")
        self.bridge.load_robot_model_async(preferred, self._on_model_loaded)

    def _on_model_loaded(self, ok: bool, msg: str):
        QtCore.QTimer.singleShot(0, lambda: self._rebuild_joint_sliders(ok, msg))

    def _rebuild_joint_sliders(self, ok: bool, msg: str):
        if not ok:
            self.set_status(msg)
            return

        # clear old layout
        while self.joint_layout.count():
            item = self.joint_layout.takeAt(0)
            w = item.widget()
            if w is not None:
                w.deleteLater()

        self._joint_rows.clear()

        joints = self.bridge.group_joints[:] if self.bridge.group_joints else list(self.bridge.get_current_joints().keys())
        if not joints:
            self.set_status("No joints to build sliders.")
            return

        for row, jname in enumerate(joints):
            # limits
            if jname in self.bridge.joint_limits_deg:
                mn, mx = self.bridge.joint_limits_deg[jname]
            else:
                mn, mx = (-180.0, 180.0)

            jr = JointRow(jname, mn, mx, unit="deg", step=0.1)
            self._joint_rows[jname] = jr

            self.joint_layout.addWidget(jr.label, row, 0)
            self.joint_layout.addWidget(jr.slider, row, 1)
            self.joint_layout.addWidget(jr.spin, row, 2)

        self.set_sliders_from_current()
        self.set_status(msg)

    def set_sliders_from_current(self):
        joints = self.bridge.get_current_joints()
        for j, jr in self._joint_rows.items():
            if j in joints:
                jr.set_value(math.degrees(joints[j]))

    # ---------- Camera ----------
    def refresh_camera_topics(self):
        self.cmb_cam.blockSignals(True)
        self.cmb_cam.clear()
        self.cmb_cam.addItem("(none)")
        topics = self.bridge.list_image_topics()
        for tname, ttype in topics:
            self.cmb_cam.addItem(f"{tname}  [{ttype}]", (tname, ttype))
        self.cmb_cam.blockSignals(False)

        # auto select likely astra topic
        if topics:
            idx = 0
            for i in range(self.cmb_cam.count()):
                data = self.cmb_cam.itemData(i)
                if not data:
                    continue
                tname, _tt = data
                if "astra" in tname.lower() or "camera" in tname.lower():
                    idx = i
                    break
            self.cmb_cam.setCurrentIndex(idx)
            self.on_cam_selected()

    def on_cam_selected(self):
        data = self.cmb_cam.currentData()
        if not data:
            self.lbl_cam_info.setText("Camera: (none)")
            return
        tname, ttype = data
        msg = self.bridge.set_camera_topic(tname, ttype)
        self.lbl_cam_info.setText(msg)

    def refresh_camera_view(self):
        if not HAS_CAMERA_DEPS:
            return
        qimg = self.bridge.get_last_camera_qimage()
        if qimg is None:
            return
        pix = QtGui.QPixmap.fromImage(qimg)
        pix = pix.scaled(self.lbl_cam_view.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        self.lbl_cam_view.setPixmap(pix)
        self.lbl_cam_view.setText("")
        self.lbl_cam_info.setText(f"{self.bridge._cam_topic} | FPS: {self.bridge.get_camera_fps():.1f}")

    # ---------- Actions ----------
    def on_plan(self, tab_index: int):
        self._apply_planning_settings_to_bridge()
        pipeline, planner = self._selected_pipeline_planner()

        # tab 0: pose, tab 1: joints (we keep this ordering)
        if tab_index == 0:
            x = self.in_x.value() / 1000.0
            y = self.in_y.value() / 1000.0
            z = self.in_z.value() / 1000.0
            rr = math.radians(self.in_r.value())
            pp = math.radians(self.in_p.value())
            yy = math.radians(self.in_yaw.value())
            qx, qy, qz, qw = rpy_to_quat(rr, pp, yy)

            target = Pose()
            target.position.x = x
            target.position.y = y
            target.position.z = z
            target.orientation.x = qx
            target.orientation.y = qy
            target.orientation.z = qz
            target.orientation.w = qw

            self.set_status("Planning (pose)...")

            def done_cb(res: PlanResult):
                QtCore.QTimer.singleShot(0, lambda: self.set_status(res.message))

            self.bridge.plan_to_pose_async(target, pipeline, planner, done_cb)

        elif tab_index == 1:
            # joint goal
            if not self._joint_rows:
                self.set_status("No joint sliders. Rebuild sliders first.")
                return

            targets_rad: Dict[str, float] = {}
            for j, jr in self._joint_rows.items():
                targets_rad[j] = math.radians(jr.spin.value())

            self.set_status("Planning (joints)...")

            def done_cb(res: PlanResult):
                QtCore.QTimer.singleShot(0, lambda: self.set_status(res.message))

            self.bridge.plan_to_joints_async(targets_rad, pipeline, planner, done_cb)

        else:
            # camera tab -> default to pose plan
            self.set_status("Switch to Pose Goal or Joint Sliders tab to plan.")

    def on_exec(self):
        self.set_status("Executing...")

        def done_cb(msg: str):
            QtCore.QTimer.singleShot(0, lambda: self.set_status(msg))

        self.bridge.execute_last_async(done_cb)


# ---------------------- main ----------------------
def main():
    rclpy.init()
    node = MoveItBridge()

    app = QtWidgets.QApplication([])
    win = MainWindow(node)
    win.resize(1200, 720)
    win.show()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_timer = QtCore.QTimer()
    spin_timer.timeout.connect(lambda: executor.spin_once(timeout_sec=0.0))
    spin_timer.start(5)

    try:
        app.exec_()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
