import math
import threading
from typing import Dict, List, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints,
    DisplayTrajectory,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.srv import QueryPlannerInterfaces
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener

from robot_gui.utils.angles import quaternion_to_rpy, rpy_to_quaternion
from robot_task_manager.action import CheckerBoard
from robot_task_manager.action import MoveToPose
from robot_task_manager.action import MoveToPoseCartesian

class PeeBackend(Node):
    """Backend for TF (base_link -> tcp_lik) and MoveIt plan/execute."""

    def __init__(self) -> None:
        super().__init__("robot_pee_gui")

        self.declare_parameter("group_name", "arm")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_link", "tcp_lik")
        self.declare_parameter("move_group_action_name", "/move_action")
        self.declare_parameter("execute_action_name", "/execute_trajectory")
        self.declare_parameter("query_planners_service_name", "/query_planner_interface")
        self.declare_parameter("display_topic", "/display_planned_path")
        self.declare_parameter("allowed_planning_time", 5.0)
        self.declare_parameter("num_planning_attempts", 10)
        self.declare_parameter("position_tolerance", 0.003)
        self.declare_parameter("orientation_tolerance_deg", 2.0)
        self.declare_parameter("max_velocity_scaling", 0.3)
        self.declare_parameter("max_acceleration_scaling", 0.3)

        self.declare_parameter("checkerboard_action_name", "/checker_board")
        self.declare_parameter("move_to_pose_action_name", "/move_to_pose")
        self.declare_parameter("move_to_pose_cartesian_action_name", "/move_to_pose_cartesian")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self.move_group_client = ActionClient(self, MoveGroup, self.get_parameter("move_group_action_name").value)
        self.execute_client = ActionClient(self, ExecuteTrajectory, self.get_parameter("execute_action_name").value)
        self.query_planners_client = self.create_client(QueryPlannerInterfaces, self.get_parameter("query_planners_service_name").value)
        self.display_pub = self.create_publisher(DisplayTrajectory, self.get_parameter("display_topic").value, 10)

        self.checkerboard_client = ActionClient(
            self,
            CheckerBoard,
            self.get_parameter("checkerboard_action_name").value,
        )

        self.move_to_pose_client = ActionClient(
            self,
            MoveToPose,
            self.get_parameter("move_to_pose_action_name").value,
        )

        self.move_to_pose_cartesian_client = ActionClient(
            self,
            MoveToPoseCartesian,
            self.get_parameter("move_to_pose_cartesian_action_name").value,
        )

        self._last_planned_trajectory = None
        self._last_trajectory_start = None

    @staticmethod
    def _wait_future(future, timeout_sec: float):
        evt = threading.Event()
        future.add_done_callback(lambda _: evt.set())
        if not evt.wait(timeout_sec):
            return None
        return future.result()

    @staticmethod
    def moveit_error_name(code: int) -> str:
        for key, value in MoveItErrorCodes.__dict__.items():
            if key.isupper() and isinstance(value, int) and value == code:
                return key
        return f"UNKNOWN_{code}"

    def lookup_pose(self, ref_frame: str, child_frame: str) -> Tuple[bool, Dict[str, float], str]:
        try:
            tf_msg = self.tf_buffer.lookup_transform(ref_frame, child_frame, rclpy.time.Time())
            t = tf_msg.transform.translation
            q = tf_msg.transform.rotation
            roll, pitch, yaw = quaternion_to_rpy(q.x, q.y, q.z, q.w)
            data = {
                "x": t.x,
                "y": t.y,
                "z": t.z,
                "roll": math.degrees(roll),
                "pitch": math.degrees(pitch),
                "yaw": math.degrees(yaw),
            }
            return True, data, f"Đọc TF thành công: {ref_frame} <- {child_frame}"
        except TransformException as ex:
            return False, {}, f"Không đọc được TF {ref_frame} <- {child_frame}: {ex}"
        except Exception as ex:
            return False, {}, f"Lỗi không xác định khi đọc TF: {ex}"

    def lookup_fixed_pose(self) -> Tuple[bool, Dict[str, float], str]:
        return self.lookup_pose(self.default_base_frame(), self.default_ee_link())

    def get_planner_options(self) -> Tuple[Dict[str, List[str]], str]:
        if not self.query_planners_client.wait_for_service(timeout_sec=2.0):
            return {}, "Không thấy service /query_planner_interface. Sẽ dùng pipeline/planner mặc định."

        future = self.query_planners_client.call_async(QueryPlannerInterfaces.Request())
        response = self._wait_future(future, 5.0)
        if response is None:
            return {}, "Timeout khi gọi /query_planner_interface."

        options: Dict[str, List[str]] = {}
        for desc in response.planner_interfaces:
            pipeline = str(desc.pipeline_id).strip() or str(desc.name).strip() or "(default)"
            planners = options.setdefault(pipeline, [])
            for planner_id in desc.planner_ids:
                planner = str(planner_id).strip()
                if planner and planner not in planners:
                    planners.append(planner)

        for pipeline in options:
            options[pipeline].sort()

        if not options:
            return {}, "MoveIt không trả về pipeline/planner nào. Sẽ dùng mặc định."
        return options, f"Đọc được {len(options)} pipeline từ /query_planner_interface."

    def _build_goal_constraints(
        self,
        ref_frame: str,
        link_name: str,
        x: float,
        y: float,
        z: float,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
        pos_tol: float,
        ori_tol_deg: float,
    ) -> Constraints:
        qx, qy, qz, qw = rpy_to_quaternion(
            math.radians(roll_deg),
            math.radians(pitch_deg),
            math.radians(yaw_deg),
        )

        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [max(float(pos_tol), 1e-5)]

        pos_c = PositionConstraint()
        pos_c.header.frame_id = ref_frame
        pos_c.link_name = link_name
        pos_c.constraint_region.primitives.append(sphere)
        pos_c.constraint_region.primitive_poses.append(pose)
        pos_c.weight = 1.0

        ori_c = OrientationConstraint()
        ori_c.header.frame_id = ref_frame
        ori_c.link_name = link_name
        ori_c.orientation = pose.orientation
        ori_tol_rad = math.radians(max(float(ori_tol_deg), 0.01))
        ori_c.absolute_x_axis_tolerance = ori_tol_rad
        ori_c.absolute_y_axis_tolerance = ori_tol_rad
        ori_c.absolute_z_axis_tolerance = ori_tol_rad
        ori_c.weight = 1.0

        c = Constraints()
        c.position_constraints.append(pos_c)
        c.orientation_constraints.append(ori_c)
        return c

    def plan_to_pose(
        self,
        group_name: str,
        x: float,
        y: float,
        z: float,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
        pos_tol: float,
        ori_tol_deg: float,
        vel_scale: float,
        acc_scale: float,
        pipeline_id: str = "",
        planner_id: str = "",
    ) -> Tuple[bool, str]:
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            return False, "Không thấy action server MoveGroup. Hãy kiểm tra move_group đang chạy."

        ref_frame = self.default_base_frame()
        link_name = self.default_ee_link()

        goal = MoveGroup.Goal()
        goal.request.group_name = group_name
        goal.request.num_planning_attempts = int(self.get_parameter("num_planning_attempts").value)
        goal.request.allowed_planning_time = float(self.get_parameter("allowed_planning_time").value)
        goal.request.max_velocity_scaling_factor = float(vel_scale)
        goal.request.max_acceleration_scaling_factor = float(acc_scale)
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints.append(
            self._build_goal_constraints(
                ref_frame=ref_frame,
                link_name=link_name,
                x=x,
                y=y,
                z=z,
                roll_deg=roll_deg,
                pitch_deg=pitch_deg,
                yaw_deg=yaw_deg,
                pos_tol=pos_tol,
                ori_tol_deg=ori_tol_deg,
            )
        )
        if pipeline_id.strip():
            goal.request.pipeline_id = pipeline_id.strip()
        if planner_id.strip():
            goal.request.planner_id = planner_id.strip()

        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        send_goal_future = self.move_group_client.send_goal_async(goal)
        goal_handle = self._wait_future(send_goal_future, 10.0)
        if goal_handle is None:
            return False, "Timeout khi gửi goal tới MoveGroup."
        if not goal_handle.accepted:
            return False, "MoveGroup từ chối goal."

        result_future = goal_handle.get_result_async()
        result_wrap = self._wait_future(result_future, 60.0)
        if result_wrap is None:
            return False, "Timeout khi chờ kết quả planning."

        result = result_wrap.result
        code = int(result.error_code.val)
        code_name = self.moveit_error_name(code)
        if code != MoveItErrorCodes.SUCCESS:
            self._last_planned_trajectory = None
            self._last_trajectory_start = None
            return False, f"Planning thất bại. error_code={code} ({code_name})"

        if len(result.planned_trajectory.joint_trajectory.points) == 0:
            self._last_planned_trajectory = None
            self._last_trajectory_start = None
            return False, "Planning trả về SUCCESS nhưng trajectory rỗng."

        self._last_planned_trajectory = result.planned_trajectory
        self._last_trajectory_start = result.trajectory_start

        display = DisplayTrajectory()
        display.trajectory_start = result.trajectory_start
        display.trajectory.append(result.planned_trajectory)
        self.display_pub.publish(display)

        return True, (
            f"Plan thành công. points={len(result.planned_trajectory.joint_trajectory.points)}, "
            f"planning_time={result.planning_time:.3f}s"
        )

    def execute_last_plan(self) -> Tuple[bool, str]:
        if self._last_planned_trajectory is None:
            return False, "Chưa có quỹ đạo nào được plan."

        if not self.execute_client.wait_for_server(timeout_sec=5.0):
            return False, "Không thấy action server ExecuteTrajectory."

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = self._last_planned_trajectory
        goal.controller_names = []

        send_goal_future = self.execute_client.send_goal_async(goal)
        goal_handle = self._wait_future(send_goal_future, 10.0)
        if goal_handle is None:
            return False, "Timeout khi gửi goal ExecuteTrajectory."
        if not goal_handle.accepted:
            return False, "ExecuteTrajectory từ chối goal."

        result_future = goal_handle.get_result_async()
        result_wrap = self._wait_future(result_future, 120.0)
        if result_wrap is None:
            return False, "Timeout khi chờ robot execute."

        result = result_wrap.result
        code = int(result.error_code.val)
        code_name = self.moveit_error_name(code)
        if code != MoveItErrorCodes.SUCCESS:
            return False, f"Execute thất bại. error_code={code} ({code_name})"

        return True, "Execute thành công."

    def call_move_to_pose_action(
        self,
        x: float,
        y: float,
        z: float,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
        velocity_scale: float = 0.3,
    ) -> Tuple[bool, str]:
        if not self.move_to_pose_client.wait_for_server(timeout_sec=3.0):
            return False, "Không thấy action server /move_to_pose"

        qx, qy, qz, qw = rpy_to_quaternion(
            math.radians(roll_deg),
            math.radians(pitch_deg),
            math.radians(yaw_deg),
        )

        goal = MoveToPose.Goal()

        goal.target_pose.position.x = float(x)
        goal.target_pose.position.y = float(y)
        goal.target_pose.position.z = float(z)

        goal.target_pose.orientation.x = qx
        goal.target_pose.orientation.y = qy
        goal.target_pose.orientation.z = qz
        goal.target_pose.orientation.w = qw

        goal.velocity_scale = float(velocity_scale)

        send_goal_future = self.move_to_pose_client.send_goal_async(goal)
        goal_handle = self._wait_future(send_goal_future, 10.0)

        if goal_handle is None:
            return False, "Timeout khi gửi goal MoveToPose."

        if not goal_handle.accepted:
            return False, "MoveToPose action bị từ chối."

        result_future = goal_handle.get_result_async()
        result_wrap = self._wait_future(result_future, 120.0)

        if result_wrap is None:
            return False, "Timeout khi chờ kết quả MoveToPose."

        result = result_wrap.result

        if not result.success:
            return False, f"MoveToPose thất bại: {result.message}"

        return True, f"MoveToPose hoàn thành: {result.message}"

    def call_move_to_pose_cartesian_action(
        self,
        x: float,
        y: float,
        z: float,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
        velocity_scale: float = 0.3,
    ) -> Tuple[bool, str]:
        if not self.move_to_pose_cartesian_client.wait_for_server(timeout_sec=3.0):
            return False, "Không thấy action server /move_to_pose_cartesian"

        qx, qy, qz, qw = rpy_to_quaternion(
            math.radians(roll_deg),
            math.radians(pitch_deg),
            math.radians(yaw_deg),
        )

        goal = MoveToPoseCartesian.Goal()

        goal.target_pose.position.x = float(x)
        goal.target_pose.position.y = float(y)
        goal.target_pose.position.z = float(z)

        goal.target_pose.orientation.x = qx
        goal.target_pose.orientation.y = qy
        goal.target_pose.orientation.z = qz
        goal.target_pose.orientation.w = qw

        goal.velocity_scale = float(velocity_scale)

        send_goal_future = self.move_to_pose_cartesian_client.send_goal_async(goal)
        goal_handle = self._wait_future(send_goal_future, 10.0)

        if goal_handle is None:
            return False, "Timeout khi gửi goal MoveToPoseCartesian."

        if not goal_handle.accepted:
            return False, "MoveToPoseCartesian action bị từ chối."

        result_future = goal_handle.get_result_async()
        result_wrap = self._wait_future(result_future, 120.0)

        if result_wrap is None:
            return False, "Timeout khi chờ kết quả MoveToPoseCartesian."

        result = result_wrap.result

        if not result.success:
            return False, f"MoveToPoseCartesian thất bại: {result.message}"

        return True, f"MoveToPoseCartesian hoàn thành: {result.message}"

    def call_checkerboard_action(
        self,
        step: float,
        velocity_scale: float = 0.3,
    ) -> Tuple[bool, str]:
        if not self.checkerboard_client.wait_for_server(timeout_sec=3.0):
            return False, "Không thấy action server /checker_board"

        goal = CheckerBoard.Goal()

        goal.step = float(step)
        goal.velocity_scale = float(velocity_scale)

        send_goal_future = self.checkerboard_client.send_goal_async(goal)
        goal_handle = self._wait_future(send_goal_future, 10.0)

        if goal_handle is None:
            return False, "Timeout khi gửi goal CheckerBoard."

        if not goal_handle.accepted:
            return False, "CheckerBoard action bị từ chối."

        result_future = goal_handle.get_result_async()
        result_wrap = self._wait_future(result_future, 120.0)

        if result_wrap is None:
            return False, "Timeout khi chờ kết quả CheckerBoard."

        result = result_wrap.result

        if not result.success:
            return False, f"CheckerBoard thất bại: {result.message}"

        return True, f"CheckerBoard hoàn thành: {result.message}"
    
    def send_gui_action(
        self,
        action_key: str = "",
        action_name: str = "",
        frame_id: str = "",
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        roll_deg=None,
        pitch_deg=None,
        yaw_deg=None,
        qx=None,
        qy=None,
        qz=None,
        qw=None,
        velocity_scale: float = 0.5,
        step: float = 0.05,
        **kwargs,
    ) -> Tuple[bool, str]:

        # Lấy tên action từ GUI
        name = str(action_key or action_name).strip().lower()
        name = name.replace(".action", "")
        name = name.replace(" ", "_")

        # Nếu GUI truyền roll/pitch/yaw bằng tên khác thì vẫn nhận
        if roll_deg is None:
            roll_deg = kwargs.get("roll", kwargs.get("r", 0.0))
        if pitch_deg is None:
            pitch_deg = kwargs.get("pitch", kwargs.get("p", 0.0))
        if yaw_deg is None:
            yaw_deg = kwargs.get("yaw", kwargs.get("yaw_deg", 0.0))

        # Nếu GUI truyền quaternion thì dùng trực tiếp.
        # Nếu không truyền quaternion thì tự đổi từ RPY sang quaternion.
        if qx is None or qy is None or qz is None or qw is None:
            qx, qy, qz, qw = rpy_to_quaternion(
                math.radians(float(roll_deg)),
                math.radians(float(pitch_deg)),
                math.radians(float(yaw_deg)),
            )
        else:
            qx = float(qx)
            qy = float(qy)
            qz = float(qz)
            qw = float(qw)

            # Chuẩn hóa quaternion cho chắc
            norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
            if norm > 1e-9:
                qx /= norm
                qy /= norm
                qz /= norm
                qw /= norm

        # =====================================================
        # MoveToPoseCartesian
        # =====================================================
        if name in [
            "move_to_pose_cartesian",
            "movetoposecartesian",
            "move_to_pose_cartesian_action",
        ]:
            if not self.move_to_pose_cartesian_client.wait_for_server(timeout_sec=3.0):
                return False, "Không thấy action server /move_to_pose_cartesian"

            goal = MoveToPoseCartesian.Goal()

            goal.target_pose.position.x = float(x)
            goal.target_pose.position.y = float(y)
            goal.target_pose.position.z = float(z)

            goal.target_pose.orientation.x = qx
            goal.target_pose.orientation.y = qy
            goal.target_pose.orientation.z = qz
            goal.target_pose.orientation.w = qw

            goal.velocity_scale = float(velocity_scale)

            send_goal_future = self.move_to_pose_cartesian_client.send_goal_async(goal)
            goal_handle = self._wait_future(send_goal_future, 10.0)

            if goal_handle is None:
                return False, "Timeout khi gửi goal MoveToPoseCartesian."

            if not goal_handle.accepted:
                return False, "MoveToPoseCartesian action bị từ chối."

            result_future = goal_handle.get_result_async()
            result_wrap = self._wait_future(result_future, 120.0)

            if result_wrap is None:
                return False, "Timeout khi chờ kết quả MoveToPoseCartesian."

            result = result_wrap.result

            if not result.success:
                return False, f"MoveToPoseCartesian thất bại: {result.message}"

            return True, f"MoveToPoseCartesian hoàn thành: {result.message}"

        # =====================================================
        # MoveToPose
        # =====================================================
        if name in [
            "move_to_pose",
            "movetopose",
            "move_to_pose_action",
        ]:
            if not self.move_to_pose_client.wait_for_server(timeout_sec=3.0):
                return False, "Không thấy action server /move_to_pose"

            goal = MoveToPose.Goal()

            goal.target_pose.position.x = float(x)
            goal.target_pose.position.y = float(y)
            goal.target_pose.position.z = float(z)

            goal.target_pose.orientation.x = qx
            goal.target_pose.orientation.y = qy
            goal.target_pose.orientation.z = qz
            goal.target_pose.orientation.w = qw

            goal.velocity_scale = float(velocity_scale)

            send_goal_future = self.move_to_pose_client.send_goal_async(goal)
            goal_handle = self._wait_future(send_goal_future, 10.0)

            if goal_handle is None:
                return False, "Timeout khi gửi goal MoveToPose."

            if not goal_handle.accepted:
                return False, "MoveToPose action bị từ chối."

            result_future = goal_handle.get_result_async()
            result_wrap = self._wait_future(result_future, 120.0)

            if result_wrap is None:
                return False, "Timeout khi chờ kết quả MoveToPose."

            result = result_wrap.result

            if not result.success:
                return False, f"MoveToPose thất bại: {result.message}"

            return True, f"MoveToPose hoàn thành: {result.message}"

        # =====================================================
        # CheckerBoard
        # =====================================================
        if name in [
            "checkerboard",
            "checker_board",
            "checker_board_action",
        ]:
            if not self.checkerboard_client.wait_for_server(timeout_sec=3.0):
                return False, "Không thấy action server /checker_board"

            goal = CheckerBoard.Goal()
            goal.step = float(step)
            goal.velocity_scale = float(velocity_scale)

            send_goal_future = self.checkerboard_client.send_goal_async(goal)
            goal_handle = self._wait_future(send_goal_future, 10.0)

            if goal_handle is None:
                return False, "Timeout khi gửi goal CheckerBoard."

            if not goal_handle.accepted:
                return False, "CheckerBoard action bị từ chối."

            result_future = goal_handle.get_result_async()
            result_wrap = self._wait_future(result_future, 120.0)

            if result_wrap is None:
                return False, "Timeout khi chờ kết quả CheckerBoard."

            result = result_wrap.result

            if not result.success:
                return False, f"CheckerBoard thất bại: {result.message}"

            return True, f"CheckerBoard hoàn thành: {result.message}"

        return False, f"Không hỗ trợ action: {action_key}"
    
    def default_group_name(self) -> str:
        return str(self.get_parameter("group_name").value)

    def default_base_frame(self) -> str:
        return str(self.get_parameter("base_frame").value)

    def default_ee_link(self) -> str:
        return str(self.get_parameter("ee_link").value)

    def default_pos_tol(self) -> float:
        return float(self.get_parameter("position_tolerance").value)

    def default_ori_tol_deg(self) -> float:
        return float(self.get_parameter("orientation_tolerance_deg").value)

    def default_vel_scale(self) -> float:
        return float(self.get_parameter("max_velocity_scaling").value)

    def default_acc_scale(self) -> float:
        return float(self.get_parameter("max_acceleration_scaling").value)