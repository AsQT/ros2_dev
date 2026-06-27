#!/usr/bin/env python3
"""Campaign runner: N randomized DRL pick-place cycles with CSV telemetry."""

import csv
import json
import math
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
import rclpy.time
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker

from robot_task_manager.action import DrlPickPlace


class DrlPickPlaceCampaignRunner(Node):
    def __init__(self) -> None:
        super().__init__("drl_pick_place_campaign_runner")

        self.declare_parameter("num_runs", 20)
        self.declare_parameter("save_csv", True)
        self.declare_parameter("output_dir", "")
        self.declare_parameter("startup_delay_sec", 0.0)
        self.declare_parameter("grasp_tcp_offset_z", 0.015)
        self.declare_parameter("place_xyz", [0.46, 0.12, 0.12])
        self.declare_parameter("place_z_offset_m", 0.0)
        self.declare_parameter("gripper_close_width_m", 0.025)
        self.declare_parameter("execute", True)
        self.declare_parameter("min_pick_z_m", 0.025)
        self.declare_parameter("tf_sample_hz", 50.0)
        self.declare_parameter("tf_tcp_frame", "tcp_link")
        self.declare_parameter("tf_base_frame", "base_link")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("wood_info_topic", "/sim/pick_wood_info")
        self.declare_parameter("box_info_topic", "/sim/obstacle_box_info")
        self.declare_parameter("action_name", "drl_pickplace")
        self.declare_parameter("obstacle_id", "obstacle_box")
        self.declare_parameter("object_timeout_sec", 30.0)
        self.declare_parameter("action_server_timeout_sec", 120.0)
        self.declare_parameter("planning_scene_timeout_sec", 20.0)
        self.declare_parameter("goal_timeout_sec", 420.0)

        self._wood_marker: Optional[Marker] = None
        self._box_marker: Optional[Marker] = None
        self._obstacle_object: Optional[CollisionObject] = None
        self._current_plan_stats: list[dict] = []
        self._plan_stats_lock = threading.Lock()
        self._traj_points: list[tuple] = []
        self._sampling_active = False
        self._current_stage: str = ""

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._wood_sub = self.create_subscription(
            Marker, str(self.get_parameter("wood_info_topic").value),
            self._on_wood_marker, qos,
        )
        self._box_sub = self.create_subscription(
            Marker, str(self.get_parameter("box_info_topic").value),
            self._on_box_marker, qos,
        )
        self._plan_stats_sub = self.create_subscription(
            String, "/drl/plan_stats", self._on_plan_stats, 10,
        )

        self._client = ActionClient(
            self, DrlPickPlace, str(self.get_parameter("action_name").value)
        )
        self._apply_scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self._get_scene_client = self.create_client(GetPlanningScene, "/get_planning_scene")
        self._respawn_client = self.create_client(Trigger, "/sim/respawn_objects")
        self._collision_object_pub = self.create_publisher(CollisionObject, "/collision_object", 10)
        self._obstacle_timer = self.create_timer(0.2, self._republish_obstacle)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------

    def _on_wood_marker(self, msg: Marker) -> None:
        if msg.type == Marker.CUBE and msg.scale.x > 0.0:
            self._wood_marker = msg

    def _on_box_marker(self, msg: Marker) -> None:
        if msg.type == Marker.CUBE and msg.scale.x > 0.0:
            self._box_marker = msg

    def _on_plan_stats(self, msg: String) -> None:
        try:
            stats = json.loads(msg.data)
            with self._plan_stats_lock:
                self._current_plan_stats.append(stats)
        except Exception as exc:
            self.get_logger().warn(f"Failed to parse plan_stats: {exc}")

    def _on_feedback(self, feedback_msg) -> None:
        self._current_stage = str(feedback_msg.feedback.current_stage)

    def _republish_obstacle(self) -> None:
        if self._obstacle_object is None:
            return
        self._obstacle_object.header.stamp = self.get_clock().now().to_msg()
        self._collision_object_pub.publish(self._obstacle_object)

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------

    def _spin_until(self, future, label: str, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                raise TimeoutError(f"{label} timed out after {timeout:.1f}s")
            rclpy.spin_once(self, timeout_sec=0.05)

    def _call_respawn(self) -> None:
        timeout = float(self.get_parameter("planning_scene_timeout_sec").value)
        if not self._respawn_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("/sim/respawn_objects not available")
        future = self._respawn_client.call_async(Trigger.Request())
        self._spin_until(future, "respawn", timeout)
        resp = future.result()
        if resp is None or not resp.success:
            raise RuntimeError(f"Respawn failed: {getattr(resp, 'message', 'no response')}")
        self.get_logger().info(f"[respawn] {resp.message}")

    def _wait_for_fresh_markers(self) -> tuple[Marker, Marker]:
        self._wood_marker = None
        self._box_marker = None
        timeout = float(self.get_parameter("object_timeout_sec").value)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and (self._wood_marker is None or self._box_marker is None):
            if time.monotonic() > deadline:
                raise TimeoutError("Timed out waiting for fresh markers")
            rclpy.spin_once(self, timeout_sec=0.1)
        assert self._wood_marker is not None
        assert self._box_marker is not None
        return self._wood_marker, self._box_marker

    def _clear_obstacle(self) -> None:
        frame_id = str(self.get_parameter("frame_id").value)
        object_id = str(self.get_parameter("obstacle_id").value)
        obj = CollisionObject()
        obj.header.frame_id = frame_id
        obj.id = object_id
        obj.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)
        req = ApplyPlanningScene.Request()
        req.scene = scene
        timeout = float(self.get_parameter("planning_scene_timeout_sec").value)
        future = self._apply_scene_client.call_async(req)
        self._spin_until(future, "clear obstacle", timeout)

    def _add_obstacle(self, marker: Marker) -> None:
        frame_id = str(self.get_parameter("frame_id").value)
        object_id = str(self.get_parameter("obstacle_id").value)
        timeout = float(self.get_parameter("planning_scene_timeout_sec").value)
        if not self._apply_scene_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("/apply_planning_scene not available")
        self._clear_obstacle()
        obj = CollisionObject()
        obj.header.frame_id = frame_id
        obj.id = object_id
        obj.operation = CollisionObject.ADD
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [float(marker.scale.x), float(marker.scale.y), float(marker.scale.z)]
        pose = Pose()
        pose.position.x = float(marker.pose.position.x)
        pose.position.y = float(marker.pose.position.y)
        pose.position.z = float(marker.pose.position.z)
        pose.orientation = marker.pose.orientation
        obj.primitives.append(prim)
        obj.primitive_poses.append(pose)
        self._obstacle_object = obj
        self._republish_obstacle()
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self._apply_scene_client.call_async(req)
        self._spin_until(future, "add obstacle", timeout)
        resp = future.result()
        if resp is None or not resp.success:
            raise RuntimeError("Failed to add obstacle to PlanningScene")

    def _build_goal(self, wood_marker: Marker) -> DrlPickPlace.Goal:
        frame_id = str(self.get_parameter("frame_id").value)
        wood_center_z = float(wood_marker.pose.position.z)
        grasp_tcp_offset_z = float(self.get_parameter("grasp_tcp_offset_z").value)
        pick_z = max(
            wood_center_z + grasp_tcp_offset_z,
            float(self.get_parameter("min_pick_z_m").value),
        )
        place_xyz = [float(v) for v in self.get_parameter("place_xyz").value]
        place_xyz[2] += float(self.get_parameter("place_z_offset_m").value)

        def _ps(xyz):
            p = PoseStamped()
            p.header.stamp = self.get_clock().now().to_msg()
            p.header.frame_id = frame_id
            p.pose.position.x = float(xyz[0])
            p.pose.position.y = float(xyz[1])
            p.pose.position.z = float(xyz[2])
            p.pose.orientation.x = 0.7071068
            p.pose.orientation.y = 0.7071068
            p.pose.orientation.z = 0.0
            p.pose.orientation.w = 0.0
            return p

        goal = DrlPickPlace.Goal()
        goal.target_pick = _ps([
            float(wood_marker.pose.position.x),
            float(wood_marker.pose.position.y),
            pick_z,
        ])
        goal.target_place = _ps(place_xyz)
        goal.gripper_close_width_m = float(self.get_parameter("gripper_close_width_m").value)
        goal.execute = bool(self.get_parameter("execute").value)
        return goal

    # -------------------------------------------------------------------------
    # TF trajectory sampling
    # -------------------------------------------------------------------------

    def _start_tf_sampling(self) -> None:
        self._traj_points = []
        self._sampling_active = True
        self._traj_t0 = time.monotonic()
        tcp_frame = str(self.get_parameter("tf_tcp_frame").value)
        base_frame = str(self.get_parameter("tf_base_frame").value)
        hz = max(1.0, float(self.get_parameter("tf_sample_hz").value))
        interval = 1.0 / hz

        def _loop():
            while self._sampling_active:
                loop_t0 = time.monotonic()
                try:
                    tf = self._tf_buffer.lookup_transform(
                        base_frame, tcp_frame, rclpy.time.Time()
                    )
                    t = loop_t0 - self._traj_t0
                    tr = tf.transform.translation
                    self._traj_points.append((t, tr.x, tr.y, tr.z))
                except Exception:
                    pass
                to_sleep = interval - (time.monotonic() - loop_t0)
                if to_sleep > 0:
                    time.sleep(to_sleep)

        self._tf_thread = threading.Thread(target=_loop, daemon=True)
        self._tf_thread.start()

    def _stop_tf_sampling(self) -> list[tuple]:
        self._sampling_active = False
        if hasattr(self, "_tf_thread"):
            self._tf_thread.join(timeout=2.0)
        return list(self._traj_points)

    @staticmethod
    def _path_length(traj: list[tuple]) -> float:
        total = 0.0
        for i in range(1, len(traj)):
            dx = traj[i][1] - traj[i - 1][1]
            dy = traj[i][2] - traj[i - 1][2]
            dz = traj[i][3] - traj[i - 1][3]
            total += math.sqrt(dx * dx + dy * dy + dz * dz)
        return total

    # -------------------------------------------------------------------------
    # Single run
    # -------------------------------------------------------------------------

    def _run_one(self, run_id: int) -> dict:
        self.get_logger().info(f"\n{'='*50}\n[Campaign] Run {run_id}\n{'='*50}")
        m: dict = {
            "run_id": run_id,
            "success": False,
            "failed_stage": "",
            "duration_s": 0.0,
            "pick_x": 0.0, "pick_y": 0.0, "pick_z": 0.0,
            "place_x": 0.0, "place_y": 0.0, "place_z": 0.0,
            "wood_x": 0.0, "wood_y": 0.0, "wood_z": 0.0,
            "wood_size_x": 0.0, "wood_size_y": 0.0, "wood_size_z": 0.0,
            "obstacle_x": 0.0, "obstacle_y": 0.0, "obstacle_z": 0.0,
            "obstacle_size_x": 0.0, "obstacle_size_y": 0.0, "obstacle_size_z": 0.0,
            "planning_time_pick_s": 0.0, "planning_time_place_s": 0.0,
            "num_rl_waypoints_pick": 0, "num_rl_waypoints_place": 0,
            "planned_path_len_pick_m": 0.0, "planned_path_len_place_m": 0.0,
            "convergence_dist_pick_m": 0.0, "convergence_dist_place_m": 0.0,
            "actual_path_len_m": 0.0,
            "num_traj_samples": 0,
            "final_pos_error_m": 0.0,
        }
        t_start = time.monotonic()

        try:
            self._call_respawn()
            wood_marker, box_marker = self._wait_for_fresh_markers()
            self._add_obstacle(box_marker)
            goal = self._build_goal(wood_marker)

            m.update({
                "wood_x": float(wood_marker.pose.position.x),
                "wood_y": float(wood_marker.pose.position.y),
                "wood_z": float(wood_marker.pose.position.z),
                "wood_size_x": float(wood_marker.scale.x),
                "wood_size_y": float(wood_marker.scale.y),
                "wood_size_z": float(wood_marker.scale.z),
                "obstacle_x": float(box_marker.pose.position.x),
                "obstacle_y": float(box_marker.pose.position.y),
                "obstacle_z": float(box_marker.pose.position.z),
                "obstacle_size_x": float(box_marker.scale.x),
                "obstacle_size_y": float(box_marker.scale.y),
                "obstacle_size_z": float(box_marker.scale.z),
                "pick_x": goal.target_pick.pose.position.x,
                "pick_y": goal.target_pick.pose.position.y,
                "pick_z": goal.target_pick.pose.position.z,
                "place_x": goal.target_place.pose.position.x,
                "place_y": goal.target_place.pose.position.y,
                "place_z": goal.target_place.pose.position.z,
            })

            server_timeout = float(self.get_parameter("action_server_timeout_sec").value)
            if not self._client.wait_for_server(timeout_sec=server_timeout):
                raise RuntimeError("DrlPickPlace action server not available")

            with self._plan_stats_lock:
                self._current_plan_stats.clear()
            self._current_stage = ""

            send_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
            rclpy.spin_until_future_complete(self, send_future)
            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                raise RuntimeError("Goal rejected by action server")

            self._start_tf_sampling()

            result_future = goal_handle.get_result_async()
            goal_timeout = float(self.get_parameter("goal_timeout_sec").value)
            deadline = time.monotonic() + goal_timeout
            while rclpy.ok() and not result_future.done():
                if time.monotonic() > deadline:
                    raise TimeoutError(f"Goal timed out after {goal_timeout:.0f}s")
                rclpy.spin_once(self, timeout_sec=0.05)

            traj = self._stop_tf_sampling()
            wrapped = result_future.result()
            result = wrapped.result

            m["success"] = bool(result.success)
            m["failed_stage"] = "" if result.success else str(result.failed_stage)
            m["duration_s"] = time.monotonic() - t_start
            m["num_traj_samples"] = len(traj)
            m["actual_path_len_m"] = self._path_length(traj)

            if traj:
                fx, fy, fz = traj[-1][1], traj[-1][2], traj[-1][3]
                px = goal.target_place.pose.position.x
                py = goal.target_place.pose.position.y
                pz = goal.target_place.pose.position.z
                m["final_pos_error_m"] = math.sqrt(
                    (fx - px) ** 2 + (fy - py) ** 2 + (fz - pz) ** 2
                )

            with self._plan_stats_lock:
                stats = list(self._current_plan_stats)
            if stats:
                s = stats[0]
                m["planning_time_pick_s"] = s.get("planning_time_s", 0.0)
                m["num_rl_waypoints_pick"] = s.get("num_rl_waypoints", 0)
                m["planned_path_len_pick_m"] = s.get("planned_path_len_m", 0.0)
                m["convergence_dist_pick_m"] = s.get("convergence_dist_m", 0.0)
            if len(stats) >= 2:
                s = stats[1]
                m["planning_time_place_s"] = s.get("planning_time_s", 0.0)
                m["num_rl_waypoints_place"] = s.get("num_rl_waypoints", 0)
                m["planned_path_len_place_m"] = s.get("planned_path_len_m", 0.0)
                m["convergence_dist_place_m"] = s.get("convergence_dist_m", 0.0)

            tag = "PASS" if m["success"] else "FAIL"
            self.get_logger().info(
                f"[Campaign] Run {run_id} {tag} | "
                f"duration={m['duration_s']:.1f}s | "
                f"path={m['actual_path_len_m']:.3f}m | "
                f"stage={result.failed_stage or 'DONE'}"
            )

        except Exception as exc:
            self._stop_tf_sampling()
            m["failed_stage"] = "EXCEPTION"
            m["duration_s"] = time.monotonic() - t_start
            self.get_logger().error(f"[Campaign] Run {run_id} exception: {exc}")

        return m

    # -------------------------------------------------------------------------
    # CSV output
    # -------------------------------------------------------------------------

    def _save_csv(
        self,
        metrics_list: list[dict],
        traj_list: list[list[tuple]],
        output_dir: Path,
    ) -> None:
        output_dir.mkdir(parents=True, exist_ok=True)
        if not metrics_list:
            return

        summary_path = output_dir / "runs_summary.csv"
        fieldnames = list(metrics_list[0].keys())
        with open(summary_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(metrics_list)

        for metrics, traj in zip(metrics_list, traj_list):
            run_id = metrics.get("run_id", 0)
            traj_path = output_dir / f"run_{run_id:03d}_trajectory.csv"
            with open(traj_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["time_s", "tcp_x_m", "tcp_y_m", "tcp_z_m"])
                for row in traj:
                    writer.writerow([f"{v:.6f}" for v in row])

    # -------------------------------------------------------------------------
    # Main loop
    # -------------------------------------------------------------------------

    def run(self) -> int:
        num_runs = int(self.get_parameter("num_runs").value)
        save_csv = bool(self.get_parameter("save_csv").value)
        output_dir_param = str(self.get_parameter("output_dir").value)

        if save_csv:
            if output_dir_param:
                output_dir = Path(output_dir_param)
            else:
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                output_dir = Path.home() / "ros2_dev_2" / "reports" / f"campaign_{ts}"
            self.get_logger().info(f"[Campaign] Output dir: {output_dir}")
        else:
            output_dir = Path("/tmp/campaign_dry_run")

        startup_delay = float(self.get_parameter("startup_delay_sec").value)
        if startup_delay > 0.0:
            self.get_logger().info(
                f"[Campaign] Waiting {startup_delay:.0f}s for system startup..."
            )
            deadline = time.monotonic() + startup_delay
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.5)

        all_metrics: list[dict] = []
        all_trajs: list[list[tuple]] = []

        for i in range(1, num_runs + 1):
            m = self._run_one(i)
            all_metrics.append(m)
            all_trajs.append(list(self._traj_points))

            if save_csv:
                self._save_csv(all_metrics, all_trajs, output_dir)

        successes = sum(1 for m in all_metrics if m["success"])
        self.get_logger().info(
            f"\n[Campaign] DONE: {successes}/{num_runs} runs succeeded"
        )
        if save_csv:
            self.get_logger().info(f"[Campaign] Results saved to {output_dir}")

        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DrlPickPlaceCampaignRunner()
    code = 0
    try:
        code = node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted by user")
        code = 130
    except Exception as exc:
        node.get_logger().error(str(exc))
        code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
