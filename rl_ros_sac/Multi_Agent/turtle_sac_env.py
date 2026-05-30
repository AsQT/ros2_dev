import math
import time
from typing import Optional

import numpy as np

import gymnasium as gym # pyright: ignore[reportMissingImports]
from gymnasium import spaces # pyright: ignore[reportMissingImports]

import rclpy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, TeleportAbsolute, SetPen
from std_srvs.srv import Empty


class TurtleSacEnv(gym.Env):
    """
    Gymnasium environment bọc quanh ROS 2 turtlesim để train SAC.

    Hỗ trợ 2 chế độ:
    - Single env: namespace="" dùng /turtle1/pose, /reset, /spawn...
    - Multi env: namespace="env0" dùng /env0/turtle1/pose, /env0/reset, /env0/spawn...

    State:
        [x, y, sin(theta), cos(theta), dx, dy, distance]

    Action:
        action[0] -> linear velocity command
        action[1] -> angular velocity command

    Visual marker:
        Một turtle tên 'goal' được spawn/teleport tới vị trí goal để dễ quan sát.
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        namespace: str = "",
        node_name: str = "turtle_sac_env",
        show_goal_marker: bool = True,
        max_steps: int = 300,
        goal_radius: float = 0.6,
        dt: float = 0.1,
    ):
        super().__init__()

        self.namespace = namespace.strip("/")
        self.node_name = node_name
        self.show_goal_marker = show_goal_marker

        if not rclpy.ok():
            rclpy.init(args=None)

        # Node name cần khác nhau khi train nhiều env để tránh cảnh báo trùng tên.
        self.node = rclpy.create_node(self.node_name)

        self.cmd_pub = self.node.create_publisher(
            Twist,
            self.topic("turtle1/cmd_vel"),
            10,
        )

        self.pose_sub = self.node.create_subscription(
            Pose,
            self.topic("turtle1/pose"),
            self.pose_callback,
            10,
        )

        # Service của turtlesim tương ứng với namespace hiện tại.
        self.reset_cli = self.node.create_client(Empty, self.topic("reset"))

        # Service dùng để tạo/di chuyển marker goal.
        self.goal_name = "goal"
        self.spawn_cli = self.node.create_client(Spawn, self.topic("spawn"))
        self.goal_teleport_cli = self.node.create_client(
            TeleportAbsolute,
            self.topic(f"{self.goal_name}/teleport_absolute"),
        )
        self.goal_pen_cli = self.node.create_client(
            SetPen,
            self.topic(f"{self.goal_name}/set_pen"),
        )

        self.pose: Optional[Pose] = None
        self.goal = np.array([8.0, 8.0], dtype=np.float32)

        self.world_size = 11.088
        self.max_linear = 2.0
        self.max_angular = 3.0
        self.dt = float(dt)

        self.max_steps = int(max_steps)
        self.step_count = 0
        self.goal_radius = float(goal_radius)
        self.prev_dist = None

        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
            dtype=np.float32,
        )

        self.observation_space = spaces.Box(
            low=np.array([0.0, 0.0, -1.0, -1.0, -1.0, -1.0, 0.0], dtype=np.float32),
            high=np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.5], dtype=np.float32),
            dtype=np.float32,
        )

        self.wait_for_reset_service()
        self.wait_for_pose()

    def topic(self, name: str) -> str:
        """Tạo tên topic/service tuyệt đối theo namespace."""
        clean = name.strip("/")
        if self.namespace:
            return f"/{self.namespace}/{clean}"
        return f"/{clean}"

    def pose_callback(self, msg):
        self.pose = msg

    def spin_until_future_done(self, future, timeout=2.0):
        start = time.time()

        while rclpy.ok() and not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.05)

            if time.time() - start > timeout:
                return None

        try:
            return future.result()
        except Exception as e:
            # Ví dụ: spawn goal khi goal đã tồn tại -> turtlesim trả lỗi name exists.
            # Đây không phải lỗi nghiêm trọng, nên chỉ warn và tiếp tục.
            self.node.get_logger().warn(f"Service call failed: {e}")
            return None

    def wait_for_reset_service(self):
        while rclpy.ok() and not self.reset_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(f"Waiting for {self.topic('reset')} service...")

    def wait_for_pose(self, timeout=5.0):
        start = time.time()

        while rclpy.ok() and self.pose is None and time.time() - start < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.05)

        if self.pose is None:
            raise RuntimeError(
                f"Không nhận được {self.topic('turtle1/pose')}. "
                "Hãy kiểm tra turtlesim_node tương ứng đã chạy chưa."
            )

    def call_reset(self):
        req = Empty.Request()
        future = self.reset_cli.call_async(req)
        result = self.spin_until_future_done(future, timeout=2.0)

        if result is None:
            raise RuntimeError(f"Timeout hoặc lỗi khi gọi {self.topic('reset')}")

    def publish_stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def ensure_goal_marker(self):
        """
        Tạo turtle tên 'goal' để làm marker điểm đích.
        Nếu goal đã tồn tại thì service /spawn sẽ báo lỗi, nhưng ta bỏ qua và tiếp tục.
        """

        if not self.show_goal_marker:
            return

        if not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn(f"Không thấy service {self.topic('spawn')}")
            return

        req = Spawn.Request()
        req.x = float(self.goal[0])
        req.y = float(self.goal[1])
        req.theta = 0.0
        req.name = self.goal_name

        future = self.spawn_cli.call_async(req)
        self.spin_until_future_done(future, timeout=1.0)

        # Tắt bút của goal turtle để marker không vẽ đường khi teleport.
        if self.goal_pen_cli.wait_for_service(timeout_sec=1.0):
            pen_req = SetPen.Request()
            pen_req.r = 255
            pen_req.g = 0
            pen_req.b = 0
            pen_req.width = 3
            pen_req.off = 1

            future = self.goal_pen_cli.call_async(pen_req)
            self.spin_until_future_done(future, timeout=1.0)

    def update_goal_marker(self):
        """Di chuyển turtle 'goal' tới vị trí goal hiện tại."""

        if not self.show_goal_marker:
            return

        self.ensure_goal_marker()

        if not self.goal_teleport_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn(
                f"Không thấy service {self.topic(f'{self.goal_name}/teleport_absolute')}"
            )
            return

        req = TeleportAbsolute.Request()
        req.x = float(self.goal[0])
        req.y = float(self.goal[1])
        req.theta = 0.0

        future = self.goal_teleport_cli.call_async(req)
        self.spin_until_future_done(future, timeout=1.0)

        print(
            f"[ENV {self.namespace or 'root'}] "
            f"New goal marker: x={self.goal[0]:.2f}, y={self.goal[1]:.2f}"
        )

    def sample_goal(self):
        # Dùng self.np_random để Gymnasium seed hoạt động đúng.
        self.goal = self.np_random.uniform(
            low=1.0,
            high=10.0,
            size=(2,),
        ).astype(np.float32)

    def distance_to_goal(self):
        dx = float(self.goal[0] - self.pose.x)
        dy = float(self.goal[1] - self.pose.y)
        return math.sqrt(dx * dx + dy * dy)

    def get_obs(self):
        x = float(self.pose.x)
        y = float(self.pose.y)
        theta = float(self.pose.theta)

        dx = float(self.goal[0] - x)
        dy = float(self.goal[1] - y)
        dist = math.sqrt(dx * dx + dy * dy)

        obs = np.array(
            [
                x / self.world_size,
                y / self.world_size,
                math.sin(theta),
                math.cos(theta),
                dx / self.world_size,
                dy / self.world_size,
                dist / self.world_size,
            ],
            dtype=np.float32,
        )

        return obs

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        self.step_count = 0
        self.pose = None

        self.call_reset()
        self.publish_stop()

        self.sample_goal()
        self.update_goal_marker()
        self.wait_for_pose()

        self.prev_dist = self.distance_to_goal()

        obs = self.get_obs()
        info = {
            "goal": self.goal.copy(),
            "distance": self.prev_dist,
            "namespace": self.namespace,
        }

        return obs, info

    def step(self, action):
        self.step_count += 1

        action = np.asarray(action, dtype=np.float32)
        action = np.clip(action, -1.0, 1.0)

        # action[0] trong [-1, 1] -> linear velocity trong [0, max_linear]
        linear = float((action[0] + 1.0) * 0.5 * self.max_linear)
        angular = float(action[1] * self.max_angular)

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

        end_time = time.time() + self.dt
        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.01)

        dist = self.distance_to_goal()
        progress = self.prev_dist - dist

        # Reward shaping: tiến gần goal thì cộng, đi xa thì trừ.
        reward = 10.0 * progress - 0.01

        x = float(self.pose.x)
        y = float(self.pose.y)

        near_wall = x < 0.5 or x > 10.5 or y < 0.5 or y > 10.5
        if near_wall:
            reward -= 1.0

        terminated = dist < self.goal_radius
        truncated = self.step_count >= self.max_steps

        if terminated:
            reward += 30.0
            self.publish_stop()

        if truncated:
            self.publish_stop()

        self.prev_dist = dist

        obs = self.get_obs()
        info = {
            "distance": dist,
            "goal": self.goal.copy(),
            "step": self.step_count,
            "namespace": self.namespace,
        }

        return obs, float(reward), terminated, truncated, info

    def close(self):
        try:
            self.publish_stop()
        except Exception:
            pass

        try:
            self.node.destroy_node()
        except Exception:
            pass
