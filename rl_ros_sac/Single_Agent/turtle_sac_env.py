import math
import time
import numpy as np

import gymnasium as gym # type: ignore
from gymnasium import spaces # type: ignore

import rclpy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, TeleportAbsolute, SetPen
from std_srvs.srv import Empty


class TurtleSacEnv(gym.Env):
    """
    Gymnasium environment bọc quanh ROS 2 turtlesim để train SAC.

    State:
        [x, y, sin(theta), cos(theta), dx, dy, distance]

    Action:
        action[0] -> linear velocity command
        action[1] -> angular velocity command

    Visual marker:
        Một turtle tên 'goal' được spawn/teleport tới vị trí goal để dễ quan sát khi train/test.
    """

    metadata = {"render_modes": []}

    def __init__(self):
        super().__init__()

        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = rclpy.create_node("turtle_sac_env")

        self.cmd_pub = self.node.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        self.pose_sub = self.node.create_subscription(
            Pose,
            "/turtle1/pose",
            self.pose_callback,
            10,
        )

        # Service của turtlesim chính
        self.reset_cli = self.node.create_client(Empty, "/reset")

        # Service dùng để tạo/di chuyển marker goal
        self.goal_name = "goal"
        self.spawn_cli = self.node.create_client(Spawn, "/spawn")
        self.goal_teleport_cli = self.node.create_client(
            TeleportAbsolute,
            f"/{self.goal_name}/teleport_absolute",
        )
        self.goal_pen_cli = self.node.create_client(
            SetPen,
            f"/{self.goal_name}/set_pen",
        )

        self.pose = None
        self.goal = np.array([8.0, 8.0], dtype=np.float32)

        self.world_size = 11.088
        self.max_linear = 2.0
        self.max_angular = 3.0
        self.dt = 0.1

        self.max_steps = 300
        self.step_count = 0

        # Cho goal hơi dễ đạt để quan sát học nhanh hơn
        self.goal_radius = 0.6
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
        while not self.reset_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Waiting for /reset service...")

    def wait_for_pose(self, timeout=3.0):
        start = time.time()

        while self.pose is None and time.time() - start < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.05)

        if self.pose is None:
            raise RuntimeError(
                "Không nhận được /turtle1/pose. Hãy chạy: ros2 run turtlesim turtlesim_node"
            )

    def call_reset(self):
        req = Empty.Request()
        future = self.reset_cli.call_async(req)
        result = self.spin_until_future_done(future, timeout=2.0)

        if result is None:
            raise RuntimeError("Timeout hoặc lỗi khi gọi /reset")

    def publish_stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def ensure_goal_marker(self):
        """
        Tạo turtle tên 'goal' để làm marker điểm đích.
        Nếu goal đã tồn tại thì service /spawn sẽ báo lỗi, nhưng ta bỏ qua và tiếp tục.
        """

        if not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn("Không thấy service /spawn")
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
        """
        Di chuyển turtle 'goal' tới vị trí goal hiện tại.
        """

        self.ensure_goal_marker()

        if not self.goal_teleport_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn("Không thấy service /goal/teleport_absolute")
            return

        req = TeleportAbsolute.Request()
        req.x = float(self.goal[0])
        req.y = float(self.goal[1])
        req.theta = 0.0

        future = self.goal_teleport_cli.call_async(req)
        self.spin_until_future_done(future, timeout=1.0)

        print(f"[ENV] New goal marker: x={self.goal[0]:.2f}, y={self.goal[1]:.2f}")

    def sample_goal(self):
        self.goal = np.random.uniform(
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
        }

        return obs, info

    def step(self, action):
        self.step_count += 1

        action = np.asarray(action, dtype=np.float32)
        action = np.clip(action, -1.0, 1.0)

        linear = float((action[0] + 1.0) * 0.5 * self.max_linear)
        angular = float(action[1] * self.max_angular)

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

        end_time = time.time() + self.dt
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.01)

        dist = self.distance_to_goal()
        progress = self.prev_dist - dist

        # Reward shaping: thưởng khi tiến gần goal, phạt nhẹ theo thời gian.
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
        }

        return obs, float(reward), terminated, truncated, info

    def close(self):
        self.publish_stop()
        self.node.destroy_node()
