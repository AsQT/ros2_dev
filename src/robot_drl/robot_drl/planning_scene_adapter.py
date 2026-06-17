"""PlanningScene obstacle extraction and Cartesian path validation helpers."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, TransformStamped
from shape_msgs.msg import SolidPrimitive


class PlanningSceneObstacleError(RuntimeError):
    """Raised when a PlanningScene obstacle cannot be represented safely."""


@dataclass(frozen=True)
class SceneObstacle:
    """Obstacle represented as an AABB in the DRL/base frame."""

    object_id: str
    source_frame: str
    center_base: np.ndarray
    full_size: np.ndarray
    primitive_type: str = "box"
    pose_quat_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    raw_dimensions: tuple[float, ...] = field(default_factory=tuple)

    @property
    def half_extent_base(self) -> np.ndarray:
        return np.asarray(self.full_size, dtype=np.float32) / 2.0


@dataclass(frozen=True)
class PathValidationResult:
    """Result of point/AABB validation on a Cartesian TCP path."""

    valid: bool
    message: str
    min_clearance: float
    checked_samples: int
    obstacle_id: str = ""
    segment_index: int = -1
    sample_index: int = -1


def _normalize_frame_id(frame_id: str) -> str:
    frame = (frame_id or "").strip()
    return frame[1:] if frame.startswith("/") else frame


def _quat_xyzw_to_matrix(q_xyzw: Iterable[float]) -> np.ndarray:
    x, y, z, w = [float(v) for v in q_xyzw]
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return np.eye(3, dtype=np.float64)
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def _matrix_to_quat_xyzw(rot: np.ndarray) -> tuple[float, float, float, float]:
    m = np.asarray(rot, dtype=np.float64)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    else:
        idx = int(np.argmax(np.diag(m)))
        if idx == 0:
            s = math.sqrt(max(1.0 + m[0, 0] - m[1, 1] - m[2, 2], 0.0)) * 2.0
            x = 0.25 * s
            y = (m[0, 1] + m[1, 0]) / s
            z = (m[0, 2] + m[2, 0]) / s
            w = (m[2, 1] - m[1, 2]) / s
        elif idx == 1:
            s = math.sqrt(max(1.0 + m[1, 1] - m[0, 0] - m[2, 2], 0.0)) * 2.0
            x = (m[0, 1] + m[1, 0]) / s
            y = 0.25 * s
            z = (m[1, 2] + m[2, 1]) / s
            w = (m[0, 2] - m[2, 0]) / s
        else:
            s = math.sqrt(max(1.0 + m[2, 2] - m[0, 0] - m[1, 1], 0.0)) * 2.0
            x = (m[0, 2] + m[2, 0]) / s
            y = (m[1, 2] + m[2, 1]) / s
            z = 0.25 * s
            w = (m[1, 0] - m[0, 1]) / s
    quat = np.array([x, y, z, w], dtype=np.float64)
    norm = float(np.linalg.norm(quat))
    if norm < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    quat /= norm
    return tuple(float(v) for v in quat)


def _pose_to_matrix(pose: Pose) -> np.ndarray:
    q = pose.orientation
    rot = _quat_xyzw_to_matrix((q.x, q.y, q.z, q.w))
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = rot
    mat[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return mat


def _transform_to_matrix(transform: TransformStamped) -> np.ndarray:
    t = transform.transform.translation
    q = transform.transform.rotation
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = _quat_xyzw_to_matrix((q.x, q.y, q.z, q.w))
    mat[:3, 3] = [t.x, t.y, t.z]
    return mat


def _target_from_source_matrix(
    tf_buffer,
    target_frame: str,
    source_frame: str,
    timeout_sec: float,
    fixed_transforms: Optional[dict[tuple[str, str], np.ndarray]] = None,
) -> np.ndarray:
    target = _normalize_frame_id(target_frame)
    source = _normalize_frame_id(source_frame)
    if not source:
        raise PlanningSceneObstacleError("Collision object has empty frame_id")
    if source == target:
        return np.eye(4, dtype=np.float64)
    if fixed_transforms:
        direct = fixed_transforms.get((target, source))
        if direct is not None:
            return direct.copy()
        inverse = fixed_transforms.get((source, target))
        if inverse is not None:
            return np.linalg.inv(inverse)
    transform = tf_buffer.lookup_transform(
        target,
        source,
        rclpy.time.Time(),
        timeout=rclpy.duration.Duration(seconds=float(timeout_sec)),
    )
    return _transform_to_matrix(transform)


def _primitive_half_extent(primitive: SolidPrimitive) -> tuple[str, np.ndarray]:
    dims = [float(v) for v in primitive.dimensions]
    if primitive.type == SolidPrimitive.BOX:
        if len(dims) < 3:
            raise PlanningSceneObstacleError("BOX primitive has fewer than 3 dimensions")
        return "box", np.array(
            [
                dims[SolidPrimitive.BOX_X],
                dims[SolidPrimitive.BOX_Y],
                dims[SolidPrimitive.BOX_Z],
            ],
            dtype=np.float64,
        ) / 2.0
    if primitive.type == SolidPrimitive.SPHERE:
        if len(dims) < 1:
            raise PlanningSceneObstacleError("SPHERE primitive has no radius")
        radius = dims[SolidPrimitive.SPHERE_RADIUS]
        return "sphere", np.array([radius, radius, radius], dtype=np.float64)
    if primitive.type == SolidPrimitive.CYLINDER:
        if len(dims) < 2:
            raise PlanningSceneObstacleError("CYLINDER primitive has fewer than 2 dimensions")
        height = dims[SolidPrimitive.CYLINDER_HEIGHT]
        radius = dims[SolidPrimitive.CYLINDER_RADIUS]
        return "cylinder", np.array([radius, radius, height / 2.0], dtype=np.float64)
    if primitive.type == SolidPrimitive.CONE:
        if len(dims) < 2:
            raise PlanningSceneObstacleError("CONE primitive has fewer than 2 dimensions")
        height = dims[SolidPrimitive.CONE_HEIGHT]
        radius = dims[SolidPrimitive.CONE_RADIUS]
        return "cone", np.array([radius, radius, height / 2.0], dtype=np.float64)
    raise PlanningSceneObstacleError(f"Unsupported SolidPrimitive type={primitive.type}")


def _aabb_from_oriented_box(
    center: np.ndarray,
    rotation: np.ndarray,
    local_half_extent: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    half = np.abs(rotation) @ local_half_extent
    return center.astype(np.float32), (2.0 * half).astype(np.float32)


def collision_object_to_obstacles(
    collision_object,
    tf_buffer,
    target_frame: str = "base_link",
    timeout_sec: float = 1.0,
    fixed_transforms: Optional[dict[tuple[str, str], np.ndarray]] = None,
) -> list[SceneObstacle]:
    """Convert one MoveIt CollisionObject into one or more DRL obstacles.

    MoveIt primitive dimensions are full extents for boxes.  The policy was
    trained on obstacle center plus half extents, so this function preserves
    center and converts extents only after building a target-frame AABB.
    """
    source_frame = _normalize_frame_id(collision_object.header.frame_id)
    target_from_source = _target_from_source_matrix(
        tf_buffer, target_frame, source_frame, timeout_sec, fixed_transforms
    )
    object_pose = _pose_to_matrix(collision_object.pose)
    object_id = collision_object.id or "collision_object"
    obstacles: list[SceneObstacle] = []

    for idx, primitive in enumerate(collision_object.primitives):
        primitive_pose = (
            collision_object.primitive_poses[idx]
            if idx < len(collision_object.primitive_poses)
            else Pose()
        )
        primitive_type, local_half_extent = _primitive_half_extent(primitive)
        target_from_primitive = target_from_source @ object_pose @ _pose_to_matrix(primitive_pose)
        center, full_size = _aabb_from_oriented_box(
            target_from_primitive[:3, 3],
            target_from_primitive[:3, :3],
            local_half_extent,
        )
        obstacles.append(
            SceneObstacle(
                object_id=f"{object_id}:{idx}",
                source_frame=source_frame,
                center_base=center,
                full_size=full_size,
                primitive_type=primitive_type,
                pose_quat_xyzw=_matrix_to_quat_xyzw(target_from_primitive[:3, :3]),
                raw_dimensions=tuple(float(v) for v in primitive.dimensions),
            )
        )

    for idx, mesh in enumerate(collision_object.meshes):
        if not mesh.vertices:
            continue
        mesh_pose = (
            collision_object.mesh_poses[idx]
            if idx < len(collision_object.mesh_poses)
            else Pose()
        )
        target_from_mesh = target_from_source @ object_pose @ _pose_to_matrix(mesh_pose)
        pts = []
        for vertex in mesh.vertices:
            local = np.array([vertex.x, vertex.y, vertex.z, 1.0], dtype=np.float64)
            pts.append((target_from_mesh @ local)[:3])
        vertices = np.asarray(pts, dtype=np.float64)
        mins = vertices.min(axis=0)
        maxs = vertices.max(axis=0)
        obstacles.append(
            SceneObstacle(
                object_id=f"{object_id}:mesh{idx}",
                source_frame=source_frame,
                center_base=((mins + maxs) / 2.0).astype(np.float32),
                full_size=(maxs - mins).astype(np.float32),
                primitive_type="mesh_aabb",
                pose_quat_xyzw=_matrix_to_quat_xyzw(target_from_mesh[:3, :3]),
            )
        )

    return obstacles


def planning_scene_to_obstacles(
    planning_scene,
    tf_buffer,
    target_frame: str = "base_link",
    timeout_sec: float = 1.0,
) -> list[SceneObstacle]:
    """Extract world collision objects from a MoveIt PlanningScene."""
    fixed_transforms: dict[tuple[str, str], np.ndarray] = {}
    for transform in planning_scene.fixed_frame_transforms:
        parent = _normalize_frame_id(transform.header.frame_id)
        child = _normalize_frame_id(transform.child_frame_id)
        if parent and child:
            fixed_transforms[(parent, child)] = _transform_to_matrix(transform)

    obstacles: list[SceneObstacle] = []
    for collision_object in planning_scene.world.collision_objects:
        obstacles.extend(
            collision_object_to_obstacles(
                collision_object,
                tf_buffer=tf_buffer,
                target_frame=target_frame,
                timeout_sec=timeout_sec,
                fixed_transforms=fixed_transforms,
            )
        )
    return obstacles


def manual_obstacle(
    center_base: np.ndarray,
    full_size: np.ndarray,
    object_id: str = "input_obstacle",
) -> SceneObstacle:
    """Build a SceneObstacle from manual or vision input."""
    center = np.asarray(center_base, dtype=np.float32)
    size = np.asarray(full_size, dtype=np.float32)
    if center.shape != (3,) or size.shape != (3,):
        raise ValueError(
            f"manual obstacle must be 3D center/size, got center={center.shape}, size={size.shape}"
        )
    return SceneObstacle(
        object_id=object_id,
        source_frame="base_link",
        center_base=center,
        full_size=size,
        primitive_type="box",
        raw_dimensions=tuple(float(v) for v in size),
    )


def _point_segment_distance(point: np.ndarray, start: np.ndarray, end: np.ndarray) -> float:
    seg = end - start
    denom = float(np.dot(seg, seg))
    if denom <= 1e-12:
        return float(np.linalg.norm(point - start))
    t = float(np.clip(np.dot(point - start, seg) / denom, 0.0, 1.0))
    closest = start + t * seg
    return float(np.linalg.norm(point - closest))


def select_obstacle_for_policy(
    obstacles: list[SceneObstacle],
    start_base: np.ndarray,
    target_base: np.ndarray,
) -> Optional[SceneObstacle]:
    """Choose the single obstacle the 15D policy can consume."""
    if not obstacles:
        return None
    start = np.asarray(start_base, dtype=np.float32)
    target = np.asarray(target_base, dtype=np.float32)
    return min(
        obstacles,
        key=lambda obs: _point_segment_distance(obs.center_base, start, target)
        - float(np.linalg.norm(obs.half_extent_base)),
    )


def point_aabb_signed_distance(point: np.ndarray, obstacle: SceneObstacle) -> float:
    """Signed distance from a point to an obstacle AABB.

    Positive means outside, zero on the surface, negative inside.
    """
    p = np.asarray(point, dtype=np.float32)
    center = np.asarray(obstacle.center_base, dtype=np.float32)
    half = np.asarray(obstacle.half_extent_base, dtype=np.float32)
    delta = np.abs(p - center) - half
    outside = np.maximum(delta, 0.0)
    outside_dist = float(np.linalg.norm(outside))
    if outside_dist > 0.0:
        return outside_dist
    return float(np.max(delta))


def iter_cartesian_path_samples(
    trajectory_base: list[np.ndarray],
    max_step_m: float,
) -> Iterable[tuple[int, int, np.ndarray]]:
    """Yield dense samples for each segment of a Cartesian path."""
    if max_step_m <= 0.0:
        raise ValueError("max_step_m must be > 0")
    points = [np.asarray(pt, dtype=np.float32) for pt in trajectory_base]
    if not points:
        return
    yield 0, 0, points[0].copy()
    sample_index = 1
    for seg_idx in range(len(points) - 1):
        start = points[seg_idx]
        end = points[seg_idx + 1]
        dist = float(np.linalg.norm(end - start))
        n_steps = max(1, int(math.ceil(dist / max_step_m)))
        for j in range(1, n_steps + 1):
            alpha = j / n_steps
            sample = ((1.0 - alpha) * start + alpha * end).astype(np.float32)
            yield seg_idx, sample_index, sample
            sample_index += 1


def validate_cartesian_path_against_obstacles(
    trajectory_base: list[np.ndarray],
    obstacles: list[SceneObstacle],
    max_step_m: float = 0.02,
    margin_m: float = 0.0,
) -> PathValidationResult:
    """Validate a Cartesian TCP path against obstacle AABBs."""
    if not trajectory_base:
        return PathValidationResult(False, "Trajectory is empty.", math.inf, 0)
    if not obstacles:
        return PathValidationResult(True, "No obstacles to validate.", math.inf, 0)

    min_clearance = math.inf
    checked = 0
    for seg_idx, sample_idx, sample in iter_cartesian_path_samples(
        trajectory_base, max_step_m
    ):
        checked += 1
        for obstacle in obstacles:
            clearance = point_aabb_signed_distance(sample, obstacle) - margin_m
            if clearance < min_clearance:
                min_clearance = clearance
            if clearance < 0.0:
                return PathValidationResult(
                    valid=False,
                    message=(
                        "TCP path intersects obstacle "
                        f"'{obstacle.object_id}' at segment={seg_idx}, "
                        f"sample={sample_idx}, clearance={clearance:.5f} m"
                    ),
                    min_clearance=min_clearance,
                    checked_samples=checked,
                    obstacle_id=obstacle.object_id,
                    segment_index=seg_idx,
                    sample_index=sample_idx,
                )

    return PathValidationResult(
        valid=True,
        message=(
            f"TCP path is outside {len(obstacles)} obstacle AABB(s); "
            f"min_clearance={min_clearance:.5f} m"
        ),
        min_clearance=min_clearance,
        checked_samples=checked,
    )
