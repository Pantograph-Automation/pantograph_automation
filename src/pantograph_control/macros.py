from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field
from typing import Callable

import numpy as np
from PySide6.QtCore import QObject, Signal

from .camera import (
    MASK_RADIUS_RATIO,
    MASK_X_OFFSET,
    MASK_Y_OFFSET,
    capture_frame,
    process_frame,
)
from .kinematics import compute_joint_angles
from .mechanics import (
    A1,
    A2,
    A3,
    A4,
    A5,
    BASE_P1,
    BASE_P5,
    JOINT_LIMIT_TOL,
    THETA1_MAX,
    THETA1_MIN,
    THETA5_MAX,
    THETA5_MIN,
)
from .serial_interface import SerialConnection, SerialReturn

Point = tuple[float, float, float]

class TaskError(RuntimeError):
    """Raised when a prototype task cannot be completed."""


@dataclass(slots=True)
class Pose:
    point: Point
    theta1: float
    theta5: float


@dataclass(slots=True)
class DetectedCluster:
    pixel_x: int
    pixel_y: int
    point: Point



@dataclass(slots=True)
class TransferConfig:
    serial_ports: tuple[str, ...] = ("/dev/ttyACM0", "/dev/ttyACM1", "COM3", "COM4", "COM6")
    baudrate: int = 115200
    finished_timeout: float = 15.0
    safe_home: Point = (-0.07, 0.25, 0.25)
    dish_a_center: Point = (-0.15, 0.20, 0.02)
    dish_b_center: Point = (0.00, 0.20, 0.02)
    dish_c_center: Point = (0.00, 0.25, 0.02)
    dish_radius_m: float = 0.035
    pick_height: float = 0.02
    place_height: float = 0.02
    lift_height: float = 0.02
    pick_limit: int = 45
    manual_step_min_m: float = 0.001
    manual_step_max_m: float = 0.020
    camera_mask_x_offset: int = MASK_X_OFFSET
    camera_mask_y_offset: int = MASK_Y_OFFSET
    camera_mask_radius_ratio: float = MASK_RADIUS_RATIO


@dataclass(slots=True)
class ControllerStatus:
    state: str = "idle"
    detail: str = "Idle"
    calibrated: bool = False
    busy: bool = False
    last_error: str = ""
    detected_count: int = 0
    transferred_count: int = 0
    active_source: str = "Dish A"
    active_destination: str = "Dish B"
    destination_positions: list[Point] = field(default_factory=list)


class Controller(QObject):
    status_changed = Signal(object)
    pose_changed = Signal(object)
    clusters_changed = Signal(object)
    log_message = Signal(str)

    def __init__(self, config: TransferConfig | None = None):
        super().__init__()
        self.config = config or TransferConfig()
        self.connection: SerialConnection
        self.status = ControllerStatus(
            destination_positions=self._build_destination_pattern()
        )
        home_theta1, home_theta5 = self._compute_angles(self.config.safe_home)
        self.pose = Pose(
            point=self.config.safe_home,
            theta1=home_theta1,
            theta5=home_theta5,
        )
        self._clusters: list[DetectedCluster] = []
        self._action_lock = threading.Lock()

    @property
    def clusters(self) -> list[DetectedCluster]:
        return list(self._clusters)

    def emit_initial_state(self) -> None:
        self.status_changed.emit(self.status)
        self.pose_changed.emit(self.pose)
        self.clusters_changed.emit(self.clusters)

    def calibrate_async(self) -> bool:
        return self._start_action("calibrating", "Calibrating pantograph...", self._calibrate_impl)

    def run_transfer_async(self) -> bool:
        return self._start_action("running", "Running transfer from dish A to dish B...", self._run_transfer_impl)

    def capture_cluster_positions_async(self) -> bool:
        return self._start_action(
            "capturing",
            "Capturing source clusters from dish A...",
            self._capture_clusters_impl,
        )

    def manual_jog_async(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> bool:
        detail = f"Manual jog dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f}"
        return self._start_action(
            "manual",
            detail,
            lambda: self._manual_jog_impl(dx=dx, dy=dy, dz=dz),
        )

    def slider_value_to_step(self, slider_value: int, slider_maximum: int) -> float:
        if slider_maximum <= 0:
            return self.config.manual_step_min_m
        span = self.config.manual_step_max_m - self.config.manual_step_min_m
        ratio = slider_value / slider_maximum
        return self.config.manual_step_min_m + span * ratio

    def _start_action(
        self,
        state: str,
        detail: str,
        action: Callable[[], str | None],
    ) -> bool:
        if not self._action_lock.acquire(blocking=False):
            self.log_message.emit("Controller busy; ignoring new action request.")
            return False

        self._set_status(state=state, detail=detail, busy=True, last_error="")

        def runner() -> None:
            final_detail = detail
            final_state = "idle"
            final_error = ""
            try:
                result = action()
                if state == "manual":
                    final_detail = result or "Manual move complete."
                    final_state = "calibrated" if self.status.calibrated else "idle"
                elif state == "capturing":
                    final_detail = result or "Cluster capture complete."
                    final_state = "calibrated" if self.status.calibrated else "idle"
                elif state == "calibrating":
                    final_detail = result or "Calibration complete."
                    final_state = "calibrated"
                else:
                    final_detail = result or "Run complete."
                    final_state = "complete"
            except Exception as exc:
                final_error = str(exc)
                final_state = "error"
                final_detail = str(exc)
                self.log_message.emit(final_error)
            finally:
                self._set_status(
                    state=final_state,
                    detail=final_detail,
                    busy=False,
                    last_error=final_error,
                )
                self._action_lock.release()

        thread = threading.Thread(target=runner, daemon=True)
        thread.start()
        return True

    def _set_status(self, **updates) -> None:
        for key, value in updates.items():
            setattr(self.status, key, value)
        self.status_changed.emit(self.status)

    def _set_pose(self, pose: Pose) -> None:
        self.pose = pose
        self.pose_changed.emit(self.pose)

    def _set_clusters(self, clusters: list[DetectedCluster]) -> None:
        self._clusters = clusters
        self.status.detected_count = len(clusters)
        self.clusters_changed.emit(self.clusters)
        self.status_changed.emit(self.status)

    def _ensure_connection(self) -> None:
        if self.connection is not None:
            return

        errors: list[str] = []
        for port in self.config.serial_ports:
            try:
                self.connection = SerialConnection(port, self.config.baudrate)
                self.log_message.emit(f"Connected on {port}")
                return
            except Exception as exc:
                errors.append(f"{port}: {exc}")

        joined = "; ".join(errors) if errors else "No configured serial ports."
        raise TaskError(f"Unable to connect to pantograph controller. {joined}")

    def _calibrate_impl(self) -> str:
        self._ensure_connection()
        self._send_command(
            lambda: self.connection.send_activate(timeout=self.config.finished_timeout),
            "Failed to activate pantograph.",
        )
        self._send_command(lambda: self.connection.send_gripper("OPEN"), "Failed to open gripper during calibration.")
        self._move_to(self.config.safe_home)
        self._set_status(calibrated=True)
        return "Calibration complete. Pantograph is at the safe home pose."

    def _capture_clusters_impl(self) -> str:
        frame = capture_frame()
        centroids = process_frame(
            frame,
            self.config.camera_mask_x_offset,
            self.config.camera_mask_y_offset,
            self.config.camera_mask_radius_ratio,
        )
        clusters = self._centroids_to_clusters(frame.shape, centroids)
        self._set_clusters(clusters)
        return f"Detected {len(clusters)} candidate clusters in dish A."

    def _run_transfer_impl(self) -> str:
        if not self.status.calibrated:
            raise TaskError("Run blocked: calibrate before starting a transfer.")

        self._ensure_connection()
        self._set_status(transferred_count=0)
        self._capture_clusters_impl()

        if len(self._clusters) < self.config.pick_limit:
            raise TaskError(
                f"Detected only {len(self._clusters)} clusters in dish A; need {self.config.pick_limit} for the prototype run."
            )

        source_clusters = self._clusters[: self.config.pick_limit]
        destinations = self.status.destination_positions[: self.config.pick_limit]

        self._move_to(point=self.config.safe_home)
        self._send_command(lambda: self.connection.send_gripper("OPEN"), "Failed to open gripper before transfer run.")

        for index, (cluster, destination) in enumerate(zip(source_clusters, destinations), start=1):
            self._set_status(
                detail=f"Transferring cluster {index}/{self.config.pick_limit}",
                transferred_count=index - 1,
            )
            self._pick_cluster(cluster)
            self._place_cluster(destination)
            self._set_status(transferred_count=index)

        self._move_to(self.config.safe_home)
        self._send_command(lambda: self.connection.send_gripper("OPEN"), "Failed to open gripper after transfer run.")
        return f"Transferred {self.config.pick_limit} clusters from dish A to dish B."

    def _manual_jog_impl(self, dx: float, dy: float, dz: float) -> str:
        if not self.status.calibrated:
            raise TaskError("Manual control is disabled until calibration succeeds.")

        target_x = self.pose.point[0] + dx
        target_y = self.pose.point[1] + dy
        target_z = max(0.0, self.pose.point[2] + dz)
        self._move_to(point=Point([target_x, target_y, target_z]))
        return f"Moved to x={target_x:.4f} m, y={target_y:.4f} m, z={target_z:.4f} m."

    def _pick_cluster(self, cluster: DetectedCluster) -> None:
        self._move_to(cluster.point)
        self._send_command(lambda: self.connection.send_gripper("OPEN"), "Failed to open gripper before pick.")
        self._move_to(cluster.point)
        self._send_command(lambda: self.connection.send_gripper("CLOSE"), "Failed to close gripper on source cluster.")
        self._move_to(cluster.point)

    def _place_cluster(self, place_destination: Point) -> None:
        lift_point = Point([place_destination[0], place_destination[1], self.config.lift_height])
        self._move_to(lift_point)
        self._move_to(place_destination)
        self._send_command(lambda: self.connection.send_gripper("OPEN"), "Failed to open gripper at destination.")
        self._move_to(lift_point)

    def _move_to(self, point: Point) -> None:
        self._ensure_connection()
        theta1, theta5 = self._compute_angles(point=point)
        self._validate_joint_limits(point, theta1, theta5)
        result = self.connection.send_setpoint(
            theta5,
            theta1,
            point[2],
            timeout=self.config.finished_timeout,
        )
        self._require_ok(result, f"Failed to move to x={point[0]:.4f}, y={point[1]:.4f}, z={point[2]:.4f}.")
        self._set_pose(Pose(point=point, theta1=theta1, theta5=theta5))

    def _compute_angles(self, point: Point) -> tuple[float, float]:
        return compute_joint_angles(
            A1,
            A2,
            A3,
            A4,
            A5,
            np.array(BASE_P1, dtype=np.float64),
            np.array([point[0], point[1]]),
            np.array(BASE_P5, dtype=np.float64),
        )

    def _validate_joint_limits(self, point: Point, theta1: float, theta5: float) -> None:
        violations: list[str] = []

        if theta1 < THETA1_MIN - JOINT_LIMIT_TOL or theta1 > THETA1_MAX + JOINT_LIMIT_TOL:
            violations.append(
                f"theta1={theta1:.4f} rad outside [{THETA1_MIN:.4f}, {THETA1_MAX:.4f}]"
            )
        if theta5 < THETA5_MIN - JOINT_LIMIT_TOL or theta5 > THETA5_MAX + JOINT_LIMIT_TOL:
            violations.append(
                f"theta5={theta5:.4f} rad outside [{THETA5_MIN:.4f}, {THETA5_MAX:.4f}]"
            )

        if violations:
            raise TaskError(
                f"Target x={point[0]:.4f}, y={point[1]:.4f} violates mechanical joint limits: "
                + "; ".join(violations)
            )

    def _send_command(
        self,
        command: Callable[[], SerialReturn],
        error_prefix: str,
    ) -> None:
        result = command()
        self._require_ok(result, error_prefix)

    def _require_ok(self, result: SerialReturn, error_prefix: str) -> None:
        if not result.status:
            raise TaskError(f"{error_prefix} Serial response: {result.message}")

    def _build_destination_pattern(self) -> list[Point]:
        center_x, center_y, center_z = self.config.dish_b_center
        pattern: list[Point] = []
        ring_specs = (
            (self.config.dish_radius_m * 0.18, 6),
            (self.config.dish_radius_m * 0.42, 10),
            (self.config.dish_radius_m * 0.66, 14),
        )

        for radius, count in ring_specs:
            for index in range(count):
                angle = (2 * math.pi * index) / count
                x = center_x + radius * math.cos(angle)
                y = center_y + radius * math.sin(angle)
                pattern.append((x, y, center_z))
                if len(pattern) == self.config.pick_limit:
                    return pattern

        if len(pattern) < self.config.pick_limit:
            raise TaskError(f"Unable to generate {self.config.pick_limit} destination positions inside dish B.")
        return pattern

    def _centroids_to_clusters(
        self,
        frame_shape: tuple[int, int, int],
        centroids: list[tuple[int, int]],
    ) -> list[DetectedCluster]:
        height, width = frame_shape[:2]
        mask_center_x = width / 2 + self.config.camera_mask_x_offset
        mask_center_y = height / 2 + self.config.camera_mask_y_offset
        radius_px = min(height, width) * self.config.camera_mask_radius_ratio
        radius_m = self.config.dish_radius_m
        center_x_m, center_y_m, _ = self.config.dish_a_center
        clusters: list[DetectedCluster] = []

        for px, py in centroids:
            offset_x = (px - mask_center_x) / radius_px
            offset_y = (py - mask_center_y) / radius_px
            x_m = center_x_m + offset_x * radius_m
            y_m = center_y_m + offset_y * radius_m
            if self._point_inside_dish(x_m, y_m, self.config.dish_a_center, radius_m):
                clusters.append(DetectedCluster(pixel_x=px, pixel_y=py, point=Point([x_m, y_m, self.config.pick_height])))

        clusters.sort(key=lambda cluster: (cluster.point[1], cluster.point[0]))
        return clusters

    @staticmethod
    def _point_inside_dish(x: float, y: float, center: Point, radius: float) -> bool:
        return math.hypot(x - center[0], y - center[1]) <= radius
