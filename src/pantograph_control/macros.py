from __future__ import annotations

import math, time
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

DISH_PATTERN_RING_SPECS = (
    (0.18, 3),
    (0.45, 10),
    (0.70, 15),
    (0.88, 17),
)
A5_AXIS_CENTER_XY = (-0.055, 0.0)


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
    calibrate_timeout: float = 30.0
    safe_home_high: Point = (-0.07, 0.20, 0.23)
    safe_home_low: Point = (-0.07, 0.20, 0.10)
    lid_holder: Point = (0.004, 0.080, 0.0)
    lid_place: Point = (-0.116, 0.104, 0.0151)
    output_center: Point = (-0.069, 0.210, 0.023)
    input_center: Point = (-0.1580, 0.1560, 0.022)
    fisheye_scale: tuple[float, ...] = (1.05, 1.08)
    fisheye_threshold: tuple[float, ...] = (0.023, 0.031)
    a5_axis_scale: float = 1.04
    axis_scale_threshold: float = 0.030
    dish_radius_m: float = 0.038
    pick_height: float = 0.022
    place_height: float = 0.0235
    lift_height: float = 0.038
    nominal_clusters: int = 45
    outgoing_clusters: int = 30
    manual_step_min_m: float = 0.001
    manual_step_max_m: float = 0.020
    camera_mask_x_offset: int = MASK_X_OFFSET
    camera_mask_y_offset: int = MASK_Y_OFFSET
    camera_mask_radius_ratio: float = MASK_RADIUS_RATIO

    def __post_init__(self) -> None:
        if self.nominal_clusters <= 0:
            raise ValueError("nominal_clusters must be greater than zero.")
        if self.outgoing_clusters <= 0:
            raise ValueError("outgoing_clusters must be greater than zero.")


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
        self.connection: SerialConnection | None = None
        self.status = ControllerStatus(
            destination_positions=self._build_destination_pattern()
        )
        home_theta1, home_theta5 = self._compute_angles(self.config.safe_home_low)
        self.pose = Pose(
            point=self.config.safe_home_low,
            theta1=home_theta1,
            theta5=home_theta5,
        )
        self._clusters: list[DetectedCluster] = []
        self._source_offset = 0
        self._destination_offset = 0
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
            lambda: self.connection.send_activate(timeout=self.config.calibrate_timeout),
            "Failed to activate pantograph.",
        )
        self._move_to(self.config.safe_home_low)
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
        self.log_message.emit(
            f"Camera detected {len(centroids)} centroids; retained {len(clusters)} clusters after dish filtering."
        )
        self._set_clusters(clusters)
        return f"Detected {len(clusters)} candidate clusters in dish A."

    def _run_transfer_impl(self) -> str:
        if not self.status.calibrated:
            raise TaskError("Run blocked: calibrate before starting a transfer.")

        self._ensure_connection()
        planned_count = self._planned_transfer_count()
        source_start = self._source_offset
        destination_start = self._destination_offset
        source_stop = source_start + planned_count

        output_midpoint = Point([
            self.config.output_center[0],
            self.config.output_center[1],
            self.config.output_center[2] + 0.05])

        self._pick_lid_from_holder()
        self._move_to(output_midpoint)
        self._place_lid_on_input()

        self._move_to(self.config.safe_home_low)
        
        self._set_status(transferred_count=0)
        self._capture_clusters_impl()

        self._pick_lid_from_input()
        self._move_to(output_midpoint)
        self._place_lid_on_holder()
        self._move_to(output_midpoint)

        clusters_by_source_slot = self._match_clusters_to_source_slots(self._clusters)
        destinations = self.status.destination_positions

        transferred_count = 0
        for batch_index, source_slot_index in enumerate(range(source_start, source_stop), start=1):
            destination_index = destination_start + batch_index - 1
            destination = destinations[destination_index]
            cluster = clusters_by_source_slot.get(source_slot_index)
            if cluster is None:
                self._set_status(
                    detail=f"Skipping missing cluster {batch_index}/{planned_count}",
                    transferred_count=transferred_count,
                )
                continue

            self._set_status(
                detail=f"Transferring cluster {batch_index}/{planned_count}",
                transferred_count=transferred_count,
            )
            self._pick_cluster(cluster)
            self._place_cluster(destination)
            transferred_count += 1
            self._set_status(transferred_count=transferred_count)

        self._move_to(self.config.safe_home_low)
        self._send_command(lambda: self.connection.send_gripper("OPEN"), "Failed to open gripper after transfer run.")
        next_step = self._advance_transfer_cycle(planned_count)
        return (
            f"Transferred {transferred_count} of {planned_count} planned clusters from Dish A to Dish B. "
            f"{next_step}"
        )
    
    def _close_gripper(self) -> str:
        self._send_command(lambda: self.connection.send_gripper("CLOSE"), "Failed to open gripper after transfer run.")
        time.sleep(0.2)
        return "Closed gripper."
    
    def _open_gripper(self) -> str:
        self._send_command(lambda: self.connection.send_gripper("OPEN"), "Failed to open gripper after transfer run.")
        time.sleep(0.2)
        return "Opened gripper."
    

    def _manual_jog_impl(self, dx: float, dy: float, dz: float) -> str:
        if not self.status.calibrated:
            raise TaskError("Manual control is disabled until calibration succeeds.")

        target_x = self.pose.point[0] + dx
        target_y = self.pose.point[1] + dy
        target_z = max(0.0, self.pose.point[2] + dz)
        self._move_to(point=Point([target_x, target_y, target_z]))
        return f"Moved to x={target_x:.4f} m, y={target_y:.4f} m, z={target_z:.4f} m."

    def _pick_cluster(self, cluster: DetectedCluster) -> None:
        lift_point = Point([cluster.point[0], cluster.point[1], self.config.lift_height])
        self._move_to(lift_point)
        self._open_gripper()
        self._move_to(cluster.point)
        self._close_gripper()
        self._move_to(lift_point)

    def _place_cluster(self, place_destination: Point) -> None:
        lift_point = Point([place_destination[0], place_destination[1], self.config.lift_height])
        self._move_to(lift_point)
        self._move_to(place_destination)
        self._open_gripper()
        self._move_to(lift_point)

    def _pick_lid_from_holder(self):
        self._open_gripper()
        lift_point = Point([
            self.config.lid_holder[0],
            self.config.lid_holder[1],
            self.config.lid_holder[2] + 0.030])
        self._move_to(lift_point)
        self._move_to(self.config.lid_holder)
        self._close_gripper()
        self._move_to(lift_point)

    def _pick_lid_from_input(self):
        lift_point = Point([
            self.config.lid_place[0],
            self.config.lid_place[1],
            self.config.lid_place[2] + 0.030])
        self._move_to(lift_point)
        self._move_to(self.config.lid_place)
        self._close_gripper()
        self._move_to(lift_point)
    
    def _place_lid_on_input(self):
        lift_point = Point([
            self.config.lid_place[0],
            self.config.lid_place[1],
            self.config.lid_place[2] + 0.030])
        self._move_to(lift_point)
        self._move_to(self.config.lid_place)
        self._open_gripper()
        self._move_to(lift_point)

    def _place_lid_on_holder(self):
        lift_point = Point([
            self.config.lid_holder[0],
            self.config.lid_holder[1],
            self.config.lid_holder[2] + 0.030])
        self._move_to(lift_point)
        drop_point = Point([
            self.config.lid_holder[0],
            self.config.lid_holder[1] + 0.001,
            self.config.lid_holder[2] + 0.010
        ])
        self._move_to(drop_point)
        self._open_gripper()
        self._move_to(lift_point)
        

    def _move_to(self, point: Point) -> None:
        self._ensure_connection()
        theta1, theta5 = self._compute_angles(point=point)
        self._validate_joint_limits(point, theta1, theta5)
        result = self.connection.send_setpoint(
            theta1,
            theta5,
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

    def _planned_transfer_count(self) -> int:
        source_remaining = self.config.nominal_clusters - self._source_offset
        destination_remaining = self.config.outgoing_clusters - self._destination_offset
        planned_count = min(source_remaining, destination_remaining)
        if planned_count <= 0:
            raise TaskError("Transfer cycle state is invalid; no source or destination slots are available.")
        return planned_count

    def _advance_transfer_cycle(self, planned_count: int) -> str:
        self._source_offset += planned_count
        self._destination_offset += planned_count

        replace_source = self._source_offset >= self.config.nominal_clusters
        replace_destination = self._destination_offset >= self.config.outgoing_clusters

        if replace_source:
            self._source_offset = 0
        if replace_destination:
            self._destination_offset = 0

        if replace_source and replace_destination:
            return "Replace the incoming and outgoing dishes before the next run."
        if replace_source:
            return "Replace the incoming dish before the next run."
        if replace_destination:
            return "Replace the outgoing dish before the next run."
        return "Continue with the current dishes for the next run."

    def _match_clusters_to_source_slots(
        self,
        clusters: list[DetectedCluster],
    ) -> dict[int, DetectedCluster]:
        source_positions = self._build_source_pattern()
        distances: list[tuple[float, int, int]] = []
        for cluster_index, cluster in enumerate(clusters):
            for source_index, source_position in enumerate(source_positions):
                distance = math.hypot(
                    cluster.point[0] - source_position[0],
                    cluster.point[1] - source_position[1],
                )
                distances.append((distance, source_index, cluster_index))

        matched_clusters: set[int] = set()
        matched_slots: dict[int, DetectedCluster] = {}
        for _, source_index, cluster_index in sorted(distances):
            if source_index in matched_slots or cluster_index in matched_clusters:
                continue
            matched_slots[source_index] = clusters[cluster_index]
            matched_clusters.add(cluster_index)
            if len(matched_clusters) == min(len(clusters), len(source_positions)):
                break

        return matched_slots

    def _build_source_pattern(self) -> list[Point]:
        center_x, center_y, _ = self.config.input_center
        center = (center_x, center_y, self.config.pick_height)
        return self._build_dish_pattern(
            center=center,
            count=self.config.nominal_clusters,
            label="source",
        )

    def _build_destination_pattern(self) -> list[Point]:
        return self._build_dish_pattern(
            center=self.config.output_center,
            count=self.config.outgoing_clusters,
            label="destination",
        )

    def _build_dish_pattern(self, center: Point, count: int, label: str) -> list[Point]:
        center_x, center_y, center_z = center
        pattern: list[Point] = []

        for radius_ratio, ring_count in DISH_PATTERN_RING_SPECS:
            radius = self.config.dish_radius_m * radius_ratio
            for index in range(ring_count):
                angle = (2 * math.pi * index) / ring_count
                x = center_x + (radius * math.cos(angle))
                y = center_y + (radius * math.sin(angle))
                pattern.append((x, y, center_z))
                if len(pattern) == count:
                    return pattern

        raise TaskError(f"Unable to generate {count} {label} positions inside the dish.")

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
        center_x_m, center_y_m, _ = self.config.input_center
        clusters: list[DetectedCluster] = []

        for px, py in centroids:
            offset_x = (px - mask_center_x) / radius_px
            offset_y = (py - mask_center_y) / radius_px

            local_radius = math.hypot(offset_x * radius_m, offset_y * radius_m)
            
            # HACK: Fisheye correction
            if local_radius >= self.config.fisheye_threshold[1]:
                fi_scale = self.config.fisheye_scale[1]
            elif local_radius >= self.config.fisheye_threshold[0]:
                fi_scale = self.config.fisheye_scale[0]
            else:
                fi_scale = 1.0

            # HACK: Localized correction
            if (offset_x <= -0.005 and abs(offset_y) <= 0.025):
                offset_x = offset_x - 0.0005
                offset_y = offset_y + 0.0015
            
            x_m = center_x_m + offset_x * radius_m * fi_scale
            y_m = center_y_m + offset_y * radius_m * fi_scale
            if local_radius >= self.config.axis_scale_threshold:
                x_m, y_m = self._scale_point_along_a5_axis(x_m, y_m)
            if self._point_inside_dish(x_m, y_m, self.config.input_center, radius_m):
                clusters.append(DetectedCluster(pixel_x=px, pixel_y=py, point=Point([x_m, y_m, self.config.pick_height])))

        clusters.sort(key=lambda cluster: (cluster.point[1], cluster.point[0]))
        return clusters

    def _scale_point_along_a5_axis(self, x: float, y: float) -> tuple[float, float]:
        center_x, center_y, _ = self.config.input_center
        axis_x = A5_AXIS_CENTER_XY[0] - center_x
        axis_y = A5_AXIS_CENTER_XY[1] - center_y
        axis_length = math.hypot(axis_x, axis_y)
        if axis_length == 0:
            return x, y

        unit_x = axis_x / axis_length
        unit_y = axis_y / axis_length
        vector_x = x - center_x
        vector_y = y - center_y
        projection = vector_x * unit_x + vector_y * unit_y
        scale_delta = projection * (self.config.a5_axis_scale - 1.0)
        return x + scale_delta * unit_x, y + scale_delta * unit_y

    @staticmethod
    def _point_inside_dish(x: float, y: float, center: Point, radius: float) -> bool:
        return math.hypot(x - center[0], y - center[1]) <= radius * 1.30
