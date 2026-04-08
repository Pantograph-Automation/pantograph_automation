from __future__ import annotations

from typing import Tuple

import numpy as np
from numpy.typing import NDArray


FRAME_RTOL = 1e-7
FRAME_ATOL = 1e-9
GEOMETRY_ATOL = 1e-9


def compute_joint_angles(
    a1: float,
    a2: float,
    a3: float,
    a4: float,
    a5: float,
    P1: NDArray[np.float64],
    P3: NDArray[np.float64],
    P5: NDArray[np.float64],
) -> Tuple[float, float]:
    """
    Compute joint angles from the end-effector position using the Pantograph Mk-II
    inverse-kinematics equations.

    The public signature remains generic in P1 and P5, but points are normalized
    into the paper's frame before applying Eq. (8)-(12), where P1 is at the origin
    and P5 lies at (-a5, 0).
    """

    P1_arr = _as_point(P1)
    P3_arr = _as_point(P3)
    P5_arr = _as_point(P5)

    _validate_ground_link(a5, P1_arr, P5_arr)
    _, P3_local, _ = _normalize_to_paper_frame(P1_arr, P3_arr, P5_arr)

    try:
        return _compute_local_joint_angles(a1, a2, a3, a4, a5, P3_local)
    except ValueError as exc:
        raise OutOfBoundsException(P3_arr, str(exc)) from exc


def compute_end_point(
    a1: float,
    a2: float,
    a3: float,
    a4: float,
    a5: float,
    P1: NDArray[np.float64],
    P5: NDArray[np.float64],
    theta1: float,
    theta5: float,
) -> NDArray[np.float64]:
    """
    Compute the end-effector point from the joint configuration.

    This implements the direct kinematics described in Eq. (1)-(7) of the paper,
    and returns the useful solution with the larger local y value.
    """

    P1_arr = _as_point(P1)
    P5_arr = _as_point(P5)

    _validate_ground_link(a5, P1_arr, P5_arr)
    local_point = _compute_local_end_point(a1, a2, a3, a4, a5, theta1, theta5)
    return _denormalize_from_paper_frame(P1_arr, P5_arr, local_point)


def _compute_local_joint_angles(
    a1: float,
    a2: float,
    a3: float,
    a4: float,
    a5: float,
    P3_local: NDArray[np.float64],
) -> tuple[float, float]:
    P1_local = np.array([0.0, 0.0], dtype=np.float64)
    P5_local = np.array([-a5, 0.0], dtype=np.float64)

    d13 = _dist(P1_local, P3_local)
    d53 = _dist(P5_local, P3_local)

    _validate_triangle(a1, a2, d13, "P1-P3 triangle")
    _validate_triangle(a4, a3, d53, "P5-P3 triangle")

    alpha1 = _safe_acos((a1**2 - a2**2 + d13**2) / (2 * a1 * d13))
    beta1 = np.arctan2(P3_local[1], -P3_local[0])
    beta5 = _safe_acos((a4**2 - a3**2 + d53**2) / (2 * a4 * d53))
    alpha5 = np.arctan2(P3_local[1], P3_local[0] + a5)

    theta1 = np.pi - alpha1 - beta1
    theta5 = alpha5 + beta5
    return float(theta1), float(theta5)


def _compute_local_end_point(
    a1: float,
    a2: float,
    a3: float,
    a4: float,
    a5: float,
    theta1: float,
    theta5: float,
) -> NDArray[np.float64]:
    P2 = np.array([a1 * np.cos(theta1), a1 * np.sin(theta1)], dtype=np.float64)
    P4 = np.array([a4 * np.cos(theta5) - a5, a4 * np.sin(theta5)], dtype=np.float64)

    d = _dist(P2, P4)
    _validate_triangle(a2, a3, d, "P2-P4 triangle")

    base_projection = (a2**2 - a3**2 + d**2) / (2 * d)
    Ph = P2 + (base_projection / d) * (P4 - P2)

    height_sq = a2**2 - base_projection**2
    if height_sq < -GEOMETRY_ATOL:
        raise OutOfBoundsException(message="Direct kinematics produced a negative height.")
    height = np.sqrt(max(height_sq, 0.0))

    dx = P4[0] - P2[0]
    dy = P4[1] - P2[1]
    perpendicular = np.array([dy, -dx], dtype=np.float64) / d

    candidate_a = Ph + height * perpendicular
    candidate_b = Ph - height * perpendicular

    if candidate_a[1] >= candidate_b[1]:
        return candidate_a
    return candidate_b


def _normalize_to_paper_frame(
    P1: NDArray[np.float64],
    P3: NDArray[np.float64],
    P5: NDArray[np.float64],
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    x_axis, y_axis = _frame_axes(P1, P5)

    def transform(point: NDArray[np.float64]) -> NDArray[np.float64]:
        relative = point - P1
        return np.array(
            [
                np.dot(relative, x_axis),
                np.dot(relative, y_axis),
            ],
            dtype=np.float64,
        )

    return transform(P1), transform(P3), transform(P5)


def _denormalize_from_paper_frame(
    P1: NDArray[np.float64],
    P5: NDArray[np.float64],
    point_local: NDArray[np.float64],
) -> NDArray[np.float64]:
    x_axis, y_axis = _frame_axes(P1, P5)
    return P1 + point_local[0] * x_axis + point_local[1] * y_axis


def _frame_axes(
    P1: NDArray[np.float64],
    P5: NDArray[np.float64],
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    base_vector = P1 - P5
    base_length = np.linalg.norm(base_vector)
    if base_length <= GEOMETRY_ATOL:
        raise ValueError("P1 and P5 must not coincide.")

    x_axis = base_vector / base_length
    y_axis = np.array([-x_axis[1], x_axis[0]], dtype=np.float64)
    return x_axis, y_axis


def _validate_ground_link(a5: float, P1: NDArray[np.float64], P5: NDArray[np.float64]) -> None:
    base_distance = _dist(P1, P5)
    if not np.isclose(base_distance, a5, rtol=FRAME_RTOL, atol=FRAME_ATOL):
        raise ValueError(
            f"P1: {P1} and P5: {P5} must be separated by a5={a5}, got {base_distance}."
        )


def _validate_triangle(link_a: float, link_b: float, link_c: float, label: str) -> None:
    if link_c <= GEOMETRY_ATOL:
        raise ValueError(f"{label} is degenerate.")

    lower = abs(link_a - link_b) - GEOMETRY_ATOL
    upper = (link_a + link_b) + GEOMETRY_ATOL
    if not (lower <= link_c <= upper):
        raise ValueError(f"{label} is unreachable for length {link_c}.")


def _safe_acos(value: float) -> float:
    clipped = float(np.clip(value, -1.0, 1.0))
    return float(np.arccos(clipped))


def _as_point(point: NDArray[np.float64]) -> NDArray[np.float64]:
    array = np.asarray(point, dtype=np.float64)
    if array.shape != (2,):
        raise ValueError(f"Expected a 2D point, got shape {array.shape}.")
    return array


def _dist(P1: NDArray[np.float64], P2: NDArray[np.float64]) -> float:
    return float(np.linalg.norm(P1 - P2))


class OutOfBoundsException(Exception):
    """Raised when a point lies outside the valid pantograph workspace."""

    def __init__(self, point=None, message="Point is out of bounds"):
        if point is not None:
            message = f"{message}: {point}"
        super().__init__(message)
        self.point = point
