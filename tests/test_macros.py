from dataclasses import fields
from pathlib import Path
import math
import re
import sys
import time

import numpy as np
import pytest

TEST_FILE = Path(__file__).resolve()
REPO_ROOT = TEST_FILE.parents[1]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from pantograph_control import camera
from pantograph_control import macros as macros_module
from pantograph_control.macros import (
    Controller,
    DetectedCluster,
    TaskError,
    TransferConfig,
    Point
)

DEBUG_NOMINAL_CLUSTERS = 3
DEBUG_OUTGOING_CLUSTERS = 3
MANUAL_JOG_DX = 0.01


@pytest.fixture(autouse=True)
def clean_camera_state():
    camera.close_camera()
    yield
    camera.close_camera()


def _make_hardware_controller(
    *,
    nominal_clusters=DEBUG_NOMINAL_CLUSTERS,
    outgoing_clusters=DEBUG_OUTGOING_CLUSTERS,
):
    controller = Controller(
        TransferConfig(
            nominal_clusters=nominal_clusters,
            outgoing_clusters=outgoing_clusters,
        )
    )
    controller.connection = None
    return controller

def _close_controller_connection(controller):
    connection = getattr(controller, "connection", None)
    serial_connection = getattr(connection, "connection", None)
    close = getattr(serial_connection, "close", None)
    if close is not None:
        close()


def _calibrate_or_skip_serial_unavailable(controller):
    try:
        return controller._calibrate_impl()
    except TaskError as exc:
        if str(exc).startswith("Unable to connect to pantograph controller."):
            pytest.skip(str(exc))
        raise


def _camera_frame_shape_or_skip():
    if camera.Picamera2 is None:
        pytest.skip("Picamera2 is unavailable; skipping hardware macro camera test.")

    try:
        frame = camera.capture_frame()
    except Exception as exc:
        pytest.skip(f"Pi camera capture is unavailable: {exc}")
    return frame.shape


def _assert_point_close(actual, expected):
    assert len(actual) == len(expected)
    for actual_value, expected_value in zip(actual, expected):
        assert actual_value == pytest.approx(expected_value)


def _assert_clusters_match_frame(controller, clusters, frame_shape):
    height, width = frame_shape[:2]

    assert len(clusters) >= 0
    assert controller.status.detected_count == len(clusters)
    assert clusters == sorted(clusters, key=lambda cluster: (cluster.point[1], cluster.point[0]))

    center_x, center_y, _ = controller.config.input_center
    for cluster in clusters:
        assert isinstance(cluster, DetectedCluster)
        assert 0 <= cluster.pixel_x < width
        assert 0 <= cluster.pixel_y < height
        assert len(cluster.point) == 3
        assert cluster.point[2] == pytest.approx(controller.config.pick_height)
        assert (
            math.hypot(cluster.point[0] - center_x, cluster.point[1] - center_y)
            <= controller.config.dish_radius_m + 1e-12
        )

def test_calibrate_impl_hardware():
    controller = _make_hardware_controller()

    try:
        result = _calibrate_or_skip_serial_unavailable(controller)

        assert result == "Calibration complete. Pantograph is at the safe home pose."
        assert controller.status.calibrated is True
        _assert_point_close(controller.pose.point, controller.config.safe_home_low)
    finally:
        _close_controller_connection(controller)


def test_capture_clusters_impl_hardware():
    frame_shape = _camera_frame_shape_or_skip()
    controller = _make_hardware_controller()

    result = controller._capture_clusters_impl()

    assert re.fullmatch(r"Detected \d+ candidate clusters in dish A\.", result)
    _assert_clusters_match_frame(controller, controller.clusters, frame_shape)


def test_manual_jog_impl_hardware():
    controller = _make_hardware_controller()
    jogged = False

    try:
        _calibrate_or_skip_serial_unavailable(controller)
        start_point = controller.pose.point
        expected_target = (
            start_point[0] + MANUAL_JOG_DX,
            start_point[1],
            start_point[2],
        )

        result = controller._manual_jog_impl(dx=MANUAL_JOG_DX, dy=0.0, dz=0.0)
        jogged = True

        assert result == (
            f"Moved to x={expected_target[0]:.4f} m, "
            f"y={expected_target[1]:.4f} m, z={expected_target[2]:.4f} m."
        )
        _assert_point_close(controller.pose.point, expected_target)
    finally:
        if jogged:
            controller._manual_jog_impl(dx=-MANUAL_JOG_DX, dy=0.0, dz=0.0)
        _close_controller_connection(controller)


def test_center_input_impl_hardware():
    controller = _make_hardware_controller()

    try:

        _calibrate_or_skip_serial_unavailable(controller)
        controller._set_status(calibrated=True)
        controller._close_gripper()
        controller._move_to(controller.config.input_center)
        
        next_point = Point([
            controller.config.input_center[0],
            controller.config.input_center[1],
            controller.config.pick_height])
        
        _assert_point_close(controller.pose.point, controller.config.input_center)

        controller._move_to(next_point)
        controller._move_to(controller.config.safe_home_high)
    finally:
        _close_controller_connection(controller)


def test_center_output_impl_hardware():
    controller = _make_hardware_controller()

    try:
        _calibrate_or_skip_serial_unavailable(controller)
        controller._close_gripper()
        controller._move_to(controller.config.output_center)
        _assert_point_close(controller.pose.point, controller.config.output_center)

        next_point = Point([
            controller.config.output_center[0],
            controller.config.output_center[1],
            controller.config.output_center[2] + 0.02])
        
        controller._move_to(next_point)
        controller._move_to(controller.config.safe_home_high)
    finally:
        _close_controller_connection(controller)

def test_run_transfer_impl_hardware():
    frame_shape = _camera_frame_shape_or_skip()
    # controller = _make_hardware_controller(
    #     nominal_clusters=DEBUG_NOMINAL_CLUSTERS,
    #     outgoing_clusters=DEBUG_OUTGOING_CLUSTERS,
    # )
    controller = _make_hardware_controller(
        nominal_clusters=9,
        outgoing_clusters=9,
    )

    try:
        if input("Calibrate? >> ") == 'n': 
            controller._set_status(calibrated=True)
        else: controller._calibrate_impl()

        controller._set_status(calibrated=True)

        result = controller._run_transfer_impl()
        
        assert re.fullmatch(
            (
                rf"Transferred \d+ of {DEBUG_NOMINAL_CLUSTERS} planned clusters "
                r"from Dish A to Dish B\. "
                r"Replace the incoming and outgoing dishes before the next run\."
            ),
            result,
        )
        assert 0 <= controller.status.transferred_count <= DEBUG_NOMINAL_CLUSTERS
        _assert_clusters_match_frame(controller, controller.clusters, frame_shape)
        assert len(controller.clusters) >= controller.status.transferred_count
    finally:
        _close_controller_connection(controller)


DEBUG_ALIASES = {
    "_calibrate_impl": "test_calibrate_impl_hardware",
    "calibrate_impl": "test_calibrate_impl_hardware",
    "_capture_clusters_impl": "test_capture_clusters_impl_hardware",
    "capture_clusters_impl": "test_capture_clusters_impl_hardware",
    "_run_transfer_impl": "test_run_transfer_impl_hardware",
    "run_transfer_impl": "test_run_transfer_impl_hardware",
    "_manual_jog_impl": "test_manual_jog_impl_hardware",
    "manual_jog_impl": "test_manual_jog_impl_hardware",
    "_center_input_impl": "test_center_input_impl_hardware",
    "center_input_impl": "test_center_input_impl_hardware",
    "_center_output_impl": "test_center_output_impl_hardware",
    "center_output_impl": "test_center_output_impl_hardware",
}


def _debug_test_node(test_arg):
    if "::" in test_arg or test_arg.endswith(".py"):
        return test_arg

    test_name = DEBUG_ALIASES.get(test_arg, test_arg)
    if not test_name.startswith("test_"):
        test_name = f"test_{test_name.lstrip('_')}"

    return f"{TEST_FILE}::{test_name}"


def main(argv=None):
    raw_args = list(sys.argv[1:] if argv is None else argv)
    test_args = []
    pytest_args = ["--pdb", "-s", "--tb=short"]
    passthrough_value_options = {"-k", "-m"}
    pass_next = False

    for arg in raw_args:
        if pass_next:
            pytest_args.append(arg)
            pass_next = False
        elif arg in passthrough_value_options:
            pytest_args.append(arg)
            pass_next = True
        elif arg.startswith("-"):
            pytest_args.append(arg)
        else:
            test_args.append(_debug_test_node(arg))

    if not test_args:
        test_args.append(str(TEST_FILE))

    return pytest.main(pytest_args + test_args)


if __name__ == "__main__":
    raise SystemExit(main())
