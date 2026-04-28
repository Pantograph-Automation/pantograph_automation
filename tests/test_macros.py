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


def _make_cycle_controller(nominal_clusters, outgoing_clusters):
    return Controller(
        TransferConfig(
            nominal_clusters=nominal_clusters,
            outgoing_clusters=outgoing_clusters,
        )
    )


def _advance_cycle(controller):
    plan = controller._current_transfer_plan()
    before = (controller._source_offset, controller._destination_offset)
    controller._advance_transfer_cycle(plan.planned_count)
    after = (controller._source_offset, controller._destination_offset)
    return before, plan.planned_count, after


def _clusters_from_points(points):
    return [
        DetectedCluster(pixel_x=index, pixel_y=index, point=point)
        for index, point in enumerate(points)
    ]


def _full_cycle_cases(nominal_clusters, outgoing_clusters):
    if (nominal_clusters, outgoing_clusters) == (45, 30):
        return [
            ((0, 0), 30, (30, 0)),
            ((30, 0), 15, (0, 15)),
            ((0, 15), 15, (15, 0)),
            ((15, 0), 30, (0, 0)),
        ]
    if (nominal_clusters, outgoing_clusters) == (30, 20):
        return [
            ((0, 0), 20, (20, 0)),
            ((20, 0), 10, (0, 10)),
            ((0, 10), 10, (10, 0)),
            ((10, 0), 20, (0, 0)),
        ]
    raise ValueError("Expected a 45:30 or 30:20 cycle.")


def test_transfer_cycle_advances_45_to_30():
    controller = _make_cycle_controller(45, 30)

    assert [_advance_cycle(controller) for _ in range(4)] == _full_cycle_cases(45, 30)


def test_transfer_cycle_advances_30_to_20():
    controller = _make_cycle_controller(30, 20)

    assert [_advance_cycle(controller)[1] for _ in range(4)] == [20, 10, 10, 20]


def test_output_offset_survives_input_replacement():
    controller = _make_cycle_controller(45, 30)

    _advance_cycle(controller)
    _advance_cycle(controller)
    next_plan = controller._current_transfer_plan()

    assert controller._source_offset == 0
    assert controller._destination_offset == 15
    assert list(next_plan.destination_slots) == list(range(15, 30))


def test_second_run_selects_currently_detected_clusters():
    controller = _make_cycle_controller(45, 30)
    controller._source_offset = 30
    controller._clusters = _clusters_from_points(controller._build_source_pattern()[:15])

    assert controller._select_clusters_for_transfer(15) == controller.clusters


def test_insufficient_detected_clusters_blocks_transfer_without_advancing(monkeypatch):
    controller = _make_cycle_controller(45, 30)
    controller._source_offset = 30
    controller._set_status(calibrated=True)
    controller._clusters = _clusters_from_points(controller._build_source_pattern()[:14])

    monkeypatch.setattr(controller, "_ensure_connection", lambda: None)
    monkeypatch.setattr(controller, "_pick_lid_from_holder", lambda: None)
    monkeypatch.setattr(controller, "_place_lid_on_input", lambda: None)
    monkeypatch.setattr(controller, "_pick_lid_from_input", lambda: None)
    monkeypatch.setattr(controller, "_place_lid_on_holder", lambda: None)
    monkeypatch.setattr(controller, "_move_to", lambda point: None)
    monkeypatch.setattr(controller, "_capture_clusters_impl", lambda: "Detected 14 candidate clusters in dish A.")

    with pytest.raises(TaskError, match="detected 14 clusters, but 15 are required"):
        controller._run_transfer_impl()

    assert (controller._source_offset, controller._destination_offset) == (30, 0)


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


def _prompt_full_cycle_ratio():
    raw_ratio = input("Run full cycle ratio [45:30 or 30:20] (default 45:30) >> ").strip()
    ratio = raw_ratio or "45:30"
    if ratio == "45:30":
        return 45, 30
    if ratio == "30:20":
        return 30, 20
    raise ValueError(f"Unsupported full cycle ratio: {ratio}")


def test_run_full_cycle_impl_hardware():
    _camera_frame_shape_or_skip()
    nominal_clusters, outgoing_clusters = _prompt_full_cycle_ratio()
    controller = _make_hardware_controller(
        nominal_clusters=nominal_clusters,
        outgoing_clusters=outgoing_clusters,
    )

    try:
        if input("Calibrate? >> ") == 'n':
            controller._set_status(calibrated=True)
        else:
            controller._calibrate_impl()

        controller._set_status(calibrated=True)

        for run_index, (before, planned_count, after) in enumerate(
            _full_cycle_cases(nominal_clusters, outgoing_clusters),
            start=1,
        ):
            assert (controller._source_offset, controller._destination_offset) == before
            print(
                f"\nRun {run_index}/4 for {nominal_clusters}:{outgoing_clusters}: "
                f"source_offset={before[0]}, destination_offset={before[1]}, "
                f"planned={planned_count}"
            )
            input("Confirm dishes are loaded for this run, then press Enter. >> ")

            result = controller._run_transfer_impl()
            print(result)

            assert re.fullmatch(
                (
                    rf"Transferred {planned_count} of {planned_count} planned clusters "
                    r"from Dish A to Dish B\. .+"
                ),
                result,
            )
            assert (controller._source_offset, controller._destination_offset) == after

            if run_index < 4:
                input("Swap dishes as instructed above, then press Enter for the next run. >> ")
    finally:
        _close_controller_connection(controller)


def test_run_transfer_impl_hardware():
    frame_shape = _camera_frame_shape_or_skip()
    controller = _make_hardware_controller(
        nominal_clusters=15,
        outgoing_clusters=15,
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
    "full_cycle": "test_run_full_cycle_impl_hardware",
    "run_full_cycle_impl": "test_run_full_cycle_impl_hardware",
    "test_run_full_cycle_impl": "test_run_full_cycle_impl_hardware",
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
