from pathlib import Path
import sys

import cv2
import numpy as np
import pytest

TEST_FILE = Path(__file__).resolve()
REPO_ROOT = TEST_FILE.parents[1]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from pantograph_control import camera

TESTS_DIR = TEST_FILE.parent
TEST_FRAME = TESTS_DIR / "test_frame.png"

@pytest.fixture(autouse=True)
def clean_camera_state(monkeypatch):
    camera.close_camera()
    monkeypatch.chdir(TESTS_DIR)
    yield
    camera.close_camera()


class FakePicamera2:
    instances = []
    frames = []

    def __init__(self):
        self.configurations = []
        self.capture_streams = []
        self.start_count = 0
        self.stop_count = 0
        self.close_count = 0
        self._capture_index = 0
        type(self).instances.append(self)

    def create_still_configuration(self):
        return {"configuration": "still"}

    def configure(self, configuration):
        self.configurations.append(configuration)

    def start(self):
        self.start_count += 1

    def stop(self):
        self.stop_count += 1

    def close(self):
        self.close_count += 1

    def capture_array(self, stream):
        self.capture_streams.append(stream)
        frame = type(self).frames[self._capture_index]
        self._capture_index += 1
        return frame.copy()


@pytest.fixture
def fake_picamera(monkeypatch):
    FakePicamera2.instances = []
    FakePicamera2.frames = [
        np.array([[[10, 20, 30], [40, 50, 60]]], dtype=np.uint8),
        np.array([[[70, 80, 90], [100, 110, 120]]], dtype=np.uint8),
    ]
    monkeypatch.setattr(camera, "Picamera2", FakePicamera2)
    return FakePicamera2


def _load_test_frame():
    if not TEST_FRAME.exists():
        pytest.fail(f"Missing camera test fixture: {TEST_FRAME}")

    frame = cv2.imread(str(TEST_FRAME), cv2.IMREAD_COLOR)
    if frame is None:
        pytest.fail(f"Unable to read camera test fixture as an image: {TEST_FRAME}")
    return frame


def _save_test_image(function_name, image):
    output_path = TESTS_DIR / f"{function_name}_img.png"
    assert cv2.imwrite(str(output_path), image), f"Failed to write {output_path}"
    assert output_path.exists(), f"Missing saved test image: {output_path}"
    assert output_path.stat().st_size > 0, f"Saved test image is empty: {output_path}"
    return output_path


def _assert_color_image(image, expected_shape):
    assert isinstance(image, np.ndarray)
    assert image.dtype == np.uint8
    assert image.ndim == 3
    assert image.shape == expected_shape
    assert image.shape[2] == 3


def _assert_mask_image(image, expected_shape):
    assert isinstance(image, np.ndarray)
    assert image.dtype == np.uint8
    assert image.ndim == 2
    assert image.shape == expected_shape[:2]


def _assert_centroids(centroids, frame_shape):
    height, width = frame_shape[:2]

    assert isinstance(centroids, list)
    for centroid in centroids:
        assert isinstance(centroid, tuple)
        assert len(centroid) == 2
        x, y = centroid
        assert isinstance(x, (int, np.integer))
        assert isinstance(y, (int, np.integer))
        assert 0 <= x < width
        assert 0 <= y < height


def _mask_pipeline(frame):
    masked = camera._apply_circular_mask(
        frame,
        camera.MASK_X_OFFSET,
        camera.MASK_Y_OFFSET,
        camera.MASK_RADIUS_RATIO,
    )
    raw_mask = camera._isolate_spots(masked)
    return camera._clean_noise_morphology(raw_mask)


def _draw_centroids(frame, centroids):
    visual = frame.copy()
    for x, y in centroids:
        cv2.circle(visual, (x, y), 50, (0, 255, 0), 2)
    return visual


def test_apply_circular_mask():
    frame = _load_test_frame()

    masked = camera._apply_circular_mask(
        frame,
        camera.MASK_X_OFFSET,
        camera.MASK_Y_OFFSET,
        camera.MASK_RADIUS_RATIO,
    )

    _assert_color_image(masked, frame.shape)
    _save_test_image("_apply_circular_mask", masked)


def test_isolate_spots():
    frame = _load_test_frame()
    masked = camera._apply_circular_mask(
        frame,
        camera.MASK_X_OFFSET,
        camera.MASK_Y_OFFSET,
        camera.MASK_RADIUS_RATIO,
    )

    color_mask = camera._isolate_spots(masked)

    _assert_mask_image(color_mask, frame.shape)
    _save_test_image("_isolate_spots", color_mask)


def test_clean_noise_morphology():
    frame = _load_test_frame()
    masked = camera._apply_circular_mask(
        frame,
        camera.MASK_X_OFFSET,
        camera.MASK_Y_OFFSET,
        camera.MASK_RADIUS_RATIO,
    )
    color_mask = camera._isolate_spots(masked)

    cleaned_mask = camera._clean_noise_morphology(color_mask)

    _assert_mask_image(cleaned_mask, frame.shape)
    _save_test_image("_clean_noise_morphology", cleaned_mask)


def test_extract_centroids():
    frame = _load_test_frame()
    cleaned_mask = _mask_pipeline(frame)

    centroids = camera._extract_centroids(frame, cleaned_mask)

    _assert_centroids(centroids, frame.shape)
    assert len(centroids) == 39
    _save_test_image("_extract_centroids", _draw_centroids(frame, centroids))


def test_process_frame():
    frame = _load_test_frame()

    centroids = camera.process_frame(
        frame,
        camera.MASK_X_OFFSET,
        camera.MASK_Y_OFFSET,
        camera.MASK_RADIUS_RATIO,
    )

    _assert_centroids(centroids, frame.shape)
    assert len(centroids) == 39
    _save_test_image("process_frame", _draw_centroids(frame, centroids))

def test_process_real_frame():
    frame = camera.capture_frame()

    centroids = camera.process_frame(
        frame,
        camera.MASK_X_OFFSET,
        camera.MASK_Y_OFFSET,
        camera.MASK_RADIUS_RATIO,
    )

    _assert_centroids(centroids, frame.shape)
    _save_test_image("process_frame", _draw_centroids(frame, centroids))


def test_capture_frame():
    if camera.Picamera2 is None:
        pytest.skip("Picamera2 is unavailable; skipping real camera capture test.")

    try:
        frame = camera.capture_frame()
    except Exception as exc:
        pytest.skip(f"Pi camera capture is unavailable: {exc}")

    _assert_color_image(frame, frame.shape)
    _save_test_image("capture_frame", frame)


def _debug_test_node(test_arg):
    if "::" in test_arg or test_arg.endswith(".py"):
        return test_arg

    if test_arg.startswith("test_"):
        test_name = test_arg
    else:
        test_name = f"test_{test_arg.lstrip('_')}"

    return f"{TEST_FILE}::{test_name}"


def main(argv=None):
    raw_args = list(sys.argv[1:] if argv is None else argv)
    test_args = []
    pytest_args = ["--pdb", "-s", "--tb=short"]

    for arg in raw_args:
        if arg.startswith("-"):
            pytest_args.append(arg)
        else:
            test_args.append(_debug_test_node(arg))

    if not test_args:
        test_args.append(str(TEST_FILE))

    return pytest.main(pytest_args + test_args)


if __name__ == "__main__":
    raise SystemExit(main())
