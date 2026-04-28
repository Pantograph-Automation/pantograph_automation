from pathlib import Path
import math
import sys

import numpy as np
import pytest

TEST_FILE = Path(__file__).resolve()
REPO_ROOT = TEST_FILE.parents[1]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from pantograph_control import (
    A1,
    A2,
    A3,
    A4,
    A5,
    BASE_P1,
    BASE_P5,
    compute_end_point,
    compute_joint_angles,
)


def _base_points():
    return (
        np.array(BASE_P1, dtype=np.float64),
        np.array(BASE_P5, dtype=np.float64),
    )


def test_compute_joint_angles_runs():
    P1, P5 = _base_points()

    print(compute_joint_angles(
        A1,
        A2,
        A3,
        A4,
        A5,
        P1,
        np.array([-0.07, 0.25], dtype=np.float64),
        P5,
    ))


def test_compute_end_point_runs():
    P1, P5 = _base_points()

    J1 = 1.49 # math.pi / 3
    J2 = 3.03 # 2 * math.pi / 3

    print(compute_end_point(
        A1,
        A2,
        A3,
        A4,
        A5,
        P1,
        P5,
        J1,
        J2
    ))


DEBUG_ALIASES = {
    "compute_joint_angles": "test_compute_joint_angles_runs",
    "joint_angles": "test_compute_joint_angles_runs",
    "compute_end_point": "test_compute_end_point_runs",
    "end_point": "test_compute_end_point_runs",
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
