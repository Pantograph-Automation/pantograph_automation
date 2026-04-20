import math

A1 = 0.15
A2 = 0.17
A3 = 0.17
A4 = 0.15
A5 = 0.11

BASE_P1 = (0.0, 0.0)
BASE_P5 = (-A5, 0.0)

THETA1_MIN = -0.3
THETA1_MAX = 2 * math.pi / 3
THETA5_MIN = 2 * math.pi / 3
THETA5_MAX = math.pi + 0.3
JOINT_LIMIT_TOL = 1e-6
