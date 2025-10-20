#
# This file will contain the raspberry pi <-> hardware interfaces for the stepper motors,
# servo motors, and camera(s)
#
# The stepper motor driver will need to be position controlled. This means that we will need
# to have a calibration mechanism built in to the driver that can be accessed through the
# 'Calibrate' task
# 
# https://github.com/gavinlyonsrepo/RpiMotorLib

import sys
import time

# TODO: Stepper motor interface
class Stepper:

    def __init__(self):
        pass

# TODO: Servo driver interface
class Servo:

    def __init__(self):
        pass