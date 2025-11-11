from pantograph_control import *




# Set up the right joint
joint1 = pantograph.Joint(
    pantograph.JointConfig(
        ENA_PIN=17,
        PUL_PIN=0,
        DIR_PIN=0,
        STEPS_PER_REV=1600,
        SPEED=60
    )
)


# Set up the left joint
joint2 = pantograph.Joint(
    pantograph.JointConfig(
        ENA_PIN=16,
        PUL_PIN=2,
        DIR_PIN=2,
        STEPS_PER_REV=1600,
        SPEED=60
    )
)


# Set up the gripper
gripper = pantograph.Gripper(
    pantograph.GripperConfig(
        SERVO_PIN=7,
        FREQUENCY=50,
        MIN_DUTY=2,
        MAX_DUTY=12,
        RANGE_OF_MOTION=180
    ),
    angle_at_open=50,
    angle_at_closed=0
)

# Create the pantograph machine
machine = pantograph.Pantograph(
    pantograph.PantographConfig(
        joint1,
        joint2,
        gripper
    )
)

# Full ROM


# Callus transfer simulation

# Dish transfer simulation