import pantograph_control as control
import numpy as np
import time
wait = lambda: input("Press Enter to continue >> ")

CALLUSES_IN = [[-0.15, 0.20],
            [-0.15, 0.21]]
            # [-0.15, 0.22],
            # [-0.15, 0.23],
            # [-0.15, 0.24],
            # [-0.15, 0.25]]

HOME = [-0.07, 0.25]

UP_HEIGHT = 0.25

GRIP_HEIGHT = 0.0015
LIFT_HEIGHT = 0.02

CALLUSES_OUT = [[p[0] + 0.15, p[1]] for p in CALLUSES_IN]

compute_angles = lambda endpoint : control.compute_joint_angles(
    control.A1, control.A2, control.A3, control.A4, control.A5,
    np.array([0.0, 0.0]), endpoint, np.array([-control.A5, 0.0]))

def move_and_wait(serial_connection, q1, q2, z):
    result = serial_connection.send_setpoint(q1, q2, z)
    print("SETPOINT", result.status, result.message)
    if not result.status:
        raise RuntimeError(result.message)

    finished = serial_connection.wait_for_finished()
    print("FINISHED", finished.status, finished.message)
    if not finished.status:
        raise RuntimeError(finished.message)

try:
    serial_connection = control.SerialConnection("/dev/ttyACM0", 115200)
except:
    serial_connection = control.SerialConnection("/dev/ttyACM1", 115200)

result = serial_connection.send_activate()

print(result.status, result.message)


result = serial_connection.send_gripper("OPEN")
# time.sleep(0.2)
result = serial_connection.send_gripper("CLOSE")
# time.sleep(0.2)
result = serial_connection.send_gripper("OPEN")


angles = compute_angles(np.array(CALLUSES_IN[0]))
move_and_wait(serial_connection, angles[1], angles[0], LIFT_HEIGHT)

angles = compute_angles(np.array(HOME))
move_and_wait(serial_connection, angles[1], angles[0], LIFT_HEIGHT)
result = serial_connection.send_gripper("OPEN")
# time.sleep(1.0)

for index, P in enumerate(CALLUSES_IN):

    # Move above the callus in
    angles = compute_angles(np.array(P))
    move_and_wait(serial_connection, angles[1], angles[0], LIFT_HEIGHT)
    result = serial_connection.send_gripper("OPEN")

    # Grip the callus
    move_and_wait(serial_connection, angles[1], angles[0], GRIP_HEIGHT)
    result = serial_connection.send_gripper("CLOSE")

    # Lift the callus
    move_and_wait(serial_connection, angles[1], angles[0], LIFT_HEIGHT)
    
    # Move to corresponding point out
    angles = compute_angles(np.array(CALLUSES_OUT[index]))
    move_and_wait(serial_connection, angles[1], angles[0], LIFT_HEIGHT)

    # Lower the callus into media
    move_and_wait(serial_connection, angles[1], angles[0], GRIP_HEIGHT)
    result = serial_connection.send_gripper("OPEN")

    # Lift back out 
    move_and_wait(serial_connection, angles[1], angles[0], LIFT_HEIGHT)



angles = compute_angles(np.array(HOME))
move_and_wait(serial_connection, angles[1], angles[0], UP_HEIGHT)
result = serial_connection.send_gripper("OPEN")
