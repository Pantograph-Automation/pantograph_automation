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
try:
    serial_connection = control.SerialConnection("/dev/ttyACM0", 115200)
except:
    serial_connection = control.SerialConnection("/dev/ttyACM1", 115200)

result = serial_connection.send_activate()

print(result.status, result.message)


result = serial_connection.send_gripper("OPEN")
time.sleep(0.5)
result = serial_connection.send_gripper("CLOSE")
time.sleep(0.5)
result = serial_connection.send_gripper("OPEN")
time.sleep(0.5)
result = serial_connection.send_gripper("CLOSE")
print(result.status, result.message)
time.sleep(0.5)

angles = compute_angles(np.array(CALLUSES_IN[0]))
serial_connection.send_setpoint(angles[1], angles[0], LIFT_HEIGHT)
wait()

angles = compute_angles(np.array(HOME))
serial_connection.send_setpoint(angles[1], angles[0], LIFT_HEIGHT)
result = serial_connection.send_gripper("OPEN")
time.sleep(1.0)

for index, P in enumerate(CALLUSES_IN):

    # Move above the callus in
    angles = compute_angles(np.array(P))
    serial_connection.send_setpoint(angles[1], angles[0], LIFT_HEIGHT)
    result = serial_connection.send_gripper("OPEN")
    time.sleep(5.0)

    # Grip the callus
    serial_connection.send_setpoint(angles[1], angles[0], GRIP_HEIGHT)
    time.sleep(1.0)
    result = serial_connection.send_gripper("CLOSE")
    time.sleep(0.5)

    # Lift the callus
    serial_connection.send_setpoint(angles[1], angles[0], LIFT_HEIGHT)
    time.sleep(0.5)
    
    # Move to corresponding point out
    angles = compute_angles(np.array(CALLUSES_OUT[index]))
    serial_connection.send_setpoint(angles[1], angles[0], LIFT_HEIGHT)
    time.sleep(5.0)

    # Lower the callus into media
    serial_connection.send_setpoint(angles[1], angles[0], GRIP_HEIGHT)
    time.sleep(1.0)
    result = serial_connection.send_gripper("OPEN")
    time.sleep(0.5)

    # Lift back out 
    serial_connection.send_setpoint(angles[1], angles[0], LIFT_HEIGHT)
    time.sleep(0.5)


angles = compute_angles(np.array(HOME))
serial_connection.send_setpoint(angles[1], angles[0], UP_HEIGHT)
result = serial_connection.send_gripper("OPEN")
time.sleep(1.0)