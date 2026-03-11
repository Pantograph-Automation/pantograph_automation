import serial

class SerialReturn:
    def __init__(self, status:bool, message:str):
        self.status = status
        self.message = message

class SerialConnection:
    def __init__(self, port:str, baudrate:int, timeout:float=5.0):
        
        self.connection = serial.Serial(port, baudrate, timeout=timeout)

    def send_setpoint(self, q1: float, q2: float, z: float) -> SerialReturn:
        command = f"SETPOINT {float(q1):.3f} {float(q2):.3f} {float(z):.3f}\n"
        self.connection.write(command.encode())

        try:
            response = self.connection.readline().decode().strip().split()
            
            if response[0] == "OK":
                return SerialReturn(True, str(response[1:]))
            else:
                return SerialReturn(False, str(response))
        except Exception as e:
            return SerialReturn(False, str(e))

    def send_activate(self) -> SerialReturn:
        command = f"ACTIVATE\n"
        self.connection.write(command.encode())

        try:
            response = self.connection.readline().decode().strip().split()
            
            if response[0] == "OK":
                return SerialReturn(True, str(response[1:]))
            else:
                return SerialReturn(False, str(response))
        except Exception as e:
            return SerialReturn(False, str(e))
    
    def send_deactivate(self) -> SerialReturn:
        command = f"DEACTIVATE\n"
        self.connection.write(command.encode())

        try:
            response = self.connection.readline().decode().strip().split()
            
            if response[0] == "OK":
                return SerialReturn(True, str(response[1:]))
            else:
                return SerialReturn(False, str(response))
        except Exception as e:
            return SerialReturn(False, str(e))

    def send_gripper(self, command:str) -> SerialReturn:
        assert command in ["OPEN", "CLOSE"]
        command = f"GRIPPER {command}\n"
        self.connection.write(command.encode())

        try:
            response = self.connection.readline().decode().strip().split()
            
            if response[0] == "OK":
                return SerialReturn(True, str(response[1:]))
            else:
                return SerialReturn(False, str(response))
        except Exception as e:
            return SerialReturn(False, str(e))


