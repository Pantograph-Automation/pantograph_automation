import serial

class SerialReturn:
    def __init__(self, status:bool, message:str):
        self.status = status
        self.message = message

class SerialConnection:
    def __init__(self, port:str, baudrate:int, timeout:float=5.0):
        
        self.connection = serial.Serial(port, baudrate, timeout=timeout)

    def send_setpoint(self, x: float, y: float) -> SerialReturn:
        # Construct and send the command
        command = f"SETPOINT {x} {y}\n"
        self.connection.write(command.encode())

        # Wait until message received back (reading a line)
        try:
            # decode() converts bytes to string; strip() removes \r\n
            response = self.connection.readline().decode().strip().split()
            
            if response[0] == "OK":
                return SerialReturn(True, str(response[1:]))
            else:
                return SerialReturn(False, str(response))
        except Exception as e:
            # Handle potential serial timeouts or decoding errors
            return SerialReturn(False, str(e))

    def send_transition(self, transition: str) -> SerialReturn:
        # Construct and send the transition command
        command = f"TRANSITION {transition}\n"
        self.connection.write(command.encode())

        try:
            response = self.connection.readline().decode().strip().split()
            
            if response[0] == "OK":

                return SerialReturn(True, str(response[1:]))
            else:
                return SerialReturn(False, str(response))
        except Exception as e:
            return SerialReturn(False, str(e))


