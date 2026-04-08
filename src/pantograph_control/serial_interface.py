from __future__ import annotations

from dataclasses import dataclass

import serial


@dataclass(slots=True)
class SerialReturn:
    status: bool
    message: str


class SerialConnection:
    def __init__(self, port: str, baudrate: int, timeout: float = 5.0):
        self.connection = serial.Serial(port, baudrate, timeout=timeout)

    def send_setpoint(self, q1: float, q2: float, z: float) -> SerialReturn:
        command = f"SETPOINT {float(q1):.3f} {float(q2):.3f} {float(z):.3f}\n"
        self.connection.write(command.encode())

        try:
            response = self._read_response_tokens()
            if response == ["OK"]:
                return SerialReturn(True, "OK")
            if response == ["FINISHED"]:
                return SerialReturn(
                    False,
                    "Protocol mismatch: received FINISHED before OK for SETPOINT.",
                )
            return SerialReturn(
                False,
                f"Unexpected SETPOINT acknowledgement: {' '.join(response)}",
            )
        except Exception as exc:
            return SerialReturn(False, str(exc))

    def wait_for_finished(self, timeout: float | None = None) -> SerialReturn:
        try:
            response = self._read_response_tokens(timeout=timeout)
            if response == ["FINISHED"]:
                return SerialReturn(True, "FINISHED")
            if response == ["OK"]:
                return SerialReturn(
                    False,
                    "Protocol mismatch: received OK while waiting for FINISHED.",
                )
            return SerialReturn(
                False,
                f"Unexpected completion response: {' '.join(response)}",
            )
        except Exception as exc:
            return SerialReturn(False, str(exc))

    def send_activate(self) -> SerialReturn:
        return self._send_simple_command("ACTIVATE\n")

    def send_deactivate(self) -> SerialReturn:
        return self._send_simple_command("DEACTIVATE\n")

    def send_gripper(self, command: str) -> SerialReturn:
        if command not in {"OPEN", "CLOSE"}:
            raise ValueError("Gripper command must be OPEN or CLOSE.")
        return self._send_simple_command(f"GRIPPER {command}\n")

    def _send_simple_command(self, command: str) -> SerialReturn:
        self.connection.write(command.encode())

        try:
            response = self._read_response_tokens()
            if response and response[0] == "OK":
                return SerialReturn(True, " ".join(response[1:]).strip())
            return SerialReturn(False, " ".join(response))
        except Exception as exc:
            return SerialReturn(False, str(exc))

    def _read_response_tokens(self, timeout: float | None = None) -> list[str]:
        previous_timeout = self.connection.timeout
        if timeout is not None:
            self.connection.timeout = timeout

        try:
            raw = self.connection.readline()
        finally:
            if timeout is not None:
                self.connection.timeout = previous_timeout

        if not raw:
            raise TimeoutError("Timed out waiting for controller response.")

        decoded = raw.decode(errors="replace").strip()
        if not decoded:
            raise TimeoutError("Received an empty response from the controller.")

        return decoded.split()
