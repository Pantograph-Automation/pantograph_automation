from __future__ import annotations

from dataclasses import dataclass

import serial


SUCCESS_RESPONSES = {"ACTIVE", "COMPLETE"}
ERROR_RESPONSES = {"INVALID_TRANSITION", "INVALID_SETPOINT", "INVALID_SERIAL"}
LEGACY_RESPONSES = {"OK", "FINISHED"}


@dataclass(slots=True)
class SerialReturn:
    status: bool
    message: str


class SerialConnection:
    def __init__(self, port: str, baudrate: int, timeout: float = 5.0):
        self.connection = serial.Serial(port, baudrate, timeout=timeout)

    def send_setpoint(
        self,
        q1: float,
        q2: float,
        z: float,
        timeout: float | None = None,
    ) -> SerialReturn:
        command = f"SETPOINT {float(q1):.4f} {float(q2):.4f} {float(z):.4f}\n"
        return self._send_command(
            command,
            expected_status="COMPLETE",
            timeout=timeout,
            context="SETPOINT",
        )

    def wait_for_finished(self, timeout: float | None = None) -> SerialReturn:
        try:
            response = self._read_response_line(timeout=timeout)
            return self._parse_response(
                response,
                expected_status="COMPLETE",
                context="completion",
            )
        except Exception as exc:
            return SerialReturn(False, str(exc))

    def send_activate(self, timeout: float | None = None) -> SerialReturn:
        return self._send_command(
            "ACTIVATE\n",
            expected_status="COMPLETE",
            timeout=timeout,
            context="ACTIVATE",
        )

    def send_deactivate(self, timeout: float | None = None) -> SerialReturn:
        return self._send_command(
            "DEACTIVATE\n",
            expected_status="COMPLETE",
            timeout=timeout,
            context="DEACTIVATE",
        )

    def send_gripper(
        self,
        command: str,
        timeout: float | None = None,
    ) -> SerialReturn:
        if command not in {"OPEN", "CLOSE"}:
            raise ValueError("Gripper command must be OPEN or CLOSE.")
        return self._send_command(
            f"GRIPPER {command}\n",
            expected_status="ACTIVE",
            timeout=timeout,
            context=f"GRIPPER {command}",
        )

    def _send_command(
        self,
        command: str,
        expected_status: str,
        timeout: float | None,
        context: str,
    ) -> SerialReturn:
        self.connection.write(command.encode())

        try:
            response = self._read_response_line(timeout=timeout)
            return self._parse_response(response, expected_status, context)
        except Exception as exc:
            return SerialReturn(False, str(exc))

    def _parse_response(
        self,
        response: str,
        expected_status: str,
        context: str,
    ) -> SerialReturn:
        if response == expected_status:
            return SerialReturn(True, response)
        if response in ERROR_RESPONSES:
            return SerialReturn(False, response)
        if response in LEGACY_RESPONSES:
            return SerialReturn(
                False,
                f"Protocol mismatch: received legacy response {response} for {context}; "
                f"expected {expected_status}.",
            )
        if response in SUCCESS_RESPONSES:
            return SerialReturn(
                False,
                f"Protocol mismatch: received {response} for {context}; "
                f"expected {expected_status}.",
            )
        return SerialReturn(False, f"Unexpected {context} response: {response}")

    def _read_response_line(self, timeout: float | None = None) -> str:
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

        return decoded
