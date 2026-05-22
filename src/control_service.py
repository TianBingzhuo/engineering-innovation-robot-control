# SPDX-License-Identifier: Apache-2.0
"""Public-safe control-service sketch for a Raspberry Pi rescue robot."""

from __future__ import annotations

import time
from dataclasses import dataclass


@dataclass
class RobotState:
    left_pwm: float = 0.0
    right_pwm: float = 0.0
    servo_deg: float = 90.0
    fan_enabled: bool = False
    release_gate_open: bool = False
    last_command_time: float = 0.0


class ControlService:
    def __init__(self, timeout_s: float = 0.35) -> None:
        self.timeout_s = timeout_s
        self.state = RobotState(last_command_time=time.monotonic())

    def apply_command(self, command: str, value: float = 1.0) -> RobotState:
        now = time.monotonic()
        self.state.last_command_time = now
        if command == "forward":
            self.state.left_pwm = self._clamp(value)
            self.state.right_pwm = self._clamp(value)
        elif command == "turn_left":
            self.state.left_pwm = -0.35 * self._clamp(value)
            self.state.right_pwm = self._clamp(value)
        elif command == "turn_right":
            self.state.left_pwm = self._clamp(value)
            self.state.right_pwm = -0.35 * self._clamp(value)
        elif command == "servo":
            self.state.servo_deg = max(0.0, min(180.0, value))
        elif command == "fan":
            self.state.fan_enabled = bool(value)
        elif command == "release":
            self.state.release_gate_open = bool(value)
        elif command == "stop":
            self._stop_drive()
        else:
            raise ValueError(f"unknown command: {command}")
        return self.state

    def tick(self) -> RobotState:
        if time.monotonic() - self.state.last_command_time > self.timeout_s:
            self._stop_drive()
        return self.state

    def _stop_drive(self) -> None:
        self.state.left_pwm = 0.0
        self.state.right_pwm = 0.0

    @staticmethod
    def _clamp(value: float) -> float:
        return max(-1.0, min(1.0, value))


def main() -> int:
    service = ControlService()
    for command, value in [
        ("forward", 0.6),
        ("turn_left", 0.5),
        ("servo", 120.0),
        ("fan", 1.0),
        ("release", 1.0),
        ("stop", 0.0),
    ]:
        state = service.apply_command(command, value)
        print(f"{command:10s} left={state.left_pwm:+.2f} right={state.right_pwm:+.2f} servo={state.servo_deg:5.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
