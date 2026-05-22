"""Small actuator-command dispatch demo."""

from dataclasses import dataclass


@dataclass
class Command:
    actuator: str
    value: float


def send_command(command: Command) -> dict:
    return {
        "actuator": command.actuator,
        "value": command.value,
        "status": "accepted",
    }


def main() -> None:
    commands = [
        Command("left_motor", 0.4),
        Command("right_motor", 0.4),
        Command("release_gate", 1.0),
    ]
    for command in commands:
        print(send_command(command))


if __name__ == "__main__":
    main()
