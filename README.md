# Engineering Innovation Robot Control

Clean public archive for an intelligent rescue / engineering-innovation robot control project. The repository contains a Raspberry Pi control script, public-safe control-service demos, and CAD part files for the mechanical mechanism.

## One-Sentence Summary

A Raspberry Pi-based rescue robot can be described as a control stack connecting camera processing, color or target recognition, GPIO/I2C hardware access, actuator commands, mechanism design, and debugging loops.

## Public Technical Framing

```text
camera input
  -> OpenCV / image processing
  -> target decision
  -> main control loop
  -> GPIO / I2C / actuator commands
  -> motors, servos, fan, release gate
  -> logs and debugging
```

## Current Evidence Boundary

- The project is related to the Intelligent Rescue event in the "Intelligence+" track of the Hebei Provincial College Student Engineering Practice and Innovation Ability Competition.
- The repository keeps a cleaned public snapshot of the robot control script and CAD parts.
- Raw reports, registration forms, team information, and private photos/videos are not published here.
- Public pages focus on system architecture, self-written control logic, and mechanism artifacts.

## Public Source Code

- `src/robot_control.py`: Raspberry Pi robot control script from the competition archive.
- `src/control_service.py`: a public-safe control-service sketch with command normalization and fail-safe structure.
- `src/rpc_client_demo.py`: a synthetic RPC-style control demo.
- `hardware/cad/`: SolidWorks part files for mechanism components.

Run:

```bash
python src/rpc_client_demo.py
python src/control_service.py
```

`src/robot_control.py` targets Raspberry Pi hardware and is not expected to run on a normal desktop machine without GPIO, camera, serial, and actuator devices.

## Public Artifacts

- `docs/system-architecture.md`: control-stack diagram.
- `docs/source-release-plan.md`: release boundary and cleanup notes.
