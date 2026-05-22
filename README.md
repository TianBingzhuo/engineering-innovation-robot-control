# Engineering Innovation Robot Control

工程创新智能救援机器人控制系统

This repository records software and mechanical artifacts from an intelligent rescue robot project for the "Intelligence+" track of the Hebei Provincial College Student Engineering Practice and Innovation Ability Competition.

本仓库整理的是工创赛智能救援赛项相关资料：Raspberry Pi控制脚本、OpenCV识别逻辑、GPIO/I2C/串口硬件接口、执行机构控制，以及部分机械结构CAD文件。

## System Overview

```text
camera input
  -> OpenCV / image processing
  -> target decision
  -> main control loop
  -> GPIO / I2C / actuator commands
  -> motors, servos, fan, release gate
  -> logs and debugging
```

## Repository Layout

| Path | Description |
|---|---|
| `src/robot_control.py` | Raspberry Pi robot control script from the competition project. |
| `src/control_service.py` | Desktop-friendly command normalization and fail-safe control sketch. |
| `src/rpc_client_demo.py` | Small command-dispatch demo. |
| `docs/system-architecture.md` | Control architecture notes. |
| `hardware/cad/` | SolidWorks part files for mechanism components. |

Run:

```bash
python src/rpc_client_demo.py
python src/control_service.py
```

`src/robot_control.py` targets Raspberry Pi hardware and is not expected to run on a normal desktop machine without GPIO, camera, serial, and actuator devices.

## Team And Copyright

This project was completed in a team competition setting. The award and engineering results belong to the collaborative work of the participating students. Team members contributed to robot software, mechanism design, hardware integration, testing, documentation, and on-site debugging.

本项目为团队竞赛成果。代码、机构、硬件集成、调试、文档和现场测试都包含队友贡献。本仓库由田秉卓维护，用于整理其中可公开复盘的工程内容；后续如需补充更精确的成员署名或贡献说明，应以团队共识为准。

Unless otherwise stated, code and documents authored for this repository are released under the Apache License 2.0. Third-party libraries, hardware SDKs, and vendor materials retain their original licenses.
