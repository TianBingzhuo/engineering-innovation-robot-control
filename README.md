# Engineering Innovation Robot Control

工程创新智能救援机器人控制系统

This repository records software and mechanical artifacts from an intelligent rescue robot project for the "Intelligence+" track of the Hebei Provincial College Student Engineering Practice and Innovation Ability Competition.

本仓库整理的是工创赛智能救援赛项相关资料：Raspberry Pi 控制脚本、OpenCV 识别逻辑、GPIO/I2C/串口硬件接口、执行机构控制，以及部分机械结构 CAD 文件。

## System Overview

```text
camera input
  -> OpenCV / image processing
  -> target decision / robot control logic
  -> GPIO / I2C / serial / actuator commands
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

This project was completed in a team competition setting. The award and engineering results belong to the collaborative work of the participating students.

本项目为团队竞赛成果。代码、机构、硬件集成、调试、文档和现场测试都包含队友贡献。本仓库由田秉卓维护，用于整理其中可公开复盘的工程内容。

Known contribution split:

| Member | Main contribution |
|---|---|
| 王朔 | Mechanical structure fabrication and mechanism bring-up. |
| 张家毓 | Mechanical structure fabrication and mechanism bring-up. |
| 苏玉轩 | Vision/perception code, including camera and OpenCV recognition logic. |
| 田秉卓 | All non-vision software work: Raspberry Pi main control flow, GPIO/I2C/serial interfaces, actuator logic, manual/automatic control modes, integration debugging, and public repository maintenance. |

已知分工如下：

| 成员 | 主要贡献 |
|---|---|
| 王朔 | 机械结构制作与机构调试。 |
| 张家毓 | 机械结构制作与机构调试。 |
| 苏玉轩 | 视觉代码编写，包括摄像头输入与 OpenCV 识别逻辑。 |
| 田秉卓 | 除视觉代码之外的其他代码编写：Raspberry Pi 主控流程、GPIO/I2C/串口接口、执行机构逻辑、手动/自动控制模式、系统联调，以及本公开仓库维护。 |

Original authorship note: during the competition period, part of the non-vision software was drafted or debugged with assistance from the then-available ChatGPT. Codex was not available at that time and was not part of the original competition development. The responsibility split above refers to the team's original engineering work and later public-safe repository cleanup.

原始代码说明：竞赛开发阶段，部分非视觉代码曾使用当时可用的 ChatGPT 辅助起草或排错；彼时 Codex 尚不存在，也未参与原竞赛开发。上表分工描述的是团队在原始工程中的实际责任，以及后续公开仓库整理维护工作。

Unless otherwise stated, code and documents authored for this repository are released under the Apache License 2.0. Third-party libraries, hardware SDKs, and vendor materials retain their original licenses.
