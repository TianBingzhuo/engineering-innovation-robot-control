# Engineering Innovation Robot Control

工程创新智能救援机器人控制系统

This repository records software and mechanical artifacts from an intelligent rescue robot project for the "Intelligence+" track of the Hebei Provincial College Student Engineering Practice and Innovation Ability Competition.

本仓库整理的是工创赛智能救援赛项相关资料：Raspberry Pi 控制脚本、OpenCV 识别逻辑、GPIO/I2C/串口硬件接口、执行机构控制，以及部分机械结构 CAD 文件。

## Project At A Glance

| Item | Summary |
|---|---|
| Competition | 2025 China College Student Engineering Practice and Innovation Ability Competition, "Intelligence+" intelligent rescue event. |
| System type | Raspberry Pi based rescue robot control system with camera perception, actuator control, and manual/autonomous mode handling. |
| Repository focus | Public-safe code and mechanism artifacts that explain the control chain. |
| My role | Non-vision software: main control flow, GPIO/I2C/serial interfaces, actuator logic, manual/automatic modes, integration debugging, and repository maintenance. |
| Not included | Private competition packages, teammate-private files, unreviewed media, build/runtime artifacts, and third-party binary tools. |

## Competition Task Context

The robot was built for the intelligent rescue event, where two teams run one robot each on the same field. The robot must start autonomously, complete at least one ordinary rescue target into its own safety zone, and then continue collecting ordinary, core, and dangerous targets under the scoring and collision rules.

这个赛项的核心不是简单巡线或遥控小车，而是“同场对抗 + 自主先手 + 救援目标转移”：机器人需要识别目标颜色和位置，把普通、核心、危险救援目标转移进本方安全区，同时避免进入对方安全区、长时间碰撞和恶意阻挡等违规行为。

See [docs/competition-task.md](docs/competition-task.md) for field layout, target types, match flow, scoring logic, and how those requirements map to the code and mechanism design.

## Visual Materials

No project photos or screenshots are included in this public repository yet. This README intentionally avoids placeholder images; real mechanism or field photos can be added later after teammate, privacy, and competition-material review.

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
| `src/robot_control.py` | Original integrated Raspberry Pi robot control script from the competition project. |
| `docs/contribution-scope.md` | Team contribution split, public repository scope, and attribution notes. |
| `docs/competition-task.md` | Competition task, field, scoring, and engineering constraints. |
| `docs/system-architecture.md` | Control architecture notes. |
| `hardware/cad/` | SolidWorks part files for mechanism components. |

`src/robot_control.py` targets Raspberry Pi hardware and is not expected to run on a normal desktop machine without GPIO, camera, serial, and actuator devices. The repository keeps the original integrated code instead of splitting it into simplified demo modules.

## Suggested Reading Order

1. [docs/competition-task.md](docs/competition-task.md) for the task and scoring context.
2. [docs/system-architecture.md](docs/system-architecture.md) for the control/data flow.
3. [docs/contribution-scope.md](docs/contribution-scope.md) for authorship and public-scope boundaries.
4. [src/robot_control.py](src/robot_control.py) for the original integrated robot script.

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
