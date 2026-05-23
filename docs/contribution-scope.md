# Contribution Scope

本文件用于说明当前公开仓库里的贡献边界、材料边界和署名原则。它不是为了把团队工作拆成单人成果，而是让读者理解哪些内容可以公开复盘、哪些内容属于团队协作或未公开材料。

## Project Scope

This repository focuses on the public-safe part of an intelligent rescue robot project:

- Raspberry Pi based integrated control script.
- OpenCV/camera perception hooks kept inside the original script.
- GPIO, PWM, I2C, serial, motor, fan, roller, and release-actuator control.
- Mechanical CAD artifacts that can help explain the collection/release mechanism.
- Rule-derived documentation that explains why the control system is organized this way.

The code is hardware-bound and reflects a competition-stage robot integration process. It is not a desktop demo or a polished robotics framework.

## Team Contribution Notes

| Member | Main contribution |
|---|---|
| 王朔 | Mechanical structure fabrication and mechanism bring-up. |
| 张家毓 | Mechanical structure fabrication and mechanism bring-up. |
| 苏玉轩 | Vision/perception code, including camera and OpenCV recognition logic. |
| 田秉卓 | Non-vision software: Raspberry Pi main control flow, GPIO/I2C/serial interfaces, actuator logic, manual/automatic control modes, integration debugging, and public repository maintenance. |

中文分工说明：

| 成员 | 主要贡献 |
|---|---|
| 王朔 | 机械结构制作与机构调试。 |
| 张家毓 | 机械结构制作与机构调试。 |
| 苏玉轩 | 视觉代码编写，包括摄像头输入与 OpenCV 识别逻辑。 |
| 田秉卓 | 除视觉代码之外的其他代码编写：Raspberry Pi 主控流程、GPIO/I2C/串口接口、执行机构逻辑、手动/自动控制模式、系统联调，以及本公开仓库维护。 |

## Public Repository Boundaries

Included:

- Source and CAD files that are useful for understanding the engineering chain.
- Documentation written for public reading and resume/project review.
- Rule-derived task notes that explain control requirements without replacing official competition documents.

Not included:

- Private raw competition packages.
- Unreviewed photos, videos, certificates, or teammate-private materials.
- Build artifacts, runtime outputs, local environment files, or third-party binary tools.
- Any claim that the repository alone can reproduce the exact competition robot without the original hardware platform.

## Authorship Note

During the original competition period, part of the non-vision software was drafted or debugged with the then-available ChatGPT. Codex was not available at that time and was not part of the original competition development.

The public repository cleanup was done later to make the project easier to read, attribute, and review.
