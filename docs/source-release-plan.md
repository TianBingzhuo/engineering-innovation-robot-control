# Release Boundary

This repository includes a cleaned public snapshot of the robot control script and mechanism CAD parts. It should still avoid copying board vendor packages, tutorial assets, system images, raw videos, registration forms, and generated build outputs.

## Public Layout

```text
src/
  robot_control.py
  control_service.py
  rpc_client_demo.py
hardware/
  cad/
docs/
  system-architecture.md
  source-release-plan.md
```

## Cleanup Checklist

- remove IP addresses, Wi-Fi names, device hostnames, tokens, local paths, and teammate information;
- replace raw camera/video samples with synthetic or cropped examples;
- document dependencies such as OpenCV, PiCamera2, GPIO, I2C, and servo/motor
  libraries instead of committing copied package directories;
- add public photos or screenshots only after checking that there are no faces, names, certificates, or private labels.

## Current Release

The current release is code-and-artifact focused: `robot_control.py` records the actual control script, `control_service.py` and `rpc_client_demo.py` provide desktop-safe demos, and `hardware/cad/` keeps mechanism part files.
