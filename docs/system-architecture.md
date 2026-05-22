# System Architecture Draft

```mermaid
flowchart LR
  A["Camera / sensor input"] --> B["OpenCV processing"]
  B --> C["Target decision"]
  C --> D["Main control loop"]
  D --> E["GPIO / I2C / serial interface"]
  E --> F["Motors / servos / release mechanism"]
  D --> G["Logs and debugging"]
```

## ROS 2 Mapping Idea

- camera pipeline -> image topic
- target decision -> target-state topic or service
- main loop -> control node
- actuator commands -> actuator interface
- logs -> observability and fault-recovery notes

