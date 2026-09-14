Learning stack. Sensors and vehicle state are legal inputs. The HD map is for labels and scoring only.

| Folder | What you build |
|---|---|
| `01_control` | Throttle, steer, brake, speed hold. No perception. |
| `02_lane_keeping` | Camera → OpenCV lanes → PID steer. |
| `03_learned_perception` | Same control, but a network estimates offset from the image. |
| `04_behavioral_cloning` | Image (+ speed) → steer/throttle. Knows why it fails. |
| `05_other_agents` | Detect, predict, and plan around other vehicles. |
