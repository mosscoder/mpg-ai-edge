# Obstacle Avoidance on Go2

This document describes obstacle avoidance capabilities on the Go2 robot and how to enable/disable them.

## Default State

Obstacle avoidance is **enabled by default** on the Go2.

## RC Controller Method

The handheld remote controller provides the simplest way to toggle obstacle avoidance:

| Action | Effect |
|--------|--------|
| X (Click) | Avoidance on (default) |
| Y (Long Press for 3s) | Avoidance off |

**Note:** You cannot use the app and companion remote simultaneously.

## unitree_webrtc_connect Status

The `unitree_webrtc_connect` library defines the topic constant but provides **no high-level API or examples** for obstacle avoidance:

| Item | Details |
|------|---------|
| Topic Constant | `RTC_TOPIC["OBSTACLES_AVOID"]` |
| Topic String | `"rt/api/obstacles_avoid/request"` |
| Examples | None in the library |
| Documentation | None |
| High-level API | Not exposed |

**Status:** The topic exists but is not actively supported with helper methods or examples. Using it programmatically would require low-level experimentation with `publish_request_new()`.

## Sensor Information

Obstacle avoidance uses the 4D LiDAR L1 sensor:

- **Field of View:** 360° × 90°
- **Minimum Detection Distance:** 0.05m

## Limitations

1. **Forward direction only** - Obstacle avoidance only works when moving forward
2. **Cannot combine app control with companion remote** - Choose one control method

## References

- [Go2 User Manual](https://static.generation-robots.com/media/Go2-User-Manual.pdf)
- [Go2 Handheld Remote Control Manual](https://static.generation-robots.com/media/Go2-Handheld-Remote-Control.pdf)
- [unitree_webrtc_connect GitHub](https://github.com/legion1581/unitree_webrtc_connect)
- [Library Constants](https://github.com/legion1581/unitree_webrtc_connect/blob/main/unitree_webrtc_connect/constants.py)
