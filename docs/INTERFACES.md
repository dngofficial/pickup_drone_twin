# Implemented interfaces

`urc_core` owns estimation, perception, tracking, routing, and mission control. `urc_bringup` exposes typed ROS 2 topics, services, actions, and TF. Godot connects through `rosbridge_server`. Gazebo sensors connect through `ros_gz_bridge` and the Gazebo adapter.

| Flow | Implemented topics and behavior |
|---|---|
| Clock and local sensing | `/clock`, `/drone/lidar/packet` v2 with ack on `/slam/status`, `/drone/scan`, `/drone/range/down`, `/joint_states`. Godot additionally produces up range, IMU and pressure. |
| Localization | `/localization/tag_observations`, `/drone/odom`, `/drone/pose`, `/slam/map` and `/localization/status`. Dynamic `map → odom` uses gated, rate-limited XYZYaw corrections. `odom → base_link` stays local. Robot State Publisher owns camera and gripper frames. |
| Perception | `/drone/camera/image/compressed`, `/drone/camera/camera_info`, `/drone/camera/depth/image_raw`. `/perception/oracle/detections_2d` and `/perception/oracle/depth_samples` share capture time and optical geometry. Selected detector emits `/perception/detections_2d`; `/perception/detections_3d` and `/objects/tracks` are map-frame ROS products. |
| Motion and manipulation | `/control/manual`, `/drone/cmd_vel` stamped in `base_link`, `/drone/estop`, `/drone/gripper/command`, `/drone/gripper/status`, `/bins/delivery_event`. The mission node publishes actuator commands. |
| Mission and UI | `/mission/status`, `/mission/path`, `/mission/coverage`, `/drone/battery`, `/visualization/objects`, `/visualization/site`, `/drone/camera/overlay/image`, `/ui/request`, `/ui/response`, `/ui/telemetry`, `/sim/control/request`, `/sim/control/response`. |

`urc_core/runtime.py` is the authoritative capability table. `urc_bringup/node.py` uses sensor-data QoS for high-rate inputs and transient-local depth one for retained visualizations and state. The Gazebo transport uses its own `/gz/urc` names and an adapter rather than bridging custom interfaces directly.

Godot oracle frames run at 5 Hz alongside sparse depth samples. Headless mode raycasts those samples without requiring a renderer. The trained detector uses JPEG pixels and registered 16-bit depth. The sparse oracle positions come through pixel/intrinsics back-projection and tracked positions are stored in the local `odom` frame. No simulator ID appears in the mission target command.

A* uses an inflated grid in the transit plane and rejects diagonal corner cuts. Obstacle heights are derived from local SLAM returns. Before descent the selected target requires fresh observations. Short-range LiDAR, clearance and a geofence constrain velocity. Loss of the local estimate holds motion and can fail the mission. Long tag outages trigger return through a station-tag viewing position. The optional beacon is reserved. ROS provides `/mission/start` as an action, pause/resume, safety services, bin and detector services, manual selection, gripper and scene commands.

Map-side site and surveyed tag frames are static. The initial map-to-odom transform accounts for the 0.65 m body offset and can change with station observations. TF and detections use the applied correction. The relative barometer and scan registration estimate altitude while the downward range checks clearance over slopes and objects. A local velocity prior is simulated from the motion of the body in both backends.

The Gazebo backend generates the same seeded site layout with simplified prop shapes, a fixed camera pitch, and simulated local flow, pressure and range. The editor can pause and step it. ROS/Gazebo wall-clock and sensor timing parity remain to be measured on Harmonic. `sim:=gazebo` requires the separately built gz-sim8 plugin.
