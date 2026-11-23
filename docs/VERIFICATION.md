# ROS 2 smoke checks

Start the ROS launch and one simulator. Use a second terminal configured for the same ROS domain and workspace to check that the simulator and controller are exchanging messages:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 node list
ros2 topic echo /clock --once
ros2 topic echo /localization/status --once
ros2 topic echo /mission/status --once
ros2 run tf2_ros tf2_echo map base_link
```

With native Windows Godot and the Docker ROS bridge running, prefix the `ros2` commands with `docker compose exec ros urc-ros` from PowerShell. For the legacy all-in-browser mode, use `desktop` instead of `ros`. The `tf2_echo` command runs until stopped. Start the optional RViz-only viewer with `docker compose up --build -d rviz` and open `http://localhost:6080/vnc.html?autoconnect=true&resize=scale`.

For Godot, launch a mission before testing the E-stop:

```bash
ros2 action send_goal /mission/start urc_msgs/action/StartMission '{mission_id: ros-smoke}' --feedback
```

From another sourced ROS terminal:

```bash
ros2 service call /safety/estop std_srvs/srv/Trigger '{}'
ros2 topic echo /mission/status --once
ros2 service call /safety/clear_estop std_srvs/srv/Trigger '{}'
ros2 service call /mission/resume std_srvs/srv/Trigger '{}'
```

Check that the aircraft holds after the E-stop, remains held after clearing it, and moves again only after resume. In RViz, use `map` as the fixed frame and check the localized vehicle, SLAM points, coverage and camera overlay. The camera display needs a rendered simulator.

The remaining offline tests cover mission safety, localization, routing, model decoding, and Gazebo world generation. Run `python -m pytest -q` at the project root with NumPy, SciPy, PyYAML, Pillow, and pytest installed. These tests exercise modules used by the ROS nodes. Live ROS and Docker runs still need an installed runtime.

Run the Godot dashboard regression check with Godot 4.6 or newer:

```bash
godot --headless --path godot --script res://tests/test_dashboard.gd
```

It checks Start/Pause/Resume, E-stop guidance, the actual simulation pause state, and keyboard focus. In the native window, also check that Clear leaves an E-stopped mission held until Resume is clicked. Without trained bottle/hammer weights, YOLO is disabled; decoder tests do not establish real detection accuracy.

With a trained model, check both `/perception/detections_2d` and `/perception/detections_3d` during a live flight. YOLO uses matching RGB/depth frames and retries projection when the LiDAR pose arrives later. The ROS adapter runs one inference job in a background worker, skipping frames while busy so model latency does not queue old sensor data. The RGB-D regression tests cover delayed poses and continued clock processing while inference is pending.

The four binary sensor-adapter tests require ROS and are skipped by the native Windows offline run. With the `ros` container running, execute them from PowerShell:

```powershell
docker compose cp tests/test_ros_adapter.py ros:/tmp/test_ros_adapter.py
docker compose exec -T ros urc-ros python3 /tmp/test_ros_adapter.py
```

These checks preserve depth-pixel byte order, compressed-image data, image/cloud output arrays, and ordinary ROS fields. Images remain byte buffers inside the controller, and sensor callbacks share a mutually exclusive callback group so workers do not contend for the controller lock. During a live Start check, confirm that `/localization/status` reports `odometry_valid: true`, the aircraft takes off and enters `SEARCH`, and LiDAR acknowledgments continue near the configured 10 Hz. The existing freshness checks still stop a mission if localization or clearance is unavailable.
