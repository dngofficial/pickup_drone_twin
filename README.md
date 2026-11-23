# URC Pickup Drone Twin

ROS 2 Jazzy controls a simulated drone that searches seeded terrain, sorts bottles into Bin A and hammers into Bin B, and returns to the station. The drone uses LiDAR SLAM, station-tag fixes and A* routing. Choose Godot for the flight dashboard or Gazebo Harmonic for the alternate physics and sensor backend. Both connect to the same ROS 2 autonomy stack and RViz configuration.

## Windows with Docker Desktop

Start Docker Desktop and wait for its Linux engine to be running. From PowerShell at the extracted project root:

```powershell
$env:GODOT_EXE = 'C:\path\to\Godot_v4.6.1-stable_win64.exe'
.\tools\start_windows.ps1
```

Godot runs as a native Windows application and connects to ROS in Docker at `ws://127.0.0.1:9090`. Add `-Rviz` to the launcher for an optional [RViz-only browser view](http://localhost:6080/vnc.html?autoconnect=true&resize=scale). The launcher checks Docker and the WebSocket connection before starting Godot. To start ROS separately, run `docker compose up --build -d ros`, then import `godot/project.godot` into Windows Godot and press F5 to run the project. Use Godot 4.6 or newer in the 4.x series.

The [Windows setup guide](docs/DOCKER_DESKTOP.md) covers connection troubleshooting, the optional RViz viewer, Gazebo, training, and ONNX inference. The older all-in-browser desktop remains available explicitly as `desktop`.

## Native Ubuntu 24.04

Install ROS 2 Jazzy and Godot 4.6 or newer in the 4.x series. Build and launch ROS from the project root:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch urc_bringup twin.launch.py sim:=godot rviz:=true
```

In another terminal at the project root, import the Godot assets once and start the simulator:

```bash
godot --headless --path godot --import
godot --path godot -- --seed=42
```

Godot connects automatically to `ws://127.0.0.1:9090`. Use `--bridge=ws://HOST:9090` to change the address. Click **Start** or press Enter. Use **Pilot** or V to switch to ROS-controlled manual flight, WASD/QE/arrows for motion, G for the gripper, T for camera tilt in manual mode, C for the view, M for the map, and L for LiDAR. Space pauses or resumes the mission. X latches E-stop, U clears it, and Space explicitly resumes. The dashboard also supports bin selection, map targets, reset, pause, and step.

To run the ROS bridge in a container while launching Godot on the host, use `docker compose up --build ros`.

## Gazebo Harmonic on native Ubuntu

Follow the Gazebo Harmonic instructions to enable the Gazebo package repository. From the project root:

```bash
sudo apt-get install gz-harmonic libgz-sim8-dev libgz-transport13-dev libgz-msgs10-dev ros-jazzy-ros-gz-bridge cmake g++
cmake -S gazebo -B gazebo/build -DCMAKE_BUILD_TYPE=Release
cmake --build gazebo/build --parallel
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/gazebo/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
godot --headless --path godot --import
python tools/generate_gazebo.py --seed 42 --output "$PWD/data/gazebo/urc_terrain.sdf"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch urc_bringup twin.launch.py sim:=gazebo rviz:=true autostart:=true world:="$PWD/data/gazebo/urc_terrain.sdf"
```

Keep the generated manifest and OBJ beside the SDF. The Gazebo backend has a fixed downward camera and simplified collision geometry. Pause and step using Gazebo’s controls.

## Perception and ROS checks

The default oracle detector uses simulated visibility and depth. It works without a trained model. To train and export YOLOv8, use the [trainer guide](docs/DOCKER_DESKTOP.md#download-the-dataset-and-train-the-checkpoint). On native Jazzy, make ONNX Runtime available to the ROS Python interpreter and pass `detector:=yolo model:=/absolute/path/to/hammer_bottle_yolov8n.onnx` to the launch command. Store the `.classes.json` sidecar next to the model.

- [ROS interfaces](docs/INTERFACES.md)
- [ROS smoke checks](docs/VERIFICATION.md)
- [Asset credits](third_party/NOTICES.md)

The drone is kinematic in Godot. Tag detections use simulated tag geometry. The gripper uses contact checks and rigid-body release. Flight hardware, calibrated sensors, real tag detection, and hardware grasping have not been validated.
