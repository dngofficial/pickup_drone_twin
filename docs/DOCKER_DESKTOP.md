# Run the URC twin on Windows with Docker Desktop

Install [Docker Desktop for Windows](https://docs.docker.com/desktop/setup/install/windows-install/), use its WSL 2 engine and leave it in Linux container mode. Docker Desktop runs the Linux containers itself. The Dockerfile uses an Ubuntu 24.04 ROS Jazzy base inside the image. You can run these commands from Windows PowerShell without installing an Ubuntu distribution or a separate Ubuntu VM. Extract the project to a Windows folder and open PowerShell beside `compose.yaml`.

Start Docker Desktop from the Windows Start menu, or run:

```powershell
docker desktop start
```

Wait until Docker Desktop reports that the engine is running, then check:

```powershell
docker version
docker compose version
```

`docker version` must show both **Client** and **Server** sections before you run Compose. An error mentioning `dockerDesktopLinuxEngine` and “The system cannot find the file specified” means the Linux engine is unavailable. Start Docker Desktop and wait for it to finish starting; keep it running while using the simulator. If Docker Desktop is already open, check that it is using Linux containers and resolve any startup error shown in its window.

## Native Windows Godot (default)

Use the installed Windows Godot application for the simulator. ROS runs in Docker. RViz is optional; only its Linux window uses a browser viewer.

Stop an old browser desktop first, then start the ROS bridge:

```powershell
docker compose stop desktop
docker compose up --build -d ros
```

In the Windows Godot project manager, click **Import**, select this project's `godot/project.godot`, open it, and press **F5**. Installing Godot alone does not run this simulator. The project connects automatically to `ws://127.0.0.1:9090`; wait for **ROS CONNECTED**, then click **Start**. Use Godot 4.6 or newer in the 4.x series. The default oracle detector needs no trained checkpoint.

Alternatively, use the launcher from PowerShell at the project root. Replace the example path with the extracted Godot executable, not its ZIP or containing folder. Godot does not need to be on PATH; the launcher accepts the full executable path, as supported by the [Godot command-line documentation](https://docs.godotengine.org/en/stable/tutorials/editor/command_line_tutorial.html).

```powershell
$env:GODOT_EXE = 'C:\path\to\Godot_v4.6.1-stable_win64.exe'
.\tools\start_windows.ps1
```

The launcher checks the Godot version and Docker engine, starts a fresh ROS session, verifies a WebSocket handshake on port 9090, imports the assets, and opens native Godot. Close an existing simulator window before relaunching. A fresh ROS session clears the previous mission, simulation clock and sensor sequences. When rerunning the scene directly from the Godot editor, run `docker compose restart ros` as well. `-GodotExe 'C:\path\to\Godot.exe'` also works instead of the environment variable. `-SceneSeed 43` selects another scene. If Windows blocks local PowerShell scripts, invoke this launcher with a process-only policy override:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\start_windows.ps1 -GodotExe $env:GODOT_EXE
```

## Optional RViz2 view

Add `-Rviz` when launching, or start the viewer separately at any time:

```powershell
.\tools\start_windows.ps1 -Rviz
# Or, with native Godot already running:
docker compose up --build -d rviz
```

Open [the RViz viewer](http://localhost:6080/vnc.html?autoconnect=true&resize=scale). This viewer launches only RViz2. Godot stays in its Windows window. The `rviz` service shares ROS networking and shared memory with `ros` for DDS discovery, following Docker's [service namespace configuration](https://docs.docker.com/reference/compose-file/services/). The ROS domain is 43 in both containers. RViz uses the project's `map` frame and displays the drone, SLAM map, coverage, route, tracks and camera overlay. Stop just the viewer with `docker compose stop rviz`; the simulator continues.

For RViz2 installed directly on a ROS-enabled Linux/WSL system, source the workspace, match `ROS_DOMAIN_ID=43`, and run `rviz2 -d ros2_ws/src/urc_bringup/rviz/twin.rviz --ros-args -p use_sim_time:=true`. That system also needs working DDS connectivity to the Docker container; matching the domain alone does not provide it. The Compose viewer avoids this host-network setup.

## Troubleshoot "WAITING FOR ROS"

This means Godot is running but has not connected to rosbridge. Check from PowerShell:

```powershell
docker version
docker compose ps -a
docker compose logs --tail 100 ros
Test-NetConnection 127.0.0.1 -Port 9090
.\tools\start_windows.ps1 -GodotExe $env:GODOT_EXE -CheckOnly -BridgeTimeoutSeconds 5
```

`-CheckOnly` checks the executable, Docker, and an actual WebSocket handshake without starting containers or launching the simulator. A successful TCP port check alone does not verify rosbridge.

- If `docker version` has no **Server** section, start Docker Desktop and wait for its Linux engine. ROS cannot run until the engine is available.
- If only `desktop` is running, stop it and start `ros` with the commands above. The old desktop service publishes only port 6080 and keeps rosbridge inside its container, so Windows Godot cannot reach it at localhost:9090.
- If `ros` exits or is unhealthy, inspect its logs and rerun `docker compose up --build -d ros`. The bridge must bind to `0.0.0.0` inside Docker; Compose already supplies this argument and publishes port 9090 on Windows loopback.
- If Godot still shows a different address, remove an old `URC_ROSBRIDGE_URL` environment override or launch with `-- --bridge=ws://127.0.0.1:9090`. Hover over the connection line in Mission Control to see its target address. The Godot output logs also report connection attempts and success.

To inspect the running ROS stack, open a second PowerShell window in the same folder.

```powershell
docker compose exec ros urc-ros ros2 node list
docker compose exec ros urc-ros ros2 topic list
docker compose exec ros urc-ros ros2 topic echo /clock --once
docker compose exec ros urc-ros ros2 topic echo /localization/status --once
```

The `/clock` check needs native Godot running and connected. Stop the ROS stack and optional viewer with `docker compose --profile rviz down`. Closing Godot does not stop the containers.

## Legacy browser desktop

The explicit `desktop` service still runs both the simulator and RViz in a virtual desktop. Use it only when that setup is wanted. Stop the native-mode stack before switching; the viewer modes use the same port 6080:

```powershell
docker compose --profile rviz down
docker compose up --build desktop
```

Open [the simulator desktop](http://localhost:6080/vnc.html?autoconnect=true&resize=scale). For a direct Dockerfile build of this legacy mode, use `docker build -t urc-twin .` and `docker run --rm --init -p 127.0.0.1:6080:6080 urc-twin`.

## Gazebo Harmonic and RViz

Stop the native-mode stack with `docker compose --profile rviz down`, or stop the previous legacy desktop session with **Ctrl+C**, then select the Gazebo backend. Gazebo and RViz use the legacy desktop and port.

```powershell
$env:URC_SIM = 'gazebo'
docker compose up --build desktop
```

Reconnect to [the simulator desktop](http://localhost:6080/vnc.html?autoconnect=true&resize=scale). Gazebo uses the generated seed-42 world and starts the mission automatically. RViz is available in the same desktop. Set `$env:URC_SCENE_SEED = '43'` before starting to choose another world. Set `$env:URC_SIM = 'godot'` before restarting to return to the Godot dashboard.

## Download the dataset and train the checkpoint

Sign in to Roboflow, open [API Keys in workspace settings](https://app.roboflow.com/settings/api), and get a **private API key**. The downloader uses the key to access version 1 of the [Hammer-Bottle dataset](https://universe.roboflow.com/urc-cvc4i/hammer-bottle-l3zmd-odqnx). If access fails, check that your signed-in account can export that version. Paste the key only into the local hidden prompt below.

Build the CPU trainer from PowerShell and open its shell.

```powershell
docker compose build trainer
docker compose run --rm trainer
```

In the container shell, generate the mixed dataset, fine-tune the model and export it. Use new output folders for a fresh dataset export or mix. The sample generates 600 synthetic images. The first `yolov8n.pt` is the generic pretrained starting point and downloads during training. The project checkpoint `best.pt` is produced by fine-tuning on your two-class dataset.

```bash
read -rs -p 'Roboflow private API key: ' ROBOFLOW_API_KEY
printf '\n'
export ROBOFLOW_API_KEY
python tools/download_dataset.py --version 1 --output data/real
unset ROBOFLOW_API_KEY
xvfb-run -a -s '-screen 0 1024x768x24' python tools/generate_synthetic.py --seeds 0:50 --frames 12 --output data/synthetic
python tools/mix_dataset.py --real data/real --synthetic data/synthetic --output data/mixed
python tools/train_yolo.py --data data/mixed/mixed.yaml --device cpu --epochs 45 --batch 4
python tools/export_onnx.py --checkpoint runs/train/weights/best.pt --output weights/hammer_bottle_yolov8n.onnx
exit
```

The mixer remaps the dataset classes into `bottle`, `hammer` order and converts polygon annotations to enclosing detection boxes. It keeps exact duplicate images out of different splits. The training script defaults to two data-loader workers; use `--workers` to adjust this for available RAM.

The `runs/train/weights/best.pt` path applies to the first training run. Ultralytics increments the run directory on subsequent runs. Use the new path printed by training in the export command. Docker mounts `data`, `runs` and `weights` from the extracted project folder, so the dataset, checkpoint, ONNX file and its `.classes.json` sidecar persist on Windows.

### Use an NVIDIA GPU for training

Docker Desktop GPU passthrough requires its WSL 2 engine and a compatible NVIDIA Windows driver. The override adds CUDA 12.8 PyTorch. Use these commands in place of the CPU trainer's PowerShell commands. Compose 2.30 or newer is required for the `gpus` setting.

```powershell
docker compose -f compose.yaml -f compose.gpu.yaml build trainer
docker compose -f compose.yaml -f compose.gpu.yaml run --rm trainer python -c "import torch; assert torch.cuda.is_available(); print(torch.cuda.get_device_name(0))"
docker compose -f compose.yaml -f compose.gpu.yaml run --rm trainer
```

In the same training command above, use `--device 0 --batch 4`. The ONNX export and the desktop's inference both run on CPU.

## Run live ONNX inference

YOLO requires your trained two-class model. The project does not bundle trained weights. Until `weights/hammer_bottle_yolov8n.onnx` and `weights/hammer_bottle_yolov8n.classes.json` exist, the dashboard disables YOLO and keeps Oracle available. A generic COCO checkpoint does not provide this project's bottle/hammer class contract. Follow the dataset/training/export steps above to produce the files. The Windows launcher checks for both files before changing the running stack.

After exporting the model, stop native Godot before restarting ROS with the YOLO detector. The `ros` service mounts `weights` and includes CPU ONNX Runtime. Inference uses paired RGB/depth frames in a background worker so the controller can continue processing flight sensors. Only one inference job runs at a time; older frames are skipped while it is busy.

```powershell
$env:URC_DETECTOR = 'yolo'
.\tools\start_windows.ps1 -Rviz
```

Click **Start** in native Godot. Examine the camera overlay and confidence values. The exported checkpoint's held-out real and synthetic metrics are printed by the training script. To switch back, close Godot, set `$env:URC_DETECTOR = 'oracle'`, and rerun the launcher. `-Rviz` is optional for either detector.

To check the ROS logs or stop the stack from a second PowerShell window, run `docker compose logs ros` or `docker compose --profile rviz down`.

## Pause, resume, and E-stop

The main mission button changes from **Start** to **Pause** while running, and to **Resume** while held. Space is the same mission pause/resume shortcut. After an E-stop, click **Clear**, then **Resume**; clearing the latch alone keeps the aircraft held. **Pause sim / Resume sim** separately freezes or resumes simulation time. The main **Resume** action also resumes simulation time when needed. A failed mission must be reset at home before starting again.
