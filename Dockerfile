# syntax=docker/dockerfile:1
FROM ros:jazzy-ros-base-noble AS ros
SHELL ["/bin/bash", "-c"]
WORKDIR /opt/urc_ws
COPY ros2_ws/src src
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions python3-rosdep ros-jazzy-rviz2 && \
    (test -e /etc/ros/rosdep/sources.list.d/20-default.list || rosdep init) && \
    rosdep update --rosdistro jazzy && \
    source /opt/ros/jazzy/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy && \
    colcon build --symlink-install && rm -rf /var/lib/apt/lists/*
COPY --chmod=755 docker/urc-ros /usr/local/bin/urc-ros
RUN apt-get update && apt-get install -y --no-install-recommends python3-venv && \
    rm -rf /var/lib/apt/lists/* && python3 -m venv /opt/urc/inference && \
    /opt/urc/inference/bin/pip install --no-cache-dir 'numpy<2' 'onnxruntime>=1.19,<2'
EXPOSE 9090
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["urc-ros", "ros2", "launch", "urc_bringup", "twin.launch.py", "sim:=godot", "bridge_address:=0.0.0.0"]

FROM ros AS rviz
ENV QT_X11_NO_MITSHM=1 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    DISPLAY=:99
RUN apt-get update && apt-get install -y --no-install-recommends \
        libegl1 libgl1-mesa-dri libglu1-mesa libxkbcommon-x11-0 \
        novnc openbox websockify x11-utils x11vnc xauth xvfb && \
    rm -rf /var/lib/apt/lists/*
COPY --chmod=755 docker/urc-rviz /usr/local/bin/urc-rviz
EXPOSE 6080
CMD ["urc-rviz"]

FROM rviz AS desktop
ARG GODOT_VERSION=4.6.3
ARG TARGETARCH
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/urc/gazebo/lib
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates cmake curl g++ libasound2t64 mesa-utils unzip \
        ros-jazzy-ros-gz ros-jazzy-gz-sim-vendor ros-jazzy-gz-transport-vendor \
        ros-jazzy-gz-msgs-vendor && rm -rf /var/lib/apt/lists/*
RUN case "$TARGETARCH" in amd64) godot_arch=x86_64; godot_platform=linux.64 ;; arm64) godot_arch=arm64; godot_platform=linux.arm64 ;; *) exit 1 ;; esac && \
    asset="Godot_v${GODOT_VERSION}-stable_linux.${godot_arch}" && \
    curl --fail --location --retry 3 "https://downloads.godotengine.org/?version=${GODOT_VERSION}&flavor=stable&platform=${godot_platform}&slug=linux.${godot_arch}.zip" -o /tmp/godot.zip && \
    unzip -j /tmp/godot.zip "$asset" -d /tmp && \
    install -m 755 "/tmp/$asset" /usr/local/bin/godot && rm "/tmp/$asset" /tmp/godot.zip && \
    godot --version
COPY gazebo /opt/urc/gazebo-src
RUN source /opt/ros/jazzy/setup.bash && \
    cmake -S /opt/urc/gazebo-src -B /tmp/urc-gazebo-build -DURC_GAZEBO_VENDOR=ON \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/urc/gazebo && \
    cmake --build /tmp/urc-gazebo-build --parallel 2 && \
    cmake --install /tmp/urc-gazebo-build && rm -rf /tmp/urc-gazebo-build
WORKDIR /opt/urc/project
COPY godot godot
COPY config config
COPY tools tools
COPY requirements-train.txt ./
COPY --chmod=755 docker/urc-desktop /usr/local/bin/urc-desktop
RUN godot --headless --path godot --import && \
    python3 tools/generate_gazebo.py --seed 42 --output /tmp/urc-test-world/urc.sdf && \
    urc-ros gz sdf -k /tmp/urc-test-world/urc.sdf && rm -rf /tmp/urc-test-world
EXPOSE 6080
CMD ["urc-desktop"]

FROM desktop AS trainer
ARG TORCH_INDEX_URL=https://download.pytorch.org/whl/cpu
RUN python3 -m venv /opt/urc/training && \
    /opt/urc/training/bin/pip install --no-cache-dir --index-url "$TORCH_INDEX_URL" \
        torch==2.7.1 torchvision==0.22.1 && \
    /opt/urc/training/bin/pip install --no-cache-dir -r requirements-train.txt
ENV PATH=/opt/urc/training/bin:$PATH
CMD ["bash"]

FROM desktop AS simulator
