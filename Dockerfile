# Dockerfile for the mavsim ROS2 bridge (user_repo_new)
# Multi-platform build supporting Linux, macOS, and Windows
#
# Produces mavlab/mavsim-controller:latest - build context is the repository
# root (parent of user_repo_new/), since this also needs ros2_ws/src/interfaces
# and sensor_bridge/, which live outside user_repo_new/. Build with:
#   docker build -f user_repo_new/Dockerfile -t mavlab/mavsim-controller:latest .
# (from repo root), or just run user_repo_new/build.sh.
#
# The actual bridge/controller logic (base_controller.py, python_controller.py,
# run_controller.py, observer.py, visualizer_server.py, recording_service.py,
# local_sensor_generator.py) lives under user_repo_new/core/ and is baked in
# below as a fallback for a bare `docker pull` + hand-rolled `docker run` -
# user_repo_new/start.sh/start.bat bind-mount the same files from a live
# checkout on top of these, so local edits take effect on a container
# restart without needing to rebuild this image.

# Use ROS2 Humble base image (Ubuntu 22.04 based)
FROM ros:humble-ros-base

# Set working directory
WORKDIR /app

# Install rosbag2, MCAP storage (for client-side recording), and build tools
#
# libegl1: provides libEGL.so.1, the vendor-neutral EGL loader. Without it,
# GPU-accelerated headless Chromium (see user_repo_new/core/observer.py)
# fails to initialize even when NVIDIA's own EGL driver (libEGL_nvidia.so,
# injected by `docker run --gpus all` via nvidia-container-toolkit) is
# present - ANGLE calls dlopen("libEGL.so.1") directly and there's nothing
# else on the dlopen path to satisfy it. libgl1/libglvnd0/libglx0 (the
# GLX-side equivalents) already get pulled in transitively, but libegl1 does
# not.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-common-interfaces \
    ros-humble-rosbridge-suite \
    libegl1 \
    python3-colcon-common-extensions \
    python3-pip \
    python3.10 \
    && rm -rf /var/lib/apt/lists/*

# ROS2 environment is already set up in base image
ENV ROS_DISTRO=humble

# Copy interfaces package (custom message types) - always from ros2_ws/src,
# NEVER from user_repo_new/interfaces/ (a separate, divergent reference copy
# for external users building their own ROS2 packages - different content,
# not interchangeable with this one).
RUN mkdir -p /ros2_ws/src
COPY ros2_ws/src/interfaces /ros2_ws/src/interfaces

# Build interfaces package
WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-select interfaces --cmake-args -DCMAKE_BUILD_TYPE=Release" || \
    echo "Warning: Interfaces build may have failed, but continuing..."

# Source ROS2 Humble and interfaces workspace in environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /app

# Copy sensor bridge package and install it
# Note: Build context should be repository root (parent of user_repo_new/)
# Copy the entire sensor_bridge package structure
COPY sensor_bridge/pyproject.toml /app/sensor_bridge_setup/
COPY sensor_bridge/mavsim_sensor_bridge /app/sensor_bridge_setup/mavsim_sensor_bridge

# Install sensor bridge package (non-editable so it and websockets/numpy/pyyaml are in site-packages)
RUN cd /app/sensor_bridge_setup && pip install --no-cache-dir . || \
    echo "Warning: sensor_bridge installation failed, continuing..."

# Copy requirements first for better layer caching
COPY user_repo_new/core/requirements.txt .

# Install Python dependencies (including test dependencies)
RUN pip install --no-cache-dir -r requirements.txt && \
    pip install --no-cache-dir pytest pytest-asyncio pytest-mock

# Headless sensor observer (plans/plan_headless_observer.md): a headless Chromium
# tab, launched as a subprocess of base_controller.py in this same container, so
# camera/lidar sensor data streams to the local bridge even when no human has a
# browser tab open. --with-deps installs the OS-level libs Chromium needs (this
# base image is Ubuntu 22.04 via ros:humble-ros-base, which it supports). GPU
# passthrough supported when available (see core/observer.py), falls back to
# software WebGL (SwiftShader) otherwise.
RUN pip install --no-cache-dir playwright && \
    playwright install --with-deps chromium

# Copy bridge core infrastructure (all connection/handshake/recording code).
# Baked in as a fallback only - start.sh/start.bat bind-mount these same
# files from user_repo_new/core/ at `docker run` time, which take precedence
# over what's copied here, so routine code changes don't need a rebuild.
COPY user_repo_new/core/python_controller.py .
COPY user_repo_new/core/base_controller.py .
COPY user_repo_new/core/run_controller.py .
COPY user_repo_new/core/recording_service.py .
COPY user_repo_new/core/local_sensor_generator.py .
COPY user_repo_new/core/observer.py .

# Local ROS2 topic visualizer: a standalone Flask app + rosbridge websocket,
# both launched as subprocesses of base_controller.py in every mode
# (CLI/web/token), so a browser can inspect the local ROS2 topics (time
# histories, camera, point cloud, camera+lidar overlay) even when the user
# only has SSH/browser access and no rviz2/X11.
COPY user_repo_new/core/visualizer_server.py .
COPY user_repo_new/core/vendor/three.min.js /app/static/three.min.js

# Keyboard teleop (plans/plan_teleop.md): an rclpy node that publishes
# interfaces/Actuator commands on /<vessel>/actuator_cmd from browser
# keypresses, plus its own page/WebSocket server - launched as a subprocess
# of base_controller.py the same way as rosbridge/visualizer above. Flat
# into /app (not a package), matching every other core script - start.sh
# bind-mounts the same files from user_repo_new/teleop/ on top of these.
COPY user_repo_new/teleop/allocation.py .
COPY user_repo_new/teleop/teleop_node.py .

# Expose sensor bridge ports for vessel *
# 70*1: Camera
# 70*2: Lidar
# 70*3: Imaging Sonar
# 70*4: Depth Camera
# 70*5: Auxiliary
EXPOSE 7011 7012 7013 7014 7015
EXPOSE 7021 7022 7023 7024 7025
EXPOSE 7031 7032 7033 7034 7035
EXPOSE 7041 7042 7043 7044 7045
EXPOSE 7051 7052 7053 7054 7055
EXPOSE 7061 7062 7063 7064 7065
EXPOSE 7071 7072 7073 7074 7075
EXPOSE 7081 7082 7083 7084 7085
EXPOSE 7091 7092 7093 7094 7095

# Web app port for --mode web
EXPOSE 8888

# ROS2 topic visualizer: rosbridge websocket + its static/API Flask server
EXPOSE 9090 8899

# Keyboard teleop: page + key/telemetry WebSocket
EXPOSE 8900 8901

# Set Python unbuffered for real-time logging
ENV PYTHONUNBUFFERED=1

# Make sensor bridge importable (pip install may only install metadata as UNKNOWN)
ENV PYTHONPATH=/app/sensor_bridge_setup

# Entrypoint: always dispatches to run_controller.py, which auto-discovers
# the client's controller file (my_controller.py, mounted by start.sh/
# start.bat) and either runs the CLI/token-mode control loop directly, or -
# for --mode web - is bypassed entirely in favor of bridge_webapp.py (see
# start.sh's web-mode docker run, which overrides --entrypoint directly).
RUN echo '#!/bin/bash' > /app/entrypoint.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /app/entrypoint.sh && \
    echo 'source /ros2_ws/install/setup.bash' >> /app/entrypoint.sh && \
    echo 'exec python3 run_controller.py "$@"' >> /app/entrypoint.sh && \
    chmod +x /app/entrypoint.sh

# Copy test files into container (for running tests)
COPY user_repo_new/core/tests /app/tests

# Default entrypoint: auto-discovers and runs client's my_controller.py
# Client mounts their code as: -v ./bridge_controller.py:/app/user_code/my_controller.py
# For testing, override entrypoint: docker run --entrypoint="" ... bash -c "..."
ENTRYPOINT ["/app/entrypoint.sh"]
