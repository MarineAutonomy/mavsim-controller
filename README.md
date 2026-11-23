# mavsim-controller

ROS2 bridge that connects your controller to a MAVSim simulation session. The easiest way to get started is **web mode**, which opens a local UI in your browser to start and stop sessions.

## Prerequisites

You need [Docker Desktop](https://docs.docker.com/get-docker/) installed and running.

### Windows

1. Download Docker Desktop for Windows from https://docs.docker.com/desktop/setup/install/windows-install/
2. Run the installer and follow the prompts (WSL 2 is required; the installer will guide you)
3. Start **Docker Desktop** from the Start menu and wait until it shows **Engine running**
4. Verify in PowerShell or Git Bash:

```bash
docker --version
```

### macOS

1. Download Docker Desktop for Mac from https://docs.docker.com/desktop/setup/install/mac-install/
   - Apple Silicon: choose the **Apple Chip** build
   - Intel: choose the **Intel Chip** build
2. Open the `.dmg`, drag Docker to Applications, then launch **Docker**
3. Complete the first-run setup and wait until the menu-bar whale icon shows Docker is running
4. Verify in Terminal:

```bash
docker --version
```

> **Note:** Host networking (used by `start.sh` on Mac) needs Docker Desktop **4.34+**. Enable host networking under **Settings → Resources → Network** if prompted.

### Linux

1. Install Docker Desktop for Linux from https://docs.docker.com/desktop/setup/install/linux/
   - Or install Docker Engine only: https://docs.docker.com/engine/install/
2. Start Docker Desktop (or ensure the Docker daemon is running), then add your user to the `docker` group if needed:

```bash
sudo usermod -aG docker $USER
# log out and back in for the group change to take effect
```

3. Verify:

```bash
docker --version
```

## Pull the image

From any directory:

```bash
docker pull mavlab/mavsim-controller:latest
```

`start.sh` will also pull this image automatically on first run if it is not already present locally.

## Run in web mode

Clone this repository (if you have not already), then from the repo root:

```bash
chmod +x start.sh
./start.sh
```

Web mode is the default when you pass no arguments. You can also start it explicitly:

```bash
./start.sh --mode web
```

When it starts, open the web UI at **http://localhost:8888**.

Optional web-mode flags:

```bash
./start.sh --mode web --port 8888 --backend-url http://localhost:5000 --frontend-url http://localhost:5173
```

| Flag | Default | Description |
|------|---------|-------------|
| `--port` | `8888` | Local web UI port |
| `--backend-url` | `http://localhost:5000` | MAVSim backend URL |
| `--frontend-url` | `http://localhost:5173` | MAVSim frontend URL (needed for camera/lidar rendering) |
| `--ros-domain-id` | `42` | ROS 2 domain ID for the bridge |
| `--enable-terminal` | off | Adds a **Terminal** tab to the visualizer (see below) |
| `--upload` | off | Also upload each recorded bag to the backend, for the dashboard's Recordings panel. Without it, bags are only saved in `./recordings/` |
| `--rosbridge-port` | `9090` | Port of the bridge's own rosbridge, which the visualizer uses. On Linux the bridge shares the host's network, so move it if the host already runs a rosbridge on 9090 |

Recordings are written to `./recordings/`. A local visualizer is available at http://localhost:8899 and keyboard teleop at http://localhost:8900.

### Terminal in the visualizer

With `--enable-terminal` (in any mode, or `-e MAVSIM_ENABLE_TERMINAL=1` on a hand-written `docker run`), the visualizer gets a **Terminal** tab: a bash shell inside the bridge container, with ROS 2 sourced and the bridge's `ROS_DOMAIN_ID` set. Use it to test by hand, for example to reset the simulation or watch a topic:

```bash
ros2 topic pub --once -w 1 /sim_reset std_msgs/msg/Empty
ros2 topic echo /sim_reset_event
```

The terminal has no login: **anyone who can reach port 8899 gets a root shell in the container**, and on Linux the container shares the host's network. Turn it on only on a trusted network. Closing the tab ends its shell.
