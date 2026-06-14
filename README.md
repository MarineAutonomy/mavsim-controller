# mavsim Bridge Controller (`user_repo_new`)

Write your mavsim controller in **any language**, inside or outside Docker.

Instead of editing a Python `control_loop()` that runs inside the container
(the old [`user_repo`](../user_repo/) model), this workspace runs a **bridge**:
a container that connects to the simulation and re-exposes it as plain **local
ROS2 topics**. Your own code just subscribes to telemetry and publishes actuator
commands — in Python, C++, MATLAB, or anything with ROS2 bindings.

```
 your code (any language)                 bridge container (this repo)            cloud / sim
 ┌─────────────────────────┐  ROS2   ┌──────────────────────────────┐  REST/WS  ┌──────────┐
 │ subscribe vessel_state  │◀────────│ publish telemetry topics      │◀──────────│ rosbridge│
 │ publish  actuator_cmd   │────────▶│ store latest cmd, forward via │──────────▶│ backend  │
 └─────────────────────────┘         │ REST on a timer               │           │  + sim   │
                                      └──────────────────────────────┘           └──────────┘
```

## What the bridge exposes (local ROS2)

For every vessel **you own** (bound via your controller code or token):

| Topic | Type | Direction |
|-------|------|-----------|
| `/<vessel>/vessel_state` | `std_msgs/Float64MultiArray` | bridge → you (telemetry) |
| `/<vessel>/vessel_state_der` | `std_msgs/Float64MultiArray` | bridge → you |
| `/<vessel>/odometry_sim` | `nav_msgs/Odometry` | bridge → you |
| `/<vessel>/<sensor>` (with `--enable-sensors`) | `Imu` / `NavSatFix` / `interfaces/DVL` / `CompressedImage` / `PointCloud2` | bridge → you |
| `/<vessel>/actuator_cmd` | `interfaces/Actuator` | **you → bridge (commands)** |

The bridge stores the latest `actuator_cmd` it receives and, on a timer
(`--rate`, default 10 Hz), forwards it to the simulator over REST.

`interfaces/Actuator`: fill `actuator_names` (`cs_01`, `th_01`, …) and
`actuator_values` (control surfaces in degrees, thrusters in RPM). See
[`interfaces/README.md`](interfaces/README.md).

## Quick start (Linux)

```bash
# 1. Get a controller code from the mavsim simulation UI, then start the bridge:
./start.sh ABC123

# 2. In another terminal, run your controller against the local ROS2 topics.
#    (build interfaces once: see interfaces/README.md)
export ROS_DOMAIN_ID=0
python3 examples/python_ros2/example_controller.py --vessel matsya_01 --cs cs_01 --th th_01
```

Both must share the same `ROS_DOMAIN_ID` and be on the same machine (the bridge
uses `--network host` on Linux). To run your code in its own container instead,
see [`examples/docker/`](examples/docker/).

## Files

| File | Purpose |
|------|---------|
| `start.sh` / `start.bat` | Launch the bridge (Linux/macOS, Windows) |
| `bridge_controller.py` | The bridge itself — **do not edit**; mounted into the image |
| `bridge_webapp.py` | Web control panel for `--mode web` |
| `interfaces/` | The `interfaces` ROS2 package (build it for your own code) |
| `examples/python_ros2/` | Example controller (rclpy) |
| `examples/docker/` | Bridge + your controller as sibling containers |
| `examples/cpp_ros2/` | C++ / MATLAB snippets |
| `camera_viewer.html` | Standalone camera viewer (with `--enable-sensors`) |
| `terminalMessages.py` | Optional colored-print helper for your own scripts |
| `recordings/` | MCAP bags land here (created automatically) |

## Options

```
CLI / token mode:
  ./start.sh <code> [--vessel-name NAME] [--backend-url URL] [--enable-sensors]
             [--rate HZ] [--ros-domain-id N] [--cmd-timeout SEC] [--observe-others]
  ./start.sh --token /path/to/token.json [same options]

Web mode:
  ./start.sh --mode web [--port 8888] [--backend-url URL] [--ros-domain-id N]
```

| Option | Meaning |
|--------|---------|
| `--rate HZ` | How often the latest command is forwarded to the sim (default 10) |
| `--ros-domain-id N` | `ROS_DOMAIN_ID` for the bridge; your code must match (default 0) |
| `--cmd-timeout SEC` | Stop forwarding a command if none arrives within this window (default 1.0; `0` = always resend last) |
| `--observe-others` | Also publish **read-only** `odometry_sim` + `actuator_state` for vessels you do NOT own |
| `--enable-sensors` | Expose camera/lidar/imu/... topics and enable the camera viewer |
| `--vessel-name NAME` | Bind a specific vessel (otherwise auto/all available) |

## Multiple users, multiple vehicles

Each user runs **their own bridge** on their own machine and connects to the
same simulation through the backend:

- A **controller code** binds the available vessel(s); a **token** (downloadable
  from the UI) binds a specific subset, e.g. one vessel per user.
- The backend enforces **one controller per vessel** — a second machine trying
  to bind an already-owned vessel is rejected.
- The bridge **only exposes `/<vessel>/actuator_cmd` for vessels it owns.**
  Vessels driven from other machines have **no local command topic here**, so
  you can never accidentally command someone else's vehicle.

### Seeing other vessels (`--observe-others`)

For collision avoidance or multi-agent coordination, add `--observe-others`
(CLI) or tick the checkbox in web mode. The bridge then also publishes, for
vessels you do **not** own:

- `/<vessel>/odometry_sim` (`nav_msgs/Odometry`) — read-only
- `/<vessel>/actuator_state` (`interfaces/Actuator`) — read-only

`actuator_state` is the simulator-**reported** actuator value (from
`vessel_state`), not another agent's raw command — those stay on their own
machines. No command topic is ever created for non-owned vessels.

## Web mode

```bash
./start.sh --mode web        # open http://localhost:8888
```

The panel lets you enter a code/token, set rate / command-timeout / domain id,
toggle **Enable Sensors** and **Observe Others**, start/stop the bridge, watch
logs, and preview the camera. (Web mode launches the bridge via a mounted
script, so no image rebuild is required.)

## Recordings

If recording was enabled at simulation start, MCAP bags are written to
`recordings/` and uploaded to the backend (they appear on the Recordings page).

## Migrating from `user_repo`

| `user_repo` (old) | `user_repo_new` (this) |
|-------------------|------------------------|
| Edit `my_controller.py` `control_loop()` in Python, runs in container | Run any-language code outside; publish `actuator_cmd` over ROS2 |
| Logic coupled to `BaseController` | Logic decoupled; only the ROS2 topic contract matters |
| Single process | Bridge process + your controller process |

Your old control logic maps directly: whatever `control_loop()` returned
(`{'cs_01': x, 'th_01': y}`) becomes an `interfaces/Actuator` message you publish
on `/<vessel>/actuator_cmd`.

## Requirements

- Docker ([install](https://docs.docker.com/get-docker/))
- For native (non-Docker) user code: ROS2 (e.g. Humble) + the built `interfaces` package
- The mavsim web platform running/reachable (`./start.sh` at the repo root, or a remote backend URL)
