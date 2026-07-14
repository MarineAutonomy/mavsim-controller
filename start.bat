@echo off
setlocal enabledelayedexpansion

REM start.bat - Run the mavsim ROS2 bridge controller (Windows)
REM
REM The bridge connects to the simulation and exposes it as LOCAL ROS2 topics:
REM   - publishes telemetry:  /<vessel>/vessel_state, /vessel_state_der,
REM                           /odometry_sim, plus camera/lidar/imu/gps/encoder
REM                           topics (sensors are always enabled - see
REM                           plans/plan_headless_observer.md)
REM   - subscribes commands:  /<vessel>/actuator_cmd   (interfaces/Actuator)
REM
REM Camera/lidar data is captured in a browser (WebGL rendering), so a headless
REM Chromium tab is launched automatically alongside the bridge to trigger that
REM rendering without needing a human to keep a tab open - it needs to know
REM where the MAVSim frontend is actually hosted via --frontend-url (defaults
REM to http://localhost:5173, override whenever the bridge isn't on the same
REM machine as the frontend - e.g. a remote/lab server, or once deployed to AWS).
REM
REM You then run YOUR controller (any language) so that it subscribes to the
REM telemetry topics and publishes interfaces/Actuator on /<vessel>/actuator_cmd.
REM The bridge forwards the latest command to the simulator over REST.
REM
REM NOTE: Docker Desktop on Windows does not support --network host, so your
REM ROS2 code must run in a SIBLING container on the same Docker network and
REM ROS_DOMAIN_ID as the bridge. See examples\docker\docker-compose.yml.
REM
REM Modes:
REM   Web mode (default - no arguments at all): start.bat
REM     Starts a local web UI to start/stop sessions from a browser - the
REM     easiest way to run this if you don't already have a controller-code
REM     or token file in hand. Passing ANY argument (a controller-code,
REM     --token, or an explicit --mode) opts back into CLI/token mode below.
REM   CLI mode:   start.bat <controller-code> [options...]
REM   Token mode: start.bat --token <path-to-token.json> [options...]
REM   Web mode (explicit): start.bat --mode web [options...]
REM
REM Usage:
REM   start.bat
REM   start.bat <controller-code> [--vessel-name NAME] [--backend-url URL]
REM             [--frontend-url URL] [--rate HZ] [--ros-domain-id N]
REM             [--cmd-timeout SEC] [--observe-others]
REM   start.bat --token <path-to-token.json> [options...]
REM   start.bat --mode web [--port 8888] [--backend-url URL] [--frontend-url URL] [--ros-domain-id N]
REM
REM By default this pulls the published mavlab/mavsim-controller:latest image
REM (any --build flag can be combined with any of the above, in any order,
REM e.g. `start.bat --build` or `start.bat --build --mode web`). Pass --build
REM (or set BUILD_LOCAL=1) to instead build the image locally from .\Dockerfile
REM - only needed if you've changed the Dockerfile itself (new OS dependency,
REM ROS2/Python version bump). Routine changes to core\*.py are picked up on
REM the next run with no rebuild at all, since those files are bind-mounted
REM into the container below.
REM
REM --ros-domain-id defaults to 42 (not 0) on every run, including local Docker
REM testing against a `docker compose` mavsim stack - a local stack assigns
REM each simulation session its own ROS_DOMAIN_ID starting at 0 (one per
REM session slot, MAX_SESSIONS=5 by default -> domains 0-4). ROS2/DDS
REM discovery is network-scoped, not container-scoped, so defaulting to the
REM same domain as a local session would let this bridge silently see (and
REM possibly record) that session's own internal ROS2 topics. 42 stays clear
REM of that range. Irrelevant for real cloud deployments (bridge and
REM simulation task are on separate networks there), but always set anyway.
REM
REM Recordings are saved to .\recordings\ on your machine.

set "DOCKER_IMAGE=mavlab/mavsim-controller:latest"
set "CONTAINER_NAME=mavsim-bridge-%RANDOM%"
set "SCRIPT_DIR=%~dp0"
if "%SCRIPT_DIR:~-1%"=="\" set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"
for %%I in ("%SCRIPT_DIR%\..") do set "REPO_ROOT=%%~fI"

set "DEFAULT_BACKEND_URL=http://localhost:5000"
set "DEFAULT_FRONTEND_URL=http://localhost:5173"
set "DEFAULT_WEBAPP_PORT=8888"
set "RECORDINGS_DIR=%SCRIPT_DIR%\recordings"
set "BRIDGE_FILE=%SCRIPT_DIR%\bridge_controller.py"
set "WEBAPP_FILE=%SCRIPT_DIR%\bridge_webapp.py"
REM Bridge core infrastructure (base_controller.py, python_controller.py,
REM etc.) - bind-mounted into the container below on top of the image's own
REM baked-in copies, so editing any of these locally takes effect on the
REM next container start with no `docker build` needed.
set "CORE_DIR=%SCRIPT_DIR%\core"
set "CORE_FILES=base_controller.py python_controller.py run_controller.py recording_service.py local_sensor_generator.py observer.py visualizer_server.py"

REM ---- Opt-in local build (default is docker pull) ----
REM For maintainers who've changed the Dockerfile itself (new OS dependency,
REM ROS2/Python version bump) - bind-mounting core\ (below) only covers
REM application code, not image-level setup, so a real rebuild is sometimes
REM still necessary. Everyone else keeps getting the published image.
if "%BUILD_LOCAL%"=="1" (set "BUILD_LOCAL=true") else (set "BUILD_LOCAL=false")

REM ---- Parse leading --build / --mode flags (either order) ----
REM Both are stripped from the front of the argument list here, before mode
REM detection below - so e.g. `start.bat --build` (no other args) must still
REM mean "web mode + build locally", not "one argument present, so fall
REM through to CLI mode".
:parse_leading_flags
if "%~1"=="--build" (
    set "BUILD_LOCAL=true"
    shift
    goto :parse_leading_flags
)
if "%~1"=="--mode" (
    if "%~2"=="" (
        echo [ERROR] --mode requires a value: web or cli
        exit /b 1
    )
    set "MODE=%~2"
    shift & shift
    goto :parse_leading_flags
)

REM ---- Detect mode ----
REM No arguments at all defaults to web mode - the friendliest entry point
REM for a user who doesn't want to get a controller-code/token right
REM upfront. Passing anything at all (a controller-code, --token, or an
REM explicit --mode) opts back into CLI/token mode exactly as before.
if not defined MODE (
    if "%~1"=="" (set "MODE=web") else (set "MODE=cli")
)
set "WEBAPP_PORT=%DEFAULT_WEBAPP_PORT%"
REM Deliberately NOT 0 - see the ros-domain-id note near the top of this file.
set "ROS_DOMAIN_ID_VAL=42"

if not "%MODE%"=="web" if not "%MODE%"=="cli" (
    echo [ERROR] Invalid mode: %MODE%
    exit /b 1
)

REM ---- Prerequisites ----
if not exist "%RECORDINGS_DIR%" mkdir "%RECORDINGS_DIR%"

where docker >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Docker is not installed. See https://docs.docker.com/get-docker/
    exit /b 1
)

if not exist "%BRIDGE_FILE%" (
    echo [ERROR] bridge_controller.py not found next to start.bat
    exit /b 1
)

for %%f in (%CORE_FILES%) do (
    if not exist "%CORE_DIR%\%%f" (
        echo [ERROR] %%f not found in %CORE_DIR% - is your checkout up to date?
        exit /b 1
    )
)

if "%BUILD_LOCAL%"=="true" (
    echo [INFO] Building %DOCKER_IMAGE% locally from %SCRIPT_DIR%\Dockerfile ...
    docker build -f "%SCRIPT_DIR%\Dockerfile" -t "%DOCKER_IMAGE%" "%REPO_ROOT%"
    if errorlevel 1 (
        echo [ERROR] Failed to build image
        exit /b 1
    )
) else (
    docker image inspect "%DOCKER_IMAGE%" >nul 2>&1
    if errorlevel 1 (
        echo [INFO] Pulling %DOCKER_IMAGE% ...
        docker pull "%DOCKER_IMAGE%"
        if errorlevel 1 (
            echo [ERROR] Failed to pull image
            exit /b 1
        )
    )
)

REM ---- GPU passthrough (NVIDIA only for now) ----
REM The headless sensor observer defaults to CPU-only SwiftShader rendering,
REM which can become unresponsive under real load (multiple vessels/sensors,
REM heavy post-processing). When a real GPU is available, request passthrough
REM so the observer can use it instead - core\observer.py independently
REM re-checks GPU access from inside the container before picking Chromium's
REM render flags, so a false positive here just falls back to SwiftShader
REM rather than breaking anything. Requires Docker Desktop's WSL2 backend
REM with the NVIDIA driver for WSL installed - see NVIDIA's CUDA-on-WSL docs.
set "GPU_AVAILABLE=false"
where nvidia-smi >nul 2>&1
if not errorlevel 1 (
    nvidia-smi >nul 2>&1
    if not errorlevel 1 (
        docker info 2>nul | findstr /r /c:"Runtimes:.*nvidia" >nul 2>&1
        if not errorlevel 1 set "GPU_AVAILABLE=true"
    )
)
set "GPU_ARGS="
if "%GPU_AVAILABLE%"=="true" set "GPU_ARGS=--gpus all"

REM ===========================================================
REM Web mode
REM ===========================================================
if "%MODE%"=="web" (
    set "BACKEND_URL=%DEFAULT_BACKEND_URL%"
    set "FRONTEND_URL=%DEFAULT_FRONTEND_URL%"
    call :parse_web_args %1 %2 %3 %4 %5 %6 %7 %8 %9

    if not exist "%WEBAPP_FILE%" (
        echo [ERROR] bridge_webapp.py not found next to start.bat
        exit /b 1
    )

    call :rewrite_url "!BACKEND_URL!" CONTAINER_BACKEND_URL
    call :rewrite_url "!FRONTEND_URL!" CONTAINER_FRONTEND_URL

    echo [INFO] Starting mavsim bridge in WEB mode
    echo [INFO]   Image:        %DOCKER_IMAGE%
    echo [INFO]   Web UI:       http://localhost:!WEBAPP_PORT!
    echo [INFO]   Backend:      !BACKEND_URL!
    echo [INFO]   Frontend:     !FRONTEND_URL!
    echo [INFO]   ROS_DOMAIN_ID: !ROS_DOMAIN_ID_VAL!
    echo [INFO]   Recordings:   %RECORDINGS_DIR%
    echo [INFO]   Visualizer:   http://localhost:8899 (time histories, camera, point cloud, overlay)
    if "%GPU_AVAILABLE%"=="true" (
        echo [INFO]   GPU:          NVIDIA GPU detected - passing through for hardware-accelerated rendering
    ) else (
        echo [INFO]   GPU:          none detected - observer uses CPU-only SwiftShader rendering
    )
    echo.

    set "WEB_CMD=source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && cd /app && exec python3 /app/user_code/bridge_webapp.py --port '!WEBAPP_PORT!' --backend-url '!CONTAINER_BACKEND_URL!' --frontend-url '!CONTAINER_FRONTEND_URL!'"

    docker run --rm ^
        --name "%CONTAINER_NAME%" ^
        %GPU_ARGS% ^
        -p "!WEBAPP_PORT!:!WEBAPP_PORT!" ^
        -p "7001-7095:7001-7095" ^
        -p "9090:9090" ^
        -p "8899:8899" ^
        -e "ROS_DOMAIN_ID=!ROS_DOMAIN_ID_VAL!" ^
        -v "%RECORDINGS_DIR%:/tmp/mavsim_bags" ^
        -v "%BRIDGE_FILE%:/app/user_code/my_controller.py:ro" ^
        -v "%WEBAPP_FILE%:/app/user_code/bridge_webapp.py:ro" ^
        -v "%CORE_DIR%\base_controller.py:/app/base_controller.py:ro" ^
        -v "%CORE_DIR%\python_controller.py:/app/python_controller.py:ro" ^
        -v "%CORE_DIR%\run_controller.py:/app/run_controller.py:ro" ^
        -v "%CORE_DIR%\recording_service.py:/app/recording_service.py:ro" ^
        -v "%CORE_DIR%\local_sensor_generator.py:/app/local_sensor_generator.py:ro" ^
        -v "%CORE_DIR%\observer.py:/app/observer.py:ro" ^
        -v "%CORE_DIR%\visualizer_server.py:/app/visualizer_server.py:ro" ^
        --entrypoint /bin/bash ^
        "%DOCKER_IMAGE%" ^
        -c "!WEB_CMD!"
    exit /b %errorlevel%
)

REM ===========================================================
REM CLI / Token mode
REM ===========================================================

set "TOKEN_FILE="
if "%~1"=="--token" (
    if "%~2"=="" (
        echo [ERROR] --token requires a path to a JSON token file
        exit /b 1
    )
    set "TOKEN_FILE=%~2"
    shift & shift
    if not exist "!TOKEN_FILE!" (
        echo [ERROR] Token file not found: !TOKEN_FILE!
        exit /b 1
    )
)

if "%TOKEN_FILE%"=="" if "%~1"=="" (
    echo [ERROR] Controller code or --token is required in CLI mode
    echo [ERROR] (tip: run %~nx0 with no arguments at all for web mode instead)
    echo.
    echo Usage:
    echo   %~nx0 ^<controller-code^> [--vessel-name NAME] [--backend-url URL] [--frontend-url URL] [--rate HZ] [--ros-domain-id N] [--cmd-timeout SEC] [--observe-others]
    echo   %~nx0 --token ^<path-to-token.json^> [options...]
    echo   %~nx0 --mode web [--port 8888] [--backend-url URL] [--frontend-url URL] [--ros-domain-id N]
    exit /b 1
)

set "CONTROLLER_CODE="
if "%TOKEN_FILE%"=="" (
    set "CONTROLLER_CODE=%~1"
    shift
)
set "BACKEND_URL=%DEFAULT_BACKEND_URL%"
set "FRONTEND_URL=%DEFAULT_FRONTEND_URL%"
set "VESSEL_NAME="
set "OBSERVE_OTHERS="
set "CMD_TIMEOUT=1.0"
set "EXTRA_ARGS="

:parse_cli_args
if "%~1"=="" goto :done_cli_args
if "%~1"=="--backend-url" (
    set "BACKEND_URL=%~2"
    shift & shift
    goto :parse_cli_args
)
if "%~1"=="--frontend-url" (
    set "FRONTEND_URL=%~2"
    shift & shift
    goto :parse_cli_args
)
if "%~1"=="--vessel-name" (
    set "VESSEL_NAME=%~2"
    set "EXTRA_ARGS=!EXTRA_ARGS! --vessel-name %~2"
    shift & shift
    goto :parse_cli_args
)
if "%~1"=="--rate" (
    set "EXTRA_ARGS=!EXTRA_ARGS! --rate %~2"
    shift & shift
    goto :parse_cli_args
)
if "%~1"=="--ros-domain-id" (
    set "ROS_DOMAIN_ID_VAL=%~2"
    shift & shift
    goto :parse_cli_args
)
if "%~1"=="--cmd-timeout" (
    set "CMD_TIMEOUT=%~2"
    shift & shift
    goto :parse_cli_args
)
if "%~1"=="--observe-others" (
    set "OBSERVE_OTHERS=true"
    shift
    goto :parse_cli_args
)
set "EXTRA_ARGS=!EXTRA_ARGS! %~1"
shift
goto :parse_cli_args
:done_cli_args

call :rewrite_url "!BACKEND_URL!" CONTAINER_BACKEND_URL
call :rewrite_url "!FRONTEND_URL!" CONTAINER_FRONTEND_URL
set "EXTRA_ARGS=!EXTRA_ARGS! --frontend-url !CONTAINER_FRONTEND_URL!"

echo [INFO] Starting mavsim bridge
echo [INFO]   Image:        %DOCKER_IMAGE%
if not "%TOKEN_FILE%"=="" (echo [INFO]   Token:        %TOKEN_FILE%) else (echo [INFO]   Code:         %CONTROLLER_CODE%)
echo [INFO]   Backend:      %BACKEND_URL%
echo [INFO]   Frontend:     !FRONTEND_URL!
echo [INFO]   ROS_DOMAIN_ID: %ROS_DOMAIN_ID_VAL%
echo [INFO]   Cmd timeout:  %CMD_TIMEOUT%s
echo [INFO]   Recordings:   %RECORDINGS_DIR%
if not "%VESSEL_NAME%"=="" echo [INFO]   Vessel:       %VESSEL_NAME%
echo [INFO]   Sensors:      always enabled (camera/lidar via headless observer)
echo [INFO]   Visualizer:   http://localhost:8899 (time histories, camera, point cloud, overlay)
if "%GPU_AVAILABLE%"=="true" (
    echo [INFO]   GPU:          NVIDIA GPU detected - passing through for hardware-accelerated rendering
) else (
    echo [INFO]   GPU:          none detected - observer uses CPU-only SwiftShader rendering
)
if "%OBSERVE_OTHERS%"=="true" echo [INFO]   Observe-others: enabled
echo.

REM Sensors (camera/lidar) are always enabled now, so these ports are always
REM needed. 9090/8899: rosbridge websocket + the local ROS2 topic visualizer.
set "SENSOR_PORTS=-p 7001-7095:7001-7095 -p 9090:9090 -p 8899:8899"

set "OBSERVE_ENV="
if "%OBSERVE_OTHERS%"=="true" set "OBSERVE_ENV=-e MAVSIM_OBSERVE_OTHERS=1"

if not "%TOKEN_FILE%"=="" (
    docker run --rm ^
        --name "%CONTAINER_NAME%" ^
        %GPU_ARGS% ^
        %SENSOR_PORTS% ^
        -e "ROS_DOMAIN_ID=%ROS_DOMAIN_ID_VAL%" ^
        -e "MAVSIM_CMD_TIMEOUT=%CMD_TIMEOUT%" ^
        %OBSERVE_ENV% ^
        -v "%RECORDINGS_DIR%:/tmp/mavsim_bags" ^
        -v "%BRIDGE_FILE%:/app/user_code/my_controller.py:ro" ^
        -v "%TOKEN_FILE%:/app/token.json:ro" ^
        -v "%CORE_DIR%\base_controller.py:/app/base_controller.py:ro" ^
        -v "%CORE_DIR%\python_controller.py:/app/python_controller.py:ro" ^
        -v "%CORE_DIR%\run_controller.py:/app/run_controller.py:ro" ^
        -v "%CORE_DIR%\recording_service.py:/app/recording_service.py:ro" ^
        -v "%CORE_DIR%\local_sensor_generator.py:/app/local_sensor_generator.py:ro" ^
        -v "%CORE_DIR%\observer.py:/app/observer.py:ro" ^
        -v "%CORE_DIR%\visualizer_server.py:/app/visualizer_server.py:ro" ^
        "%DOCKER_IMAGE%" ^
        --token /app/token.json ^
        %EXTRA_ARGS%
) else (
    docker run --rm ^
        --name "%CONTAINER_NAME%" ^
        %GPU_ARGS% ^
        %SENSOR_PORTS% ^
        -e "ROS_DOMAIN_ID=%ROS_DOMAIN_ID_VAL%" ^
        -e "MAVSIM_CMD_TIMEOUT=%CMD_TIMEOUT%" ^
        %OBSERVE_ENV% ^
        -v "%RECORDINGS_DIR%:/tmp/mavsim_bags" ^
        -v "%BRIDGE_FILE%:/app/user_code/my_controller.py:ro" ^
        -v "%CORE_DIR%\base_controller.py:/app/base_controller.py:ro" ^
        -v "%CORE_DIR%\python_controller.py:/app/python_controller.py:ro" ^
        -v "%CORE_DIR%\run_controller.py:/app/run_controller.py:ro" ^
        -v "%CORE_DIR%\recording_service.py:/app/recording_service.py:ro" ^
        -v "%CORE_DIR%\local_sensor_generator.py:/app/local_sensor_generator.py:ro" ^
        -v "%CORE_DIR%\observer.py:/app/observer.py:ro" ^
        -v "%CORE_DIR%\visualizer_server.py:/app/visualizer_server.py:ro" ^
        "%DOCKER_IMAGE%" ^
        --code "%CONTROLLER_CODE%" ^
        --backend-url "%CONTAINER_BACKEND_URL%" ^
        %EXTRA_ARGS%
)
exit /b %errorlevel%

REM ===========================================================
REM Helpers
REM ===========================================================

:rewrite_url
REM Replaces localhost / 127.0.0.1 with host.docker.internal for Docker Desktop
set "_url=%~1"
set "_url=!_url:localhost=host.docker.internal!"
set "_url=!_url:127.0.0.1=host.docker.internal!"
set "%~2=!_url!"
goto :eof

:parse_web_args
if "%~1"=="" goto :eof
if "%~1"=="--port" (
    set "WEBAPP_PORT=%~2"
    shift & shift
    goto :parse_web_args
)
if "%~1"=="--backend-url" (
    set "BACKEND_URL=%~2"
    shift & shift
    goto :parse_web_args
)
if "%~1"=="--frontend-url" (
    set "FRONTEND_URL=%~2"
    shift & shift
    goto :parse_web_args
)
if "%~1"=="--ros-domain-id" (
    set "ROS_DOMAIN_ID_VAL=%~2"
    shift & shift
    goto :parse_web_args
)
echo [WARN] Unknown argument: %~1 (ignored)
shift
goto :parse_web_args
