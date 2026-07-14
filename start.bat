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
REM   CLI mode (default): start.bat <controller-code> [options...]
REM   Token mode:         start.bat --token <path-to-token.json> [options...]
REM   Web mode:           start.bat --mode web [options...]
REM
REM Usage:
REM   start.bat <controller-code> [--vessel-name NAME] [--backend-url URL]
REM             [--frontend-url URL] [--rate HZ] [--ros-domain-id N]
REM             [--cmd-timeout SEC] [--observe-others]
REM   start.bat --token <path-to-token.json> [options...]
REM   start.bat --mode web [--port 8888] [--backend-url URL] [--frontend-url URL] [--ros-domain-id N]
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

set "DEFAULT_BACKEND_URL=http://localhost:5000"
set "DEFAULT_FRONTEND_URL=http://localhost:5173"
set "DEFAULT_WEBAPP_PORT=8888"
set "RECORDINGS_DIR=%SCRIPT_DIR%\recordings"
set "BRIDGE_FILE=%SCRIPT_DIR%\bridge_controller.py"
set "WEBAPP_FILE=%SCRIPT_DIR%\bridge_webapp.py"

REM ---- Detect mode ----
set "MODE=cli"
set "WEBAPP_PORT=%DEFAULT_WEBAPP_PORT%"
REM Deliberately NOT 0 - see the ros-domain-id note near the top of this file.
set "ROS_DOMAIN_ID_VAL=42"

if "%~1"=="--mode" (
    if "%~2"=="" (
        echo [ERROR] --mode requires a value: web or cli
        exit /b 1
    )
    set "MODE=%~2"
    shift & shift
)

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

docker image inspect "%DOCKER_IMAGE%" >nul 2>&1
if errorlevel 1 (
    echo [INFO] Pulling %DOCKER_IMAGE% ...
    docker pull "%DOCKER_IMAGE%"
    if errorlevel 1 (
        echo [ERROR] Failed to pull image
        exit /b 1
    )
)

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
    echo.

    set "WEB_CMD=source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && cd /app && exec python3 /app/user_code/bridge_webapp.py --port '!WEBAPP_PORT!' --backend-url '!CONTAINER_BACKEND_URL!' --frontend-url '!CONTAINER_FRONTEND_URL!'"

    docker run --rm ^
        --name "%CONTAINER_NAME%" ^
        -p "!WEBAPP_PORT!:!WEBAPP_PORT!" ^
        -p "7001-7095:7001-7095" ^
        -p "9090:9090" ^
        -p "8899:8899" ^
        -e "ROS_DOMAIN_ID=!ROS_DOMAIN_ID_VAL!" ^
        -v "%RECORDINGS_DIR%:/tmp/mavsim_bags" ^
        -v "%BRIDGE_FILE%:/app/user_code/my_controller.py:ro" ^
        -v "%WEBAPP_FILE%:/app/user_code/bridge_webapp.py:ro" ^
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
        %SENSOR_PORTS% ^
        -e "ROS_DOMAIN_ID=%ROS_DOMAIN_ID_VAL%" ^
        -e "MAVSIM_CMD_TIMEOUT=%CMD_TIMEOUT%" ^
        %OBSERVE_ENV% ^
        -v "%RECORDINGS_DIR%:/tmp/mavsim_bags" ^
        -v "%BRIDGE_FILE%:/app/user_code/my_controller.py:ro" ^
        -v "%TOKEN_FILE%:/app/token.json:ro" ^
        "%DOCKER_IMAGE%" ^
        --token /app/token.json ^
        %EXTRA_ARGS%
) else (
    docker run --rm ^
        --name "%CONTAINER_NAME%" ^
        %SENSOR_PORTS% ^
        -e "ROS_DOMAIN_ID=%ROS_DOMAIN_ID_VAL%" ^
        -e "MAVSIM_CMD_TIMEOUT=%CMD_TIMEOUT%" ^
        %OBSERVE_ENV% ^
        -v "%RECORDINGS_DIR%:/tmp/mavsim_bags" ^
        -v "%BRIDGE_FILE%:/app/user_code/my_controller.py:ro" ^
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
