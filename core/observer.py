#!/usr/bin/env python3
"""
Headless Sensor Observer (plans/plan_headless_observer.md).

Launches a headless Chromium tab pointed at the MAVSim frontend's Simulation
page for a specific session, purely to trigger the existing browser-side
rendering + SensorStreamManager pipeline automatically - so camera/lidar
data flows to the local sensor bridge even when no human has a tab open.

Runs as a subprocess of base_controller.py, in the same container as the
bridge (see plan doc for why: this reuses 100% of the existing rendering
pipeline unmodified, and avoids a second Docker image/container).

Uses a real GPU when one is available (see _detect_gpu_available() -
NVIDIA only for now, passed through by user_repo_new/start.sh via
`docker run --gpus all`), falling back to software WebGL (SwiftShader)
otherwise. CPU-only SwiftShader is a known throughput ceiling, not just a
slowness inconvenience: under real-world load (multiple vessels, multiple
camera/lidar sensors, heavy post-processing like rain/fog) the renderer can
become genuinely unresponsive - confirmed by live debugging (py-spy stack
dumps showed the bridge's own Python process fully idle while the browser
tab stopped producing any output, including console logs, and Chromium's
own network stack eventually dropped the sensor WebSocket connections from
its side). Even with a GPU, this script still runs a watchdog as a safety
net: if the tab goes quiet for too long, it's presumed stuck and gets torn
down and relaunched fresh against the same session, rather than leaving
sensor streaming dead for the rest of the session.
"""
import argparse
import logging
import signal
import sys
import time

logger = logging.getLogger("observer")
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - observer - %(levelname)s - %(message)s",
)

# Common to both render paths: the frontend's camera capture loop already
# uses setInterval (not requestAnimationFrame) specifically to survive
# backgrounded tabs, but Chromium still throttles JS timers at the engine
# level for tabs it considers occluded/backgrounded - these flags disable
# that throttling, which otherwise silently caps headless capture FPS well
# below what a real, focused browser tab achieves.
_CHROMIUM_ARGS_COMMON = [
    "--disable-background-timer-throttling",
    "--disable-backgrounding-occluded-windows",
    "--disable-renderer-backgrounding",
]

# Flags for headless WebGL without a real GPU. Without these, Chromium's
# WebGL context creation can silently fail and produce blank frames instead
# of an error - CameraSensor.js/SceneManager.js create real
# THREE.WebGLRenderer contexts, so this is the single biggest risk in this
# design (see plan doc section 3) and the first thing to verify in testing.
_CHROMIUM_ARGS_SWIFTSHADER = _CHROMIUM_ARGS_COMMON + [
    "--no-sandbox",
    "--disable-dev-shm-usage",
    "--use-gl=swiftshader",
    "--use-angle=swiftshader-webgl",
    "--enable-webgl",
    "--enable-unsafe-swiftshader",
    "--ignore-gpu-blocklist",
]

# Flags for real hardware-accelerated WebGL (NVIDIA passthrough via
# `docker run --gpus all`, see user_repo_new/start.sh). Empirically verified
# against a real Quadro RTX 8000 (confirmed via
# WEBGL_debug_renderer_info/UNMASKED_RENDERER_WEBGL reporting the real GPU,
# not SwiftShader) - two things were not obvious going in and both matter:
#   - "--headless=new" is required, not just preferred - the legacy headless
#     mode never had proper GPU rendering support.
#   - "--use-angle=gl" (ANGLE's desktop-GL backend) requires an X11 display
#     (it uses GLX) and fails outright in this ozone-headless container
#     ("Could not open the default X display"). "--use-angle=gl-egl" uses
#     EGL instead of GLX for the same desktop-GL backend and works headless
#     with no X server. This also needs libegl1 installed in the image
#     (examples/Dockerfile) - the NVIDIA vendor EGL library the container
#     toolkit injects has nothing to dlopen() it without the system EGL
#     loader; without it ANGLE silently falls back to its own bundled
#     SwiftShader instead of erroring.
_CHROMIUM_ARGS_GPU = _CHROMIUM_ARGS_COMMON + [
    "--headless=new",
    "--no-sandbox",
    "--use-gl=angle",
    "--use-angle=gl-egl",
    "--enable-gpu-rasterization",
    "--ignore-gpu-blocklist",
]


def _detect_gpu_available() -> bool:
    """
    Best-effort check for whether this container actually got NVIDIA GPU
    access (as opposed to just being asked for it) - independent of
    whatever start.sh guessed on the host side, so a mismatch (e.g. the
    nvidia-container-toolkit installed but misconfigured) fails safe to
    SwiftShader instead of Chromium crashing on GPU flags it can't use.

    Deliberately checks only /dev/nvidia* device files, not
    /proc/driver/nvidia/version - empirically confirmed that procfs entry
    reflects host-wide kernel module state and is visible in every
    container regardless of whether `--gpus` was passed, while /dev
    device nodes are correctly gated by Docker's per-container device
    cgroup and only appear when GPU access was actually granted.
    """
    import os
    return os.path.exists("/dev/nvidiactl")

_NAV_MAX_ATTEMPTS = 5
_NAV_RETRY_DELAY_SECONDS = 3

# Watchdog: restart the browser if the renderer stops making progress.
#
# Liveness is measured by polling a requestAnimationFrame counter installed
# in the page, NOT by watching console output. Console chatter is only a
# proxy for "the app is logging", which is a different thing from "the
# renderer is running": the frontend logs heavily while building the scene
# and then goes quiet once it settles into steady-state rendering, so a
# console-based watchdog reads a perfectly healthy tab as stuck.
#
# That is not hypothetical - it was the observed failure. Measured against a
# live two-vessel session on a Quadro RTX 8000 (hardware WebGL confirmed via
# UNMASKED_RENDERER_WEBGL, so not a SwiftShader effect), an observer tab left
# to run without a watchdog reported:
#
#   t=10s  ~60 fps   console quiet   0s
#   t=20s   ~5 fps   console quiet  59s   <- old watchdog fired here
#   t=30s  ~49 fps   console quiet  69s   <- and here
#   t=40s  ~53 fps   console quiet   0s
#   t=60s  ~53 fps   console quiet  20s
#
# The tab dips during scene/BVH construction, goes quiet, and then recovers
# to a steady ~53 fps. The old watchdog killed it mid-dip every time, so the
# session never reached steady state and camera/lidar frames never flowed -
# the browser was restarted ~once a minute indefinitely.
_WATCHDOG_TIMEOUT_SECONDS = 45
_WATCHDOG_CHECK_INTERVAL_SECONDS = 5
# Frames that must accumulate between two checks for the renderer to count as
# alive. The dip above bottomed out near 5 fps (~25 frames per 5s check), so
# a single-digit threshold distinguishes "slow but progressing" from "stopped"
# without re-introducing the false positive this replaced.
_WATCHDOG_MIN_FRAMES = 2
# Grace period after navigation before the watchdog arms at all: initial
# scene load, terrain simplification and BVH construction are legitimately
# long and partly synchronous, and must not be mistaken for a hang.
_WATCHDOG_STARTUP_GRACE_SECONDS = 90
# Evaluating in a busy page can itself block; treat a slow probe as "no
# answer this round" rather than letting it wedge the watchdog thread.
_WATCHDOG_PROBE_TIMEOUT_MS = 5000
# Best-effort graceful close before falling back to a hard kill of the
# browser's OS process - a stuck renderer can make even close() hang.
_BROWSER_CLOSE_TIMEOUT_MS = 5000
_RESTART_DELAY_SECONDS = 2


def parse_args():
    parser = argparse.ArgumentParser(description="MAVSim headless sensor observer")
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--api-token", required=True)
    parser.add_argument("--namespace", default="", help="Informational only, used for logging")
    parser.add_argument(
        "--frontend-url", required=True,
        help="Where the MAVSim frontend is actually hosted, e.g. http://<server-ip>:5173 "
             "(never assumed to be localhost - see plan doc section 2)",
    )
    return parser.parse_args()


def _run_observer_session(playwright, url, is_running, chromium_args):
    """
    Launch one browser, navigate to the observer URL, and run until the
    tab goes quiet for too long (watchdog trip) or is_running() becomes
    False (SIGTERM). Always closes the browser before returning.

    Returns "stuck" if the watchdog tripped (caller should relaunch),
    or "stopped" if is_running() went False (caller should exit).
    """
    browser = playwright.chromium.launch(headless=True, args=chromium_args)
    try:
        # "local-network-access" is required whenever the frontend is served
        # from a public origin (e.g. https://mavsim.mavlab.in). Chromium's
        # Local Network Access checks (enforced by default since ~v142)
        # refuse any connection from a public page to a loopback address
        # unless the user grants a permission prompt - and in headless there
        # is nobody to click "Allow", so every ws://127.0.0.1:<port> to the
        # sensor bridge failed with net::ERR_BLOCKED_BY_LOCAL_NETWORK_ACCESS_CHECKS
        # and no camera/lidar data ever left the tab. Pages served from a
        # local origin (http://localhost:5173, a LAN IP) are exempt, which is
        # why this only ever showed up against the cloud deployment.
        #
        # Granting the permission is deliberately preferred over
        # `--disable-features=LocalNetworkAccessChecks`: Playwright appends
        # our args after its own `--disable-features=...` list and Chromium
        # keeps only the last such flag, so that route would silently drop
        # every feature Playwright disables for stability.
        context = browser.new_context(
            viewport={"width": 1280, "height": 720},
            permissions=["local-network-access"],
        )
        page = context.new_page()
        # Console output is forwarded for diagnostics only - it is no longer
        # the watchdog's liveness signal (see _WATCHDOG_TIMEOUT_SECONDS).
        page.on("console", lambda msg: logger.info(f"[browser console] {msg.type}: {msg.text}"))
        page.on("pageerror", lambda exc: logger.warning(f"[browser page error] {exc}"))
        # Web Worker console output (e.g. LidarStreamWorker.js) isn't
        # surfaced via the page-level console listener - Workers only
        # support "console" and "close" events, not "pageerror".
        page.on("worker", lambda w: (
            logger.info(f"[worker created] {w.url}"),
            w.on("console", lambda msg: logger.info(f"[worker console] {msg.type}: {msg.text}")),
            w.on("close", lambda w2: logger.warning(f"[worker closed] {w2.url}")),
        ))

        loaded = False
        for attempt in range(1, _NAV_MAX_ATTEMPTS + 1):
            try:
                logger.info(f"Navigating to observer URL (attempt {attempt}/{_NAV_MAX_ATTEMPTS})...")
                page.goto(url, wait_until="load", timeout=30000)
                loaded = True
                logger.info("Observer page loaded - sensor streaming should now be active")
                break
            except Exception as e:
                logger.warning(f"Navigation attempt {attempt} failed: {e}")
                if attempt < _NAV_MAX_ATTEMPTS:
                    time.sleep(_NAV_RETRY_DELAY_SECONDS)

        if not loaded:
            logger.error(
                f"Could not load the observer page after {_NAV_MAX_ATTEMPTS} attempts - "
                f"is the frontend reachable from this container?"
            )
            return "stuck"

        # Install the liveness counter the watchdog polls. Kept deliberately
        # tiny and self-contained: it must not depend on any app internals,
        # so it keeps answering even if the frontend's own state machine is
        # wedged - that distinction is the whole point of the check.
        _install_liveness_counter(page)

        loaded_at = time.time()
        last_progress = time.time()
        last_frames = -1
        while is_running():
            time.sleep(_WATCHDOG_CHECK_INTERVAL_SECONDS)

            frames = _read_frame_count(page)
            now = time.time()

            if frames is None:
                # Probe failed (page busy, navigating, or context torn down).
                # Not evidence of a hang on its own; let the timeout below
                # decide if this keeps up.
                pass
            else:
                if last_frames < 0 or frames - last_frames >= _WATCHDOG_MIN_FRAMES:
                    last_progress = now
                # A reload resets the counter; treat a decrease as progress
                # rather than as a stall.
                if frames < last_frames:
                    last_progress = now
                last_frames = frames

            if now - loaded_at < _WATCHDOG_STARTUP_GRACE_SECONDS:
                continue

            stalled_for = now - last_progress
            if stalled_for > _WATCHDOG_TIMEOUT_SECONDS:
                logger.warning(
                    f"Renderer produced no frames for {stalled_for:.0f}s "
                    f"(>{_WATCHDOG_TIMEOUT_SECONDS}s threshold) - presumed stuck. "
                    "Restarting the observer browser."
                )
                return "stuck"
        return "stopped"
    finally:
        _close_browser(browser)


def _install_liveness_counter(page):
    """Install a requestAnimationFrame counter used as the watchdog's
    liveness signal, and re-install it after any navigation (a reload
    discards it along with the rest of the page's JS state)."""
    script = """() => {
      if (window.__observerFrames !== undefined) return;
      window.__observerFrames = 0;
      const tick = () => { window.__observerFrames++; requestAnimationFrame(tick); };
      requestAnimationFrame(tick);
    }"""
    try:
        page.evaluate(script)
    except Exception as e:
        logger.debug(f"Could not install liveness counter: {e}")
    # add_init_script runs on every subsequent document, so an in-page
    # route change or reload doesn't leave the watchdog blind.
    try:
        page.add_init_script(
            "window.__observerFrames = 0;"
            "(function t(){ window.__observerFrames++; requestAnimationFrame(t); })();"
        )
    except Exception as e:
        logger.debug(f"Could not register liveness init script: {e}")


def _read_frame_count(page):
    """Read the liveness counter, returning None if the page can't answer
    in time. A busy main thread can make evaluate() slow, which must not
    be conflated with the renderer having stopped."""
    try:
        # A stuck renderer can make evaluate() hang, not just return slowly;
        # without this the watchdog thread would block on the very condition
        # it exists to detect.
        page.set_default_timeout(_WATCHDOG_PROBE_TIMEOUT_MS)
        value = page.evaluate(
            "() => window.__observerFrames === undefined ? null : window.__observerFrames"
        )
    except Exception as e:
        logger.debug(f"Liveness probe failed: {e}")
        return None
    if value is None:
        # Counter missing (fresh document after a navigation) - reinstall it
        # so the next round has something to read.
        _install_liveness_counter(page)
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _close_browser(browser):
    """Best-effort graceful close, falling back to killing any leftover
    Chromium OS processes if the renderer is too stuck to even respond to
    close() itself (close() waits on the browser's own protocol, so a
    hung renderer can make it hang too, not just raise). This container
    runs a single dedicated observer instance, so a broad pkill here is
    safe - there is nothing else Chromium-related for it to affect."""
    import threading

    close_done = threading.Event()

    def _do_close():
        try:
            browser.close()
        except Exception as e:
            logger.debug(f"browser.close() raised: {e}")
        finally:
            close_done.set()

    t = threading.Thread(target=_do_close, daemon=True)
    t.start()
    if not close_done.wait(timeout=_BROWSER_CLOSE_TIMEOUT_MS / 1000):
        logger.warning(
            f"browser.close() did not complete within {_BROWSER_CLOSE_TIMEOUT_MS}ms "
            "(renderer too stuck to respond) - hard-killing leftover Chromium processes"
        )
        _hard_kill_leftover_chromium()


def _hard_kill_leftover_chromium():
    """Last-resort cleanup when graceful close() itself hangs or errors -
    so a stuck renderer can't leak indefinitely across restarts."""
    import subprocess
    try:
        subprocess.run(["pkill", "-9", "-f", "chrome-headless-shell"], timeout=5)
    except Exception as e:
        logger.debug(f"Hard-kill fallback also failed (process may already be gone): {e}")


def main():
    args = parse_args()
    base_url = args.frontend_url.rstrip("/")
    url = f"{base_url}/#/simulation?observer=1&session_id={args.session_id}&api_token={args.api_token}"

    logger.info(
        f"Starting headless observer: session={args.session_id} "
        f"namespace={args.namespace or '(unset)'} frontend={base_url}"
    )

    if _detect_gpu_available():
        logger.info("GPU passthrough detected - using hardware-accelerated rendering")
        chromium_args = _CHROMIUM_ARGS_GPU
    else:
        logger.info("No GPU access - using SwiftShader software rendering")
        chromium_args = _CHROMIUM_ARGS_SWIFTSHADER

    try:
        from playwright.sync_api import sync_playwright
    except ImportError:
        logger.error(
            "playwright is not installed - was `playwright install --with-deps chromium` "
            "run when this image was built?"
        )
        sys.exit(1)

    running = True

    def _handle_sigterm(signum, frame):
        nonlocal running
        logger.info("Received SIGTERM, shutting down observer browser...")
        running = False

    signal.signal(signal.SIGTERM, _handle_sigterm)

    restart_count = 0
    with sync_playwright() as p:
        while running:
            if restart_count > 0:
                logger.info(f"Restarting observer browser (restart #{restart_count})...")
                time.sleep(_RESTART_DELAY_SECONDS)
            result = _run_observer_session(p, url, lambda: running, chromium_args)
            if result == "stopped":
                break
            restart_count += 1


if __name__ == "__main__":
    main()
