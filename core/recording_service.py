#!/usr/bin/env python3
"""
ROS2 Bag Recording Service for mavsim

This service handles recording ROS2 topics to bag files in MCAP format.
Requires ROS2 Humble and rosbag2.

Author: mavsim Team
License: MIT
"""

import logging
import os
import re
import subprocess
import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional

logger = logging.getLogger(__name__)


class RecordingService:
    """
    Service for recording ROS2 topics to bag files in MCAP format.
    
    This service uses rosbag2 to record topics. If topics=None, it discovers
    all topics starting with the namespace prefix.
    """
    
    def __init__(self, bag_dir: str = '/tmp/mavsim_bags'):
        """
        Initialize recording service.
        
        Args:
            bag_dir: Directory to store bag files (default: /tmp/mavsim_bags)
        """
        self.bag_dir = Path(bag_dir)
        self.bag_dir.mkdir(parents=True, exist_ok=True)
        
        self._recording = False
        self._bag_path: Optional[Path] = None
        self._topics: List[str] = []
        self._namespace: Optional[str] = None
        
        logger.info(f"RecordingService initialized: bag_dir={bag_dir}")
    
    def discover_namespace_topics(self, namespace: str) -> List[str]:
        """
        Discover all ROS2 topics starting with namespace prefix.
        
        Args:
            namespace: ROS namespace (e.g., '/sim_abc123/')
            
        Returns:
            List of topic names
        """
        try:
            # Run ros2 topic list command
            # Ensure ROS2 environment is available
            env = os.environ.copy()
            # ROS2 should be sourced by entrypoint, but ensure PATH is set
            if '/opt/ros/humble/bin' not in env.get('PATH', ''):
                env['PATH'] = '/opt/ros/humble/bin:' + env.get('PATH', '')
            
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=5.0,
                env=env
            )
            
            if result.returncode != 0:
                logger.error(f"Failed to list topics: {result.stderr}")
                return []
            
            # Filter topics by namespace
            all_topics = result.stdout.strip().split('\n')
            namespace_prefix = namespace.rstrip('/')
            
            matching_topics = [
                topic.strip()
                for topic in all_topics
                if topic.strip().startswith(namespace_prefix)
            ]
            
            logger.info(f"Discovered {len(matching_topics)} topics for namespace {namespace}")
            return matching_topics
            
        except subprocess.TimeoutExpired:
            logger.error("Topic discovery timed out")
            return []
        except Exception as e:
            logger.error(f"Failed to discover topics: {e}", exc_info=True)
            return []
    
    @staticmethod
    def _build_namespace_regex(namespaces: List[str]) -> str:
        """Build a regex matching a namespace prefix and any of its subtopics.

        Mirrors discover_namespace_topics()'s startswith() semantics (matches
        the namespace itself or anything below it) but as a live regex that
        `ros2 bag record --regex` re-evaluates against the ROS graph on an
        interval, so topics created *after* recording starts (e.g. a camera
        publisher that only spins up once the browser sends its first frame)
        are still picked up. A static topic list captured once at start time
        cannot do this -- see start_recording()'s topics-is-None branch below.
        """
        alternatives = []
        for ns in namespaces:
            prefix = ns.rstrip('/')
            if not prefix:
                continue
            alternatives.append(f"{re.escape(prefix)}(/.*)?")
        return f"^({'|'.join(alternatives)})$" if alternatives else ''

    def start_recording(
        self,
        namespace: str,
        topics: Optional[List[str]] = None,
        bag_name: Optional[str] = None,
        namespaces: Optional[List[str]] = None,
    ) -> bool:
        """
        Start recording ROS2 topics to bag file.

        Args:
            namespace: ROS namespace (e.g., '/vessel_01'). Used as the single
                       namespace when `namespaces` is not given, and always
                       recorded for status/logging purposes.
            topics: Explicit list of topics to record. If given, recording is
                    locked to exactly this list (legacy/explicit mode).
            bag_name: Optional bag name (default: auto-generated timestamp)
            namespaces: One or more namespace prefixes to record everything
                        under (e.g. multiple vessels). Only used when `topics`
                        is None. Defaults to `[namespace]`.

        Returns:
            True if recording started successfully
        """
        if self._recording:
            logger.warning("Recording already in progress")
            return False

        self._namespace = namespace

        use_regex = topics is None
        regex_pattern = None

        if use_regex:
            prefixes = namespaces if namespaces else [namespace]
            regex_pattern = self._build_namespace_regex(prefixes)
            if not regex_pattern:
                logger.error("No valid namespace prefix available for recording")
                return False

            # Discover currently-visible topics purely for a sanity check and
            # for status/logging -- the actual `ros2 bag record` invocation
            # uses --regex below so it keeps discovering matching topics for
            # the lifetime of the recording, not just this instant.
            discovered = []
            for prefix in prefixes:
                discovered.extend(self.discover_namespace_topics(prefix))
            if not discovered:
                logger.warning(
                    f"No topics currently visible for namespace(s) {prefixes} "
                    "(will retry -- regex recording still requires at least "
                    "one topic to exist at start)"
                )
                return False
            self._topics = discovered
        else:
            if not topics:
                logger.warning("start_recording called with an empty explicit topic list")
                return False
            self._topics = topics

        # Generate bag name if not provided
        if bag_name is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            bag_name = f"mavsim_{timestamp}"

        self._bag_path = self.bag_dir / bag_name

        # Start rosbag2 recording
        try:
            # -o pins the output to self._bag_path explicitly - without it,
            # ros2 bag record silently auto-generates its own directory name
            # (rosbag2_<timestamp>/) instead, which stop_recording() below
            # has no way to find (its fallback glob searches for this
            # object's own "mavsim_<timestamp>" naming, not ros2's).
            if use_regex:
                # --regex is re-evaluated against the live ROS graph, so
                # topics that appear after this point (e.g. a camera
                # publisher created lazily on first frame) are still caught.
                cmd = ['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(self._bag_path), '--regex', regex_pattern]
            else:
                # Explicit topic list: fixed at start time, matches legacy
                # behaviour for callers that already know exactly what they
                # want recorded.
                cmd = ['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(self._bag_path)] + self._topics

            # Set output directory and ensure ROS2 environment
            env = os.environ.copy()
            env['ROS_BAG_DIR'] = str(self.bag_dir)
            # Ensure ROS2 is in PATH
            if '/opt/ros/humble/bin' not in env.get('PATH', ''):
                env['PATH'] = '/opt/ros/humble/bin:' + env.get('PATH', '')

            # Start rosbag2 process
            self._bag_process = subprocess.Popen(
                cmd,
                cwd=str(self.bag_dir),
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )

            # Wait a moment to check if process started successfully
            time.sleep(0.5)
            if self._bag_process.poll() is not None:
                # Process exited immediately (error)
                stdout, stderr = self._bag_process.communicate()
                logger.error(f"rosbag2 failed to start: {stderr.decode()}")
                return False

            self._recording = True
            logger.info(
                f"Recording started: bag={self._bag_path}, "
                f"mode={'regex' if use_regex else 'explicit'}, "
                f"topics_at_start={len(self._topics)}, format=MCAP"
            )
            return True

        except Exception as e:
            logger.error(f"Failed to start recording: {e}", exc_info=True)
            return False
    
    def stop_recording(self) -> Optional[str]:
        """
        Stop recording and close bag file.
        
        Returns:
            Path to bag file, or None if error
        """
        if not self._recording:
            logger.warning("No recording in progress")
            return None
        
        try:
            # Stop rosbag2 process
            if hasattr(self, '_bag_process') and self._bag_process:
                self._bag_process.terminate()
                try:
                    self._bag_process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    logger.warning("rosbag2 did not stop gracefully, killing...")
                    self._bag_process.kill()
                    self._bag_process.wait()
            
            self._recording = False
            
            # Find the actual bag file (rosbag2 may have added suffix)
            bag_path_str = str(self._bag_path) if self._bag_path else None
            
            if bag_path_str and Path(bag_path_str).exists():
                logger.info(f"Recording stopped: bag={bag_path_str}")
                return bag_path_str
            else:
                # Try to find bag file with .mcap extension
                bag_files = list(self.bag_dir.glob(f"{self._bag_path.name}*.mcap"))
                if bag_files:
                    bag_path_str = str(bag_files[0])
                    logger.info(f"Recording stopped: bag={bag_path_str}")
                    return bag_path_str
                else:
                    logger.warning("Could not find bag file after recording")
                    return None
                    
        except Exception as e:
            logger.error(f"Failed to stop recording: {e}", exc_info=True)
            return None
    
    def is_recording(self) -> bool:
        """Check if recording is in progress."""
        return self._recording
    
    def get_bag_path(self) -> Optional[str]:
        """Get path to current bag file."""
        return str(self._bag_path) if self._bag_path else None
    
    def get_topics(self) -> List[str]:
        """Get list of topics being recorded."""
        return self._topics.copy()


def upload_bag(
    bag_path: str,
    backend_url: str,
    session_id: str,
    api_token: str,
) -> bool:
    """
    Upload a recorded bag file to the cloud backend via presigned S3 URL.

    Steps:
      1. Request a presigned PUT URL from POST /api/bags/upload-url
      2. PUT the bag file directly to S3
      3. Confirm the upload via POST /api/bags/confirm-upload

    Args:
        bag_path: Local path to the bag file or directory
        backend_url: Web platform backend URL
        session_id: Session UUID
        api_token: Session API token

    Returns:
        True if upload succeeded
    """
    import requests

    bag = Path(bag_path)
    if not bag.exists():
        logger.error(f"Bag path does not exist: {bag_path}")
        return False

    # If bag_path is a directory, find the first .mcap file inside it
    if bag.is_dir():
        mcap_files = list(bag.glob("*.mcap"))
        if not mcap_files:
            # Also search rosbag2-style: metadata.yaml + *.db3/*.mcap
            mcap_files = list(bag.glob("**/*.mcap"))
        if not mcap_files:
            logger.error(f"No .mcap files found in bag directory: {bag_path}")
            return False
        bag = mcap_files[0]

    filename = bag.name
    file_size = bag.stat().st_size

    logger.info(f"Uploading bag: {bag} ({file_size} bytes) for session {session_id}")

    # Step 1: Get presigned upload URL
    try:
        resp = requests.post(
            f"{backend_url}/api/bags/upload-url",
            json={
                "sessionId": session_id,
                "apiToken": api_token,
                "filename": filename,
                "contentType": "application/octet-stream",
            },
            timeout=15,
        )
        if resp.status_code != 200:
            logger.error(f"Failed to get upload URL: {resp.status_code} {resp.text}")
            return False
        url_data = resp.json()
    except requests.RequestException as e:
        logger.error(f"Request error getting upload URL: {e}")
        return False

    upload_url = url_data.get("uploadUrl")
    s3_key = url_data.get("s3Key")
    if not upload_url or not s3_key:
        logger.error("Invalid response from upload-url endpoint")
        return False

    # Step 2: PUT the file to S3
    try:
        with open(bag, "rb") as f:
            put_resp = requests.put(
                upload_url,
                data=f,
                headers={"Content-Type": "application/octet-stream"},
                timeout=300,
            )
        if put_resp.status_code not in (200, 201, 204):
            logger.error(f"S3 PUT failed: {put_resp.status_code} {put_resp.text[:200]}")
            return False
        logger.info(f"Uploaded bag to S3: {s3_key}")
    except requests.RequestException as e:
        logger.error(f"S3 PUT error: {e}")
        return False

    # Step 3: Confirm upload
    try:
        confirm_resp = requests.post(
            f"{backend_url}/api/bags/confirm-upload",
            json={
                "sessionId": session_id,
                "apiToken": api_token,
                "s3Key": s3_key,
                "sizeBytes": file_size,
            },
            timeout=15,
        )
        if confirm_resp.status_code != 200:
            logger.error(f"Confirm upload failed: {confirm_resp.status_code} {confirm_resp.text}")
            return False
        logger.info(f"Bag upload confirmed for session {session_id}")
        return True
    except requests.RequestException as e:
        logger.error(f"Confirm upload error: {e}")
        return False

