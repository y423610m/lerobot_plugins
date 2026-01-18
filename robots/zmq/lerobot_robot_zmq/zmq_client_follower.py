from __future__ import annotations

from functools import cached_property
from typing import Any, Optional, Tuple
import threading
import asyncio
import inspect
import numpy as np

from lerobot.cameras import make_cameras_from_configs
from lerobot.utils.errors import DeviceNotConnectedError, DeviceAlreadyConnectedError
from lerobot.robots import Robot, make_robot_from_config
from lerobot.utils.constants import ACTION, OBS_STATE

import base64
import json
import logging
from functools import cached_property
from typing import Any

import cv2
import numpy as np

from lerobot.utils.constants import ACTION, OBS_STATE
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .config_zmq_client_follower import ZMQClientFollowerConfig
import zmq

class ZMQClientFollower(Robot):

    config_class = ZMQClientFollowerConfig
    name = "zmq_client_follower"

    def __init__(self, config: ZMQClientFollowerConfig):

        self._is_connected = False
        self._is_calibrated = True

        self._position = 0.0

        import zmq

        self._zmq = zmq
        super().__init__(config)
        self.config = config
        self.cameras = make_cameras_from_configs(config.cameras)
        assert len(config.robots) == 1
        self.robot = make_robot_from_config(config.robots[list(config.robots.keys())[0]])

        self.remote_ip = config.remote_ip
        self.port_zmq_cmd = config.port_zmq_cmd
        self.port_zmq_observations = config.port_zmq_observations

        self.polling_timeout_ms = config.polling_timeout_ms
        self.connect_timeout_s = config.connect_timeout_s

        self.zmq_context = None
        self.zmq_cmd_socket = None
        self.zmq_observation_socket = None

        self.last_frames = {}

        self.last_remote_state = {}

    def connect(self, calibrate: bool = True) -> None:
        if self._is_connected:
            raise DeviceAlreadyConnectedError(
                "ZMQ Daemon is already connected. Do not run `robot.connect()` twice."
            )

        zmq = self._zmq
        self.zmq_context = zmq.Context()
        self.zmq_cmd_socket = self.zmq_context.socket(zmq.PUSH)
        zmq_cmd_locator = f"tcp://{self.remote_ip}:{self.port_zmq_cmd}"
        self.zmq_cmd_socket.connect(zmq_cmd_locator)
        self.zmq_cmd_socket.setsockopt(zmq.CONFLATE, 1)

        self.zmq_observation_socket = self.zmq_context.socket(zmq.PULL)
        zmq_observations_locator = f"tcp://{self.remote_ip}:{self.port_zmq_observations}"
        self.zmq_observation_socket.connect(zmq_observations_locator)
        self.zmq_observation_socket.setsockopt(zmq.CONFLATE, 1)

        poller = zmq.Poller()
        poller.register(self.zmq_observation_socket, zmq.POLLIN)
        socks = dict(poller.poll(self.connect_timeout_s * 1000))
        if self.zmq_observation_socket not in socks or socks[self.zmq_observation_socket] != zmq.POLLIN:
            raise DeviceNotConnectedError("Timeout waiting for ZMQ Host to connect expired.")

        self._is_connected = True
        print(f"{self} connected.")

    def disconnect(self):
        """Cleans ZMQ comms"""

        if not self._is_connected:
            raise DeviceNotConnectedError(
                "ZMQ is not connected. You need to run `robot.connect()` before disconnecting."
            )
        self.zmq_observation_socket.close()
        self.zmq_cmd_socket.close()
        self.zmq_context.term()
        self._is_connected = False

    def calibrate(self) -> None:
        self._is_calibrated = True
        # TODO call calibrate?
        return

    def configure(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

    @cached_property
    def _state_order(self) -> tuple[str, ...]:
        return tuple(self.observation_features.keys())

    @cached_property
    def _cameras_ft(self) -> dict[str, tuple[int, int, int]]:
        return {name: (cfg.height, cfg.width, 3) for name, cfg in self.config.cameras.items()}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @is_connected.setter
    def is_connected(self, value: bool) -> None:
        self._is_connected = value

    @property
    def is_calibrated(self) -> bool:
        return self._is_calibrated

    @property
    def observation_features(self) -> dict[str, Any]:
        return self.robot.observation_features

    @property
    def action_features(self) -> dict[str, type]:
        return self.robot.action_features

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Command ZMQ to move to a target joint configuration. Translates to motor space + sends over ZMQ

        Args:
            action (np.ndarray): array containing the goal positions for the motors.

        Raises:
            RobotDeviceNotConnectedError: if robot is not connected.

        Returns:
            np.ndarray: the action sent to the motors, potentially clipped.
        """
        if not self._is_connected:
            raise DeviceNotConnectedError(
                "ManipulatorRobot is not connected. You need to run `robot.connect()`."
            )

        self.zmq_cmd_socket.send_string(json.dumps(action))  # action is in motor space

        # TODO(Steven): Remove the np conversion when it is possible to record a non-numpy array value
        actions = np.array([action.get(k, 0.0) for k in self._state_order], dtype=np.float32)

        action_sent = {key: actions[i] for i, key in enumerate(self._state_order)}
        action_sent[ACTION] = actions
        return action_sent

    def get_observation(self) -> dict[str, Any]:
        """
        Capture observations from the remote robot: current follower arm positions,
        present wheel speeds (converted to body-frame velocities: x, y, theta),
        and a camera frame. Receives over ZMQ, translate to body-frame vel
        """
        if not self._is_connected:
            raise DeviceNotConnectedError("ZMQClient is not connected. You need to run `robot.connect()`.")

        frames, obs_dict = self._get_data()

        # Loop over each configured camera
        for cam_name, frame in frames.items():
            if frame is None:
                logging.warning("Frame is None")
                frame = np.zeros((640, 480, 3), dtype=np.uint8)
            obs_dict[cam_name] = frame

        return obs_dict

    def _poll_and_get_latest_message(self) -> str | None:
        """Polls the ZMQ socket for a limited time and returns the latest message string."""
        zmq = self._zmq
        poller = zmq.Poller()
        poller.register(self.zmq_observation_socket, zmq.POLLIN)

        try:
            socks = dict(poller.poll(self.polling_timeout_ms))
        except zmq.ZMQError as e:
            logging.error(f"ZMQ polling error: {e}")
            return None

        if self.zmq_observation_socket not in socks:
            logging.info("No new data available within timeout.")
            return None

        last_msg = None
        while True:
            try:
                msg = self.zmq_observation_socket.recv_string(zmq.NOBLOCK)
                last_msg = msg
            except zmq.Again:
                break

        if last_msg is None:
            logging.warning("Poller indicated data, but failed to retrieve message.")

        return last_msg

    def _parse_observation_json(self, obs_string: str) -> dict[str, Any] | None:
        """Parses the JSON observation string."""
        try:
            reveived_observation = json.loads(obs_string)
            return reveived_observation
        except json.JSONDecodeError as e:
            logging.error(f"Error decoding JSON observation: {e}")
            return None

    def _decode_image_from_b64(self, image_b64: str) -> np.ndarray | None:
        """Decodes a base64 encoded image string to an OpenCV image."""
        if not image_b64:
            return None
        try:
            jpg_data = base64.b64decode(image_b64)
            np_arr = np.frombuffer(jpg_data, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                logging.warning("cv2.imdecode returned None for an image.")
            return frame
        except (TypeError, ValueError) as e:
            logging.error(f"Error decoding base64 image data: {e}")
            return None

    def _remote_state_from_obs(
        self, observation: dict[str, Any]
    ) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        """Extracts frames, and state from the parsed observation."""

        flat_state = {key: observation.get(key, 0.0) for key in self._state_order}

        state_vec = np.array([flat_state[key] for key in self._state_order], dtype=np.float32)

        obs_dict: dict[str, Any] = {**flat_state, OBS_STATE: state_vec}

        # Decode images
        current_frames: dict[str, np.ndarray] = {}
        for cam_name, image_b64 in observation.items():
            if cam_name not in self._cameras_ft:
                continue
            frame = self._decode_image_from_b64(image_b64)
            if frame is not None:
                current_frames[cam_name] = frame

        return current_frames, obs_dict

    def _get_data(self) -> tuple[dict[str, np.ndarray], dict[str, Any], dict[str, Any]]:
        """
        Polls the video socket for the latest observation data.

        Attempts to retrieve and decode the latest message within a short timeout.
        If successful, updates and returns the new frames, speed, and arm state.
        If no new data arrives or decoding fails, returns the last known values.
        """

        # 1. Get the latest message string from the socket
        latest_message_str = self._poll_and_get_latest_message()

        # 2. If no message, return cached data
        if latest_message_str is None:
            return self.last_frames, self.last_remote_state

        # 3. Parse the JSON message
        observation = self._parse_observation_json(latest_message_str)

        # 4. If JSON parsing failed, return cached data
        if observation is None:
            return self.last_frames, self.last_remote_state

        # 5. Process the valid observation data
        try:
            new_frames, new_state = self._remote_state_from_obs(observation)
        except Exception as e:
            logging.error(f"Error processing observation data, serving last observation: {e}")
            return self.last_frames, self.last_remote_state

        self.last_frames = new_frames
        self.last_remote_state = new_state

        return new_frames, new_state
