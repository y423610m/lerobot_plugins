from __future__ import annotations

from typing import Any, Optional, Tuple
import threading
import asyncio
import inspect
import numpy as np
from functools import cached_property

from lerobot.cameras import make_cameras_from_configs
from lerobot.utils.errors import DeviceNotConnectedError, DeviceAlreadyConnectedError
from lerobot.robots import Robot, make_robot_from_config

from .config_group_follower import GroupFollowerConfig

class GroupFollower(Robot):

    config_class = GroupFollowerConfig
    name = "group_follower"

    def __init__(self, config: GroupFollowerConfig):
        super().__init__(config)
        self.config = config

        print(f"{config=}")

        self.cameras = make_cameras_from_configs(config.cameras)

        self.robots = {}
        for robot_name, robot_config in self.config.robots.items():
            self.robots[robot_name] = make_robot_from_config(robot_config)

        self._is_connected = False
        self._is_calibrated = False

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        for robot in self.robots.values():
            robot.connect()
        for cam in self.cameras.values():
            cam.connect()
        self._is_connected = True
        print(f"{self} connected.")

    def disconnect(self) -> None:
        if not self.is_connected:
            return
        for robot in self.robots.values():
            robot.connect()
        for cam in self.cameras.values():
            cam.disconnect()
        self._is_connected = False
        print(f"{self} disconnected.")

    def calibrate(self) -> None:
        for robot in self.robots.values():
            robot.calibate()
        self._is_calibrated = True

    def configure(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        for robot in self.robots.values():
            robot.configure()

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @is_connected.setter
    def is_connected(self, value: bool) -> None:
        self._is_connected = value

    @property
    def is_calibrated(self) -> bool:
        return self.is_connected

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return self._cam_shape

    @property
    def observation_features(self) -> dict[str, Any]:
        ft = {}
        for robot in self.robots.values():
            ft.update(robot.observation_features())
        return {**ft, **self._cameras_ft}

    @property
    def action_features(self) -> dict[str, type]:
        ft = {}
        for robot in self.robots.values():
            ft.update(robot.action_features)
        return ft

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        for robot in self.robots.values():
            robot.send_action(action)

        return action

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        obs: dict[str, Any] = {}
        for robot in self.robots.values():
            obs.update(robot.get_observation())

        for cam_key, cam in self.cameras.items():
            h, w, c = self._cam_shape[cam_key]
            frame = None
            try:
                if hasattr(cam, "async_read"):
                    frame = cam.async_read()
                elif hasattr(cam, "read"):
                    ok, frame = cam.read()
                    if ok is False:
                        frame = None
            except Exception:
                frame = None

            if frame is None or not isinstance(frame, np.ndarray):
                frame = np.zeros((h, w, c), dtype=np.uint8)

            obs[cam_key] = frame

        return obs