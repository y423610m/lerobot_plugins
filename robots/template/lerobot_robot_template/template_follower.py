from __future__ import annotations

from functools import cached_property
from typing import Any, Optional, Tuple
import threading
import asyncio
import inspect
import numpy as np

from lerobot.cameras import make_cameras_from_configs
from lerobot.utils.errors import DeviceNotConnectedError, DeviceAlreadyConnectedError
from lerobot.robots.robot import Robot

from .config_template_follower import TemplateFollowerConfig

class TemplateFollower(Robot):

    config_class = TemplateFollowerConfig
    name = "template_follower"

    def __init__(self, config: TemplateFollowerConfig):
        super().__init__(config)
        self.config = config

        self.cameras = make_cameras_from_configs(config.cameras)

        self._is_connected = False
        self._is_calibrated = True

        self._position = 0.0


    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        for cam in self.cameras.values():
            cam.connect()

        self._is_connected = True
        print(f"{self} connected.")

    def disconnect(self) -> None:
        if not self.is_connected:
            return
        for cam in self.cameras.values():
            cam.disconnect()
        self._is_connected = False
        print(f"{self} disconnected.")

    def calibrate(self) -> None:
        return

    def configure(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @is_connected.setter
    def is_connected(self, value: bool) -> None:
        self._is_connected = value

    @property
    def is_calibrated(self) -> bool:
        return self.is_connected

    @cached_property
    def _cameras_ft(self) -> dict[str, tuple[int, int, int]]:
        return {name: (cfg.height, cfg.width, 3) for name, cfg in self.config.cameras.items()}

    @property
    def observation_features(self) -> dict[str, Any]:
        feats: dict[str, Any] = {"position": float}
        feats.update(self._cameras_ft)
        return feats

    @property
    def action_features(self) -> dict[str, type]:
        return {"position": float}

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        target = float(action.get("position", 0.0))

        self._position = target
        print(f"follower {target=}")
        return {"position": target}

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        obs: dict[str, Any] = {"position": float(self._position)}

        for cam_key, cam in self.cameras.items():
            h, w, c = self._cameras_ft[cam_key]
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