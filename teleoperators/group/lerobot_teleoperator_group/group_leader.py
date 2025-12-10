from __future__ import annotations
from dataclasses import dataclass
from typing import Any

import math

from lerobot.teleoperators import Teleoperator, make_teleoperator_from_config

from .config_group_leader import GroupLeaderConfig

class GroupLeader(Teleoperator):

    config_class = GroupLeaderConfig
    name = "group_leader"

    def __init__(self, config: GroupLeaderConfig):
        super().__init__(config)
        self.config = config

        self.teleops = {}
        for teleop_name, teleop_config in config.teleops.items():
            self.teleops[teleop_name] = make_teleoperator_from_config(teleop_config)
        self._is_connected = False
        self._is_calibrated = False

    @property
    def action_features(self) -> dict[str, type]:
        ft = {}
        for teleop in self.teleops.values():
            ft.update(teleop.action_features)
        return ft

    @property
    def feedback_features(self) -> dict[str, type]:
        ft = {}
        for teleop in self.teleops.values():
            ft.update(teleop.feedback_features)
        return ft

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return self._is_calibrated

    def connect(self, calibrate: bool = True) -> None:
        for teleop in self.teleops.values():
            teleop.connect()

        self._is_connected = True

    def calibrate(self) -> None:
        for teleop in self.teleops.values():
            teleop.calibrate()

    def configure(self) -> None:
        for teleop in self.teleops.values():
            teleop.configure()

    def disconnect(self) -> None:
        for teleop in self.teleops.values():
            teleop.disconnect()
        self._is_connected = False

    def get_action(self) -> dict[str, Any]:
        action = {}
        for teleop in self.teleops.values():
            action.update(teleop.get_action())
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        for teleop in self.teleops:
            teleop.send_feedback(feedback)

