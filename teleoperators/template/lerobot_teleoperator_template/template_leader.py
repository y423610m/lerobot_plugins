from __future__ import annotations
from dataclasses import dataclass
from typing import Any

import math

from lerobot.teleoperators.teleoperator import Teleoperator

from .config_template_leader import TemplateLeaderConfig

class TemplateLeader(Teleoperator):

    config_class = TemplateLeaderConfig
    name = "template_leader"

    def __init__(self, config: TemplateLeaderConfig):
        super().__init__(config)
        self.config = config

        self._is_connected = False
        self._is_calibrated = True

        self.count = 0
        self.position = 0.0

    @property
    def action_features(self) -> dict[str, type]:
        return {"position": float}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return self._is_calibrated

    def connect(self, calibrate: bool = True) -> None:
        self._is_connected = True

    def calibrate(self) -> None:
        return

    def configure(self) -> None:
        return

    def disconnect(self) -> None:
        self._is_connected = False

    def _get_action(self) -> None:
        self.count += 1
        self.position = math.sin(self.count / 60.0 / 10.0 * 180 / 3.1415)

    def get_action(self) -> dict[str, Any]:
        self._get_action()
        return {"position": float(self.position)}

    def send_feedback(self, feedback: dict[str, float]) -> None:
        return

