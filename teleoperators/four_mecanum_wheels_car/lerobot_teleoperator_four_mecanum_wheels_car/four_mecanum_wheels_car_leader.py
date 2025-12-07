from __future__ import annotations
from dataclasses import dataclass
from typing import Any
from functools import cached_property

import math

from lerobot.teleoperators.teleoperator import Teleoperator

from .config_four_mecanum_wheels_car_leader import FourMecanumWheelsCarLeaderConfig

from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop

class FourMecanumWheelsCarLeader(KeyboardTeleop):
    config_class = FourMecanumWheelsCarLeaderConfig
    name = "four_mecanum_wheels_car_leader"

    def __init__(self, config: FourMecanumWheelsCarLeaderConfig):
        super().__init__(config)
        self.config = config

    @cached_property
    def _state_ft(self) -> dict[str, type]:
        return dict.fromkeys(
            (
                "x.vel",
                "y.vel",
                "theta.vel",
            ),
            float,
        )

    @property
    def action_features(self) -> dict[str, type]:
        return self._state_ft

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    def get_action(self) -> dict[str, Any]:
        keys = super().get_action()  # return {key, None} if pressed.
        print(f"{keys=}")
        action = {
            "x.vel": float(0.0),
            "y.vel": float(0.0),
            "theta.vel": float(0.0),
        }

        if "w" in keys:
            action["x.vel"] += 1.0
        if "s" in keys:
            action["x.vel"] -= 1.0
        if "d" in keys:
            action["y.vel"] += 1.0
        if "a" in keys:
            action["y.vel"] -= 1.0
        
        return action

    # def _on_press(self, key):
        # if hasattr(key, "char"):
            # print(f"{key.name=} {key.value=}")
            # self.event_queue.put((key.char, True))
        # else:
            # print(f"{key.name=} {key.value=}")
