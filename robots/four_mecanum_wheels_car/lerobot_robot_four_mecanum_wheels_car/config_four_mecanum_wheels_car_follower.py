from dataclasses import dataclass, field
from typing import Dict
from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig

from time import sleep

@RobotConfig.register_subclass("four_mecanum_wheels_car_follower")
@dataclass
class FourMecanumWheelsCarFollowerConfig(RobotConfig):

    cameras: Dict[str, CameraConfig] = field(default_factory=dict)

    motors = None

    channles_settings = {
        "rf": {
            "pwm": 18,
            "forward": 27,
            "backward": 22,
        },
        # "lf": {
        #     "pwm": 17,
        #     "forward": 27,
        #     "backward": 22,
        # },
    }