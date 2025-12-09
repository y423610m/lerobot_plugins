from dataclasses import dataclass, field
from typing import Dict
from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig

from time import sleep

@dataclass
class MotorConfig:
    pwm: int = None,
    forward: int = None,
    backward: int = None,

@RobotConfig.register_subclass("four_mecanum_wheels_car_follower")
@dataclass
class FourMecanumWheelsCarFollowerConfig(RobotConfig):

    cameras: Dict[str, CameraConfig] = field(default_factory=dict)

    motors = None

    rf: MotorConfig = None
    lf: MotorConfig = None
    rb: MotorConfig = None
    lb: MotorConfig = None

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