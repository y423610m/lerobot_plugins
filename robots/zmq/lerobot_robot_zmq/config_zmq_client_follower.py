from dataclasses import dataclass, field
from typing import Dict
from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("zmq_client_follower")
@dataclass
class ZMQClientFollowerConfig(RobotConfig):
    # Network Configuration
    remote_ip: str
    port_zmq_cmd: int = 5555
    port_zmq_observations: int = 5556

    # teleop_keys: dict[str, str] = field(
    #     default_factory=lambda: {
    #         # Movement
    #         "forward": "w",
    #         "backward": "s",
    #         "left": "a",
    #         "right": "d",
    #         "rotate_left": "z",
    #         "rotate_right": "x",
    #         # Speed control
    #         "speed_up": "r",
    #         "speed_down": "f",
    #         # quit teleop
    #         "quit": "q",
    #     }
    # )

    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    # robot: RobotConfig = None

    polling_timeout_ms: int = 15
    connect_timeout_s: int = 5