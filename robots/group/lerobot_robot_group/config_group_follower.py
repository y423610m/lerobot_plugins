from dataclasses import dataclass, field
from typing import Dict, List
from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig

from time import sleep

@RobotConfig.register_subclass("group_follower")
@dataclass
class GroupFollowerConfig(RobotConfig):

    cameras: Dict[str, CameraConfig] = field(default_factory=dict)

    robots: Dict[str, RobotConfig] = field(default_factory=dict)