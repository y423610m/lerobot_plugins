from dataclasses import dataclass, field
from typing import Dict
from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("template_follower")
@dataclass
class TemplateFollowerConfig(RobotConfig):

    cameras: Dict[str, CameraConfig] = field(default_factory=dict)

    param1 = 1