from dataclasses import dataclass, field
from typing import Dict, List
from lerobot.teleoperators.config import TeleoperatorConfig

@TeleoperatorConfig.register_subclass("group_leader")
@dataclass
class GroupLeaderConfig(TeleoperatorConfig):

    teleops: Dict[str, TeleoperatorConfig] = field(default_factory=dict)