from dataclasses import dataclass
from lerobot.teleoperators.config import TeleoperatorConfig

@TeleoperatorConfig.register_subclass("template_leader")
@dataclass
class TemplateLeaderConfig(TeleoperatorConfig):

    param1 = 1