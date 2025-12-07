from dataclasses import dataclass
from lerobot.teleoperators.config import TeleoperatorConfig

@TeleoperatorConfig.register_subclass("four_mecanum_wheels_car_leader")
@dataclass
class FourMecanumWheelsCarLeaderConfig(TeleoperatorConfig):

    param1 = 1