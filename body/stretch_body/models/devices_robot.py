from dataclasses import dataclass

from body.stretch_body.arm import Arm
from body.stretch_body.base import Base
from body.stretch_body.head import Head
from body.stretch_body.lift import Lift
from body.stretch_body.pimu import Pimu
from body.stretch_body.wacc import WaccBase


@dataclass
class DevicesRobot:
    pimu: Pimu
    base: Base
    lift: Lift
    arm: Arm
    head: Head
    wacc: WaccBase
    end_of_arm: type

    # For backwards compatibility, allow users to access this with square brackets:
    def __getitem__(self, key):
        return getattr(self, key)

    @classmethod
    def fromDict(cls, json: dict):
        status = cls(**json)

        return status