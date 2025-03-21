import copy
from dataclasses import asdict, dataclass

from body.stretch_body.models.status_base import StatusBase
from body.stretch_body.models.status_dynamixel import StatusDynamixel
from body.stretch_body.models.status_pimu_base import StatusPimuBase
from body.stretch_body.models.status_prismatic_joint import StatusPrismaticJoint
from body.stretch_body.models.status_wacc import StatusWacc


@dataclass
class StatusRobot:
    pimu: StatusPimuBase
    base: StatusBase
    lift: StatusPrismaticJoint
    arm: StatusPrismaticJoint
    head: dict[str, StatusDynamixel]
    wacc: StatusWacc
    end_of_arm: dict
    # For backwards compatibility, allow users to access this with square brackets:
    def __getitem__(self, key):
        return getattr(self, key)

    @classmethod
    def fromDict(cls, json: dict):
        status = cls(**json)

        return status
    
    def toDict(self):
        return asdict(self)

    def copy(self):
        return copy.copy( StatusRobot.fromDict(self.toDict()))
