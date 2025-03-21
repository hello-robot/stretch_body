from dataclasses import dataclass

from body.stretch_body.models.status_stepper import StatusStepper


@dataclass
class StatusPrismaticJoint:
    timestamp_pc: float
    pos: float
    vel: float
    force: float
    motor: StatusStepper


    # For backwards compatibility, allow users to access this with square brackets:
    def __getitem__(self, key):
        return getattr(self, key)

    @classmethod
    def fromDict(cls, json: dict):
        status = cls(**json)

        if not isinstance(status.motor, StatusStepper):
            status.motor = StatusStepper.fromDict(status.motor)

        return status

    @classmethod
    def init(cls, motor_status:StatusStepper):
        return cls.fromDict(
            {
                "timestamp_pc": 0,
                "pos": 0.0,
                "vel": 0.0,
                "force": 0.0,
                "motor": motor_status
            }
        )
