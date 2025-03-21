from dataclasses import dataclass

from body.stretch_body.models.status_stepper import StatusStepper


@dataclass
class StatusBase:
    timestamp_pc: float
    x: float
    y: float
    theta: float
    x_vel: float
    y_vel: float
    theta_vel: float
    pose_time_s: float
    effort: tuple[float, float]
    left_wheel: StatusStepper
    right_wheel: StatusStepper
    translation_force: float
    rotation_torque: float

    # For backwards compatibility, allow users to access this with square brackets:
    def __getitem__(self, key):
        return getattr(self, key)

    @classmethod
    def fromDict(cls, json: dict):
        status = cls(**json)

        if not isinstance(status.left_wheel, StatusStepper):
            status.left_wheel = StatusStepper.fromDict(status.left_wheel)
        if not isinstance(status.right_wheel, StatusStepper):
            status.right_wheel = StatusStepper.fromDict( status.right_wheel)

        return status

    @classmethod
    def init(cls, left_wheel: StatusStepper, right_wheel: StatusStepper):
        return cls.fromDict(
            {
                "timestamp_pc": 0.0,
                "x": 0.0,
                "y": 0.0,
                "theta": 0.0,
                "x_vel": 0.0,
                "y_vel": 0.0,
                "theta_vel": 0.0,
                "pose_time_s": 0.0,
                "effort": (0.0, 0.0),
                "left_wheel": left_wheel,
                "right_wheel": right_wheel,
                "translation_force": 0.0,
                "rotation_torque": 0.0,
            }
        )
