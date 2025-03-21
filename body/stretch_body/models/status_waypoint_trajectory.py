from dataclasses import dataclass


@dataclass
class StatusWaypointTrajectory:
    state: str
    setpoint: tuple[float, float, float] | None
    segment_id: int

    # For backwards compatibility, allow users to access this with square brackets:
    def __getitem__(self, key):
        return getattr(self, key)

    @classmethod
    def fromDict(cls, json: dict):
        return cls(**json)

    @classmethod
    def init(cls):
        return cls.fromDict(
            {
                "state": "idle",
                "setpoint": None,
                "segment_id": 0,
            }
        )
