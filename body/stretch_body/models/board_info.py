from dataclasses import dataclass


@dataclass
class BoardInfo:
    board_variant: str | None
    firmware_version: str | None
    protocol_version: str | None
    hardware_id: int
    stepper_type: int|None

    # For backwards compatibility, allow users to access this with square brackets:
    def __getitem__(self, key):
        return getattr(self, key)

    @classmethod
    def fromDict(cls, json: dict):
        status = cls(**json)

        return status

    @classmethod
    def init(cls):
        return cls.fromDict(
            {
                "board_variant": None,
                "firmware_version": None,
                "protocol_version": None,
                "hardware_id": 0,
                "stepper_type": 0,
            }
        )
