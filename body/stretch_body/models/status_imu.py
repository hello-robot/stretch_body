from dataclasses import asdict, dataclass


@dataclass
class StatusImu:
    ax:float
    ay:float
    az:float
    gx:float
    gy:float
    gz:float
    mx:float
    my:float
    mz:float
    roll:float
    pitch:float
    heading:float
    timestamp:float
    qw:float
    qx:float
    qy:float
    qz:float
    bump:float

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
        return StatusImu.fromDict(self.toDict())

    @classmethod
    def init(cls):
        return cls.fromDict(
            {
                "ax": 0,
                "ay": 0,
                "az": 0,
                "gx": 0,
                "gy": 0,
                "gz": 0,
                "mx": 0,
                "my": 0,
                "mz": 0,
                "roll": 0,
                "pitch": 0,
                "heading": 0,
                "timestamp": 0,
                "qw": 0,
                "qx": 0,
                "qy": 0,
                "qz": 0,
                "bump": 0,
            }
        )
