from dataclasses import asdict, dataclass

from body.stretch_body.models.status_transport import StatusTransport


@dataclass
class StatusWacc:
    ax: float
    ay: float
    az: float
    a0: float
    d0: float
    d1: float
    d2: float
    d3: float
    single_tap_count: float
    state: float
    debug: float
    timestamp: float
    trace_on: float
    transport: StatusTransport

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
        return StatusWacc.fromDict(self.toDict())

    @classmethod
    def init(cls, transport_status: StatusTransport):
        return cls.fromDict(
            {
                "ax": 0,
                "ay": 0,
                "az": 0,
                "a0": 0,
                "d0": 0,
                "d1": 0,
                "d2": 0,
                "d3": 0,
                "single_tap_count": 0,
                "state": 0,
                "debug": 0,
                "timestamp": 0,
                "trace_on": 0,
                "transport": transport_status,
            }
        )
