from dataclasses import asdict, dataclass

from body.stretch_body.models.status_imu import StatusImu
from body.stretch_body.models.status_transport import StatusTransport


@dataclass
class StatusPimuBase:
    
    voltage: float
    current: float
    temp: float
    cpu_temp: float
    cliff_range: list[float]
    frame_id: float
    timestamp: float
    at_cliff: list[bool]
    runstop_event: bool
    bump_event_cnt: float
    cliff_event: bool
    fan_on: bool
    buzzer_on: bool
    low_voltage_alert: bool
    high_current_alert: bool
    over_tilt_alert: bool
    charger_connected: bool
    boot_detected: bool
    imu: StatusImu
    debug: float
    state: int
    trace_on: float
    motor_sync_rate: float
    motor_sync_cnt: float
    motor_sync_queues: float
    motor_sync_drop: float
    transport: StatusTransport
    current_charge: float
    charger_is_charging: bool
    over_tilt_type: float

    # For backwards compatibility, allow users to access this with square brackets:
    def __getitem__(self, key):
        return getattr(self, key)

    @classmethod
    def fromDict(cls, json: dict):
        status = cls(**json)

        if not isinstance(status.transport, StatusTransport):
            status.transport = StatusTransport.fromDict(status.transport)

        if not isinstance(status.imu, StatusImu):
            status.imu = StatusImu.fromDict(status.imu)

        return status

    def toDict(self):
        return asdict(self)

    def copy(self):
        return StatusPimuBase.fromDict(self.toDict())
    
    @classmethod
    def init(cls, status_imu: StatusImu, status_transport: StatusTransport):
        return cls.fromDict(
            {
                "voltage": 0,
                "current": 0,
                "temp": 0,
                "cpu_temp": 0,
                "cliff_range": [0, 0, 0, 0],
                "frame_id": 0,
                "timestamp": 0,
                "at_cliff": [False, False, False, False],
                "runstop_event": False,
                "bump_event_cnt": 0,
                "cliff_event": False,
                "fan_on": False,
                "buzzer_on": False,
                "low_voltage_alert": False,
                "high_current_alert": False,
                "over_tilt_alert": False,
                "charger_connected": False,
                "boot_detected": False,
                "imu": status_imu,
                "debug": 0,
                "state": 0,
                "trace_on": 0,
                "motor_sync_rate": 0,
                "motor_sync_cnt": 0,
                "motor_sync_queues": 0,
                "motor_sync_drop": 0,
                "transport": status_transport,
                "current_charge": 0,
                "charger_is_charging": False,
                "over_tilt_type": 0,
            }
        )
